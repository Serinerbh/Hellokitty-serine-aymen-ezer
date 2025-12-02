#include "ydlidar.h"
#include "usart.h"
#include <string.h>
#include <stdio.h>
#include <math.h>

#define MAX_PACKET_SIZE 515

typedef enum {
    STATE_WAIT_START_BYTE1, // Waiting for 0xAA
    STATE_WAIT_START_BYTE2, // Waiting for 0x55
    STATE_RECEIVE_HEADER,   // Receiving CT, LSN, FSA, LSA, CS
    STATE_RECEIVE_SAMPLES   // Receiving sample data
} parsing_state_t;

static parsing_state_t current_parsing_state = STATE_WAIT_START_BYTE1;
static uint8_t current_packet_buffer[MAX_PACKET_SIZE];
static uint16_t current_packet_idx = 0;
static uint16_t expected_packet_len = 0;


static void decode_packet(const uint8_t* packet_data, uint16_t packet_len);

void ydlidar_init(void) {
    current_parsing_state = STATE_WAIT_START_BYTE1;
    current_packet_idx = 0;
    expected_packet_len = 0;
    printf("YDLIDAR driver initialized.\r\n");
}

void ydlidar_process_data(const uint8_t* data, size_t len) {
    for (size_t i = 0; i < len; ++i) {
        uint8_t byte = data[i];

        switch (current_parsing_state) {
            case STATE_WAIT_START_BYTE1:
                if (byte == 0xAA) {
                    current_packet_buffer[0] = byte;
                    current_parsing_state = STATE_WAIT_START_BYTE2;
                }
                break;

            case STATE_WAIT_START_BYTE2:
                if (byte == 0x55) {
                    current_packet_buffer[1] = byte;
                    current_packet_idx = 2; // PH (2 bytes) received
                    current_parsing_state = STATE_RECEIVE_HEADER;
                    expected_packet_len = 0;
                } else {
                    // If 0x55 is not received, reset state and look for 0xAA again
                    current_parsing_state = STATE_WAIT_START_BYTE1;
                }
                break;

            case STATE_RECEIVE_HEADER:
                current_packet_buffer[current_packet_idx++] = byte;
                if (current_packet_idx >= (2 + 8)) {
                    uint8_t lsn = current_packet_buffer[3];
                    expected_packet_len = 10 + (lsn * 2);
                    
                    if (expected_packet_len > MAX_PACKET_SIZE) {
                        printf("Error: Packet too large (%d bytes). Resetting.\r\n", expected_packet_len);
                        current_parsing_state = STATE_WAIT_START_BYTE1;
                        current_packet_idx = 0;
                        expected_packet_len = 0;
                        break;
                    }
                    current_parsing_state = STATE_RECEIVE_SAMPLES;
                }
                break;

            case STATE_RECEIVE_SAMPLES:
                current_packet_buffer[current_packet_idx++] = byte;

                if (current_packet_idx >= expected_packet_len) {
                    decode_packet(current_packet_buffer, expected_packet_len);

                    current_parsing_state = STATE_WAIT_START_BYTE1;
                    current_packet_idx = 0;
                    expected_packet_len = 0;
                }
                break;
        }
    }
}


static void decode_packet(const uint8_t* packet_data, uint16_t packet_len) {
    // Checksum validation
    uint16_t calculated_checksum = 0;
    
    // Calculate checksum over PH(2), CT(1)+LSN(1), FSA(2), LSA(2)
    for (int i = 0; i < 8; i += 2) {
        uint16_t word = packet_data[i] | (packet_data[i+1] << 8);
        calculated_checksum ^= word;
    }
    // Calculate checksum over Samples (starting at byte 10)
    for (int i = 10; i < packet_len; i += 2) {
        uint16_t word = packet_data[i] | (packet_data[i+1] << 8);
        calculated_checksum ^= word;
    }
    
    lidar_response_point_cloud_t* response = (lidar_response_point_cloud_t*)&packet_data[2];
    uint16_t received_checksum = response->check_code;

    if (calculated_checksum != received_checksum) {
        printf("Checksum error: Calculated 0x%04X, Received 0x%04X. Packet discarded.\r\n", calculated_checksum, received_checksum);
        return; // Discard packet
    }

    printf("--- Decoded Lidar Packet ---\r\n");
    printf("Packet Type (CT): 0x%02X\r\n", response->packet_type);
    if (response->packet_type & 0x01) { // Check CT[bit(0)]
        printf("  (This is a Start of Scan packet)\r\n");
    } else {
        printf("  (This is a Point Cloud Data packet)\r\n");
    }
    printf("Sample Quantity (LSN): %d\r\n", response->sample_quantity);

    float start_angle_deg = ((float)(response->start_angle >> 1) / 64.0f); // Rshiftbit(FSA,1)/64
    float end_angle_deg = ((float)(response->end_angle >> 1) / 64.0f); // Rshiftbit(LSA,1)/64

    // Adjust angles to be within 0-360 range
    if (start_angle_deg > 360.0f) start_angle_deg -= 360.0f;
    if (end_angle_deg > 360.0f) end_angle_deg -= 360.0f;

    printf("Start Angle (FSA): %.2f degrees\r\n", start_angle_deg);
    printf("End Angle (LSA): %.2f degrees\r\n", end_angle_deg);

    printf("Checksum (CS): 0x%04X (Validated)\r\n", received_checksum);

    if (response->sample_quantity > 0) {
        printf("Samples:\r\n");
        float diff_angle_deg = 0;

        if (response->sample_quantity > 1) {
            diff_angle_deg = end_angle_deg - start_angle_deg;
            if (diff_angle_deg < 0) {
                diff_angle_deg += 360.0f;
            }
        }

        for (int i = 0; i < response->sample_quantity; i++) {
            uint16_t raw_distance = response->samples[i];
            float distance_mm = (float)raw_distance / 4.0f;
            
            float current_angle_deg;
            if (response->sample_quantity == 1) {
                current_angle_deg = start_angle_deg;
            } else {
                current_angle_deg = start_angle_deg + (diff_angle_deg / (response->sample_quantity - 1)) * i;
            }

            // Second-level analysis
            float ang_correct = 0.0f;
            if (distance_mm > 0.0f) {
                ang_correct = atanf(21.8f * (155.3f - distance_mm) / (155.3f * distance_mm)) * (180.0f / M_PI); // Convert radians to degrees
            }
            current_angle_deg += ang_correct;

            if (current_angle_deg > 360.0f) current_angle_deg -= 360.0f;
            else if (current_angle_deg < 0.0f) current_angle_deg += 360.0f;

            printf("  - Sample %d: Angle=%.2f deg, Distance=%.2f mm\r\n", i, current_angle_deg, distance_mm);
        }
    }
    printf("--------------------------\r\n\r\n");
}
