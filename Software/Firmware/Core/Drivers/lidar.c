/*
 * LIDAR_Driver.c
 *
 */

#include "lidar.h"
#include "usart.h"
#include <string.h>
#include <stdio.h>
#include <math.h>

#define MAX_PACKET_SIZE 515

typedef enum {
    STATE_WAIT_HEADER,      // Waiting for 0xAA 0x55 sequence
    STATE_RECEIVE_HEADER,   // Receiving CT, LSN, FSA, LSA, CS
    STATE_RECEIVE_SAMPLES   // Receiving sample data
} parsing_state_t;

static parsing_state_t current_parsing_state = STATE_WAIT_HEADER;
static uint8_t current_packet_buffer[MAX_PACKET_SIZE];
static uint16_t current_packet_idx = 0;
static uint16_t expected_packet_len = 0;
static uint16_t g_scan_distances_mm[NB_DEGRES] = {0};


static void decode_packet(const uint8_t* packet_data, uint16_t packet_len);

void ydlidar_init(uint8_t *buffer, uint16_t size) {
    current_parsing_state = STATE_WAIT_HEADER;
    current_packet_idx = 0;
    expected_packet_len = 0;
    memset(g_scan_distances_mm, 0, sizeof(g_scan_distances_mm));
    
    __HAL_UART_CLEAR_OREFLAG(&huart2);
    __HAL_UART_CLEAR_NEFLAG(&huart2);
    __HAL_UART_CLEAR_FEFLAG(&huart2);

    if (HAL_UART_Receive_DMA(&huart2, buffer, size) != HAL_OK) {
      printf("LIDAR DMA Error\r\n");
    }
    printf("YDLIDAR driver initialized.\r\n");
}

static uint8_t last_byte = 0;

void ydlidar_process_data(const uint8_t* data, size_t len) {
    for (size_t i = 0; i < len; ++i) {
        uint8_t byte = data[i];

        switch (current_parsing_state) {
            case STATE_WAIT_HEADER:
                if (last_byte == 0xAA && byte == 0x55) {
                    current_packet_buffer[0] = 0xAA;
                    current_packet_buffer[1] = 0x55;
                    current_packet_idx = 2; // PH (2 bytes) received
                    current_parsing_state = STATE_RECEIVE_HEADER;
                    expected_packet_len = 0; // Reset expected length
                }
                break;

            case STATE_RECEIVE_HEADER:
                current_packet_buffer[current_packet_idx++] = byte;
                if (current_packet_idx >= (2 + 8)) {
                    uint8_t lsn = current_packet_buffer[3];
                    expected_packet_len = 10 + (lsn * 2);

                    if (expected_packet_len > MAX_PACKET_SIZE) {
                    	printf("Error: Packet too large (%d bytes). Resetting.\r\n", expected_packet_len);
                        current_parsing_state = STATE_WAIT_HEADER;
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

                    current_parsing_state = STATE_WAIT_HEADER;
                    current_packet_idx = 0;
                    expected_packet_len = 0;
                }
                break;
        }
        last_byte = byte; // Store the current byte for the next iteration
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

    float start_angle_deg = ((float)(response->start_angle >> 1) / 64.0f); // Rshiftbit(FSA,1)/64
    float end_angle_deg = ((float)(response->end_angle >> 1) / 64.0f); // Rshiftbit(LSA,1)/64

    // Adjust angles to be within 0-360 range
    if (start_angle_deg > 360.0f) start_angle_deg -= 360.0f;
    if (end_angle_deg > 360.0f) end_angle_deg -= 360.0f;

    if (response->sample_quantity > 0) {
        // printf("Samples:\r\n");
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

            // Second-level analysis (Geometric Correction)
            float ang_correct = 0.0f;
            if (distance_mm > 0.0f) {
                ang_correct = atanf(21.8f * (155.3f - distance_mm) / (155.3f * distance_mm)) * (180.0f / M_PI);
            }
            current_angle_deg += ang_correct;

            if (current_angle_deg >= 360.0f) current_angle_deg -= 360.0f;
            else if (current_angle_deg < 0.0f) current_angle_deg += 360.0f;

            // Store in 360-degree buffer
            int index = (int)(current_angle_deg + 0.5f);
            if (index >= NB_DEGRES) index = 0;

            if (distance_mm > 0) {
                 g_scan_distances_mm[index] = (uint16_t)distance_mm;
            }
        }
    }
}

uint16_t ydlidar_get_distance(uint16_t angle_deg) {
    if (angle_deg >= NB_DEGRES) return 0;
    return g_scan_distances_mm[angle_deg];
}

void ydlidar_detect_objects(LidarObject_t* objects, uint8_t* object_count) {
    *object_count = 0;
    int points_in_object = 0;
    float sum_dist = 0;
    float start_angle = -1;
    float end_angle = -1;

    for (int i = 0; i < NB_DEGRES; i++) {
        uint16_t dist_curr = g_scan_distances_mm[i];

        if (dist_curr == 0) {
            // No data at this angle, treat as potential break
            if (points_in_object > 0) {
                // Object ended by lack of data
                goto finalize_object;
            }
            continue;
        }

        uint16_t dist_prev = (i > 0) ? g_scan_distances_mm[i - 1] : 0;

        // Discontinuity check
        if (points_in_object > 0 && fabsf((float)dist_curr - (float)dist_prev) > DETECT_THRESHOLD) {
            finalize_object: ;
            // Calculate width and filter
            float avg_dist = sum_dist / points_in_object;
            float angular_width = (float)points_in_object; // Simplified as 1 point approx 1 deg
            float width_mm = 2.0f * avg_dist * tanf((angular_width * M_PI / 180.0f) / 2.0f);

            if (width_mm >= MIN_OBJECT_WIDTH_MM && width_mm <= MAX_OBJECT_WIDTH_MM &&
                avg_dist <= MAX_DETECTION_DISTANCE_MM && points_in_object >= 2) {
                
                if (*object_count < MAX_LIDAR_OBJECTS) {
                    objects[*object_count].distance = avg_dist;
                    // Angle average (simplified, doesn't handle 0/360 wrap here)
                    objects[*object_count].angle = (start_angle + (float)(i-1)) / 2.0f;
                    objects[*object_count].width_mm = width_mm;
                    objects[*object_count].size = points_in_object;
                    
                    printf("Detected Robot %d: Ang=%.1f, Dist=%.1f, Width=%.1f\r\n", 
                            *object_count, objects[*object_count].angle, 
                            objects[*object_count].distance, objects[*object_count].width_mm);
                    (*object_count)++;
                }
            }
            // Reset for next object
            points_in_object = 0;
            sum_dist = 0;
            if (dist_curr == 0) continue; 
        }

        // Start or continue object
        if (points_in_object == 0) start_angle = (float)i;
        sum_dist += (float)dist_curr;
        points_in_object++;
    }
    printf("Total Potential Targets: %d\r\n", *object_count);
}
