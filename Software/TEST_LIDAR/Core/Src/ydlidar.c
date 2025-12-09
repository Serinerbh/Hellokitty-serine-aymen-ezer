#include "ydlidar.h"
#include "usart.h"
#include <string.h>
#include <stdio.h>
#include <math.h>

#define ENABLE_LIDAR_DEBUG // Uncomment to enable debug output via printf

#ifdef ENABLE_LIDAR_DEBUG
    #define LIDAR_LOG(...) printf(__VA_ARGS__)
#else
    #define LIDAR_LOG(...) ((void)0)
#endif

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
static uint16_t g_scan_distances_mm[NB_DEGRES] = {0}; // Buffer circulaire 360 deg


static void decode_packet(const uint8_t* packet_data, uint16_t packet_len);

void ydlidar_init(void) {
    current_parsing_state = STATE_WAIT_HEADER;
    current_packet_idx = 0;
    expected_packet_len = 0;
    memset(g_scan_distances_mm, 0, sizeof(g_scan_distances_mm));
    LIDAR_LOG("YDLIDAR driver initialized.\r\n");
}

static uint8_t last_byte = 0; // Stores the previously received byte

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
                        LIDAR_LOG("Error: Packet too large (%d bytes). Resetting.\r\n", expected_packet_len);
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
        LIDAR_LOG("Checksum error: Calculated 0x%04X, Received 0x%04X. Packet discarded.\r\n", calculated_checksum, received_checksum);
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
            int index = (int)(current_angle_deg + 0.5f); // Round to nearest int
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
    float sum_angle = 0; // Warning: Simple average works for small objects not crossing 0/360

    for (int i = 1; i < NB_DEGRES; i++) {
        uint16_t dist_prev = g_scan_distances_mm[i - 1];
        uint16_t dist_curr = g_scan_distances_mm[i];

        // Ignorer les points nuls (non mesurés)
        if (dist_curr == 0) continue;
        if (dist_prev == 0) {
            sum_dist = dist_curr;
            sum_angle = i;
            points_in_object = 1;
            continue;
        }

        // Check discontinuity
        if (fabsf((float)dist_curr - (float)dist_prev) > DETECT_THRESHOLD) {
            // End of an object, save it if valid
            if (points_in_object > 1 && *object_count < MAX_LIDAR_OBJECTS) {
                objects[*object_count].distance = sum_dist / points_in_object;
                objects[*object_count].angle = sum_angle / points_in_object;
                objects[*object_count].size = points_in_object;
                
                // Filter by distance
                if (objects[*object_count].distance <= MAX_DETECTION_DISTANCE_MM) {
                    LIDAR_LOG("Detected Object %d: Angle=%.2f, Dist=%.2f, Size=%d\r\n", *object_count, objects[*object_count].angle, objects[*object_count].distance, objects[*object_count].size);
                    (*object_count)++;
                }
            }
            
            // Start new object
            sum_dist = dist_curr;
            sum_angle = i;
            points_in_object = 1;
        } else {
            // Continue object
            sum_dist += dist_curr;
            sum_angle += i;
            points_in_object++;
        }
    }
    
    // Check last segment
    if (points_in_object > 1 && *object_count < MAX_LIDAR_OBJECTS) {
         objects[*object_count].distance = sum_dist / points_in_object;
         objects[*object_count].angle = sum_angle / points_in_object;
         objects[*object_count].size = points_in_object;
         
         // Filter by distance
         if (objects[*object_count].distance <= MAX_DETECTION_DISTANCE_MM) {
            LIDAR_LOG("Detected Object %d: Angle=%.2f, Dist=%.2f, Size=%d\r\n", *object_count, objects[*object_count].angle, objects[*object_count].distance, objects[*object_count].size);
            (*object_count)++;
         }
    }
    LIDAR_LOG("Total Objects Detected: %d\r\n", *object_count);
}

