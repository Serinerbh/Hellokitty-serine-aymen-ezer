#ifndef YDLIDAR_H
#define YDLIDAR_H

#include <stdint.h>
#include <stddef.h>

typedef struct {
    uint16_t start_sign; // 0x55AA
} lidar_response_header_t;


typedef struct {
    float angle;    // Angle in degrees
    float distance; // Distance in millimeters
} lidar_point_t;

typedef struct {
    uint8_t packet_type;      // CT: 0 for point cloud, 1 for start of scan
    uint8_t sample_quantity;  // LSN: Number of samples in this packet
    uint16_t start_angle;     // FSA
    uint16_t end_angle;       // LSA
    uint16_t check_code;      // CS
    uint16_t samples[];       // Si: Array of distance samples
} lidar_response_point_cloud_t;


void ydlidar_init(void);

void ydlidar_process_data(const uint8_t* data, size_t len);

#endif
