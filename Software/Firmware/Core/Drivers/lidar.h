/*
 * LIDAR_Driver.h
 *
 */

#ifndef LIDAR_H
#define LIDAR_H

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

#define NB_DEGRES 360
#define MAX_LIDAR_OBJECTS 20
#define DETECT_THRESHOLD 50 // Seuil de discontinuité en mm
#define MAX_DETECTION_DISTANCE_MM 2000 // Objets au-delà de 2 mètres seront ignorés

#define MIN_OBJECT_WIDTH_MM        50.0f   // Un robot fait au moins 5cm
#define MAX_OBJECT_WIDTH_MM        300.0f  // Un robot fait moins de 30cm

typedef struct {
    float angle;        // Angle moyen de l'objet (degrés)
    float distance;     // Distance moyenne (mm)
    float width_mm;     // Largeur physique en mm
    int size;           // Nombre de points
} LidarObject_t;

#define LIDAR_DMA_BUFFER_SIZE 1024

void ydlidar_init(uint8_t *buffer, uint16_t size);

void ydlidar_process_data(const uint8_t* data, size_t len);

// Nouvelle fonction de segmentation
void ydlidar_detect_objects(LidarObject_t* objects, uint8_t* object_count);
// Accès aux données brutes pour debug
uint16_t ydlidar_get_distance(uint16_t angle_deg);

#endif
