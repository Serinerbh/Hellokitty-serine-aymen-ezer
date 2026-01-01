#ifndef __ODOMETRY_H
#define __ODOMETRY_H

#include "stm32g4xx_hal.h"

// --- CONSTANTES PHYSIQUES (À AJUSTER) ---
#define WHEEL_DIAMETER    65.0f   // Diamètre des roues en mm
#define WHEEL_TRACK       161.0f  // Distance entre les deux roues en mm
#define WHEEL_CIRC        (WHEEL_DIAMETER * 3.14159f)

/**
 * @brief Structure de position du robot
 */
typedef struct {
    float x;      // Position X en mm
    float y;      // Position Y en mm
    float theta;  // Orientation en radians
} Odometry_t;

/**
 * @brief Initialise l'odométrie à (0,0,0)
 */
void Odom_Init(Odometry_t *odom);

/**
 * @brief Met à jour la position du robot
 * @param odom Pointeur vers la structure d'odométrie
 * @param v_left Vitesse roue gauche en rad/s
 * @param v_right Vitesse roue droite en rad/s
 * @param dt Pas de temps en secondes (ex: 0.01 pour 10ms)
 */
void Odom_Update(Odometry_t *odom, float v_left, float v_right, float dt);

/**
 * @brief Définit manuellement la position du robot
 */
void Odom_SetPosition(Odometry_t *odom, float x, float y, float theta);

#endif /* __ODOMETRY_H */
