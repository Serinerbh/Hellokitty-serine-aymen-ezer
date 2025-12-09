/*
 * motor.h
 *
 *  Created on: Dec 2, 2025
 *      Author: Aymen
 */

#ifndef __MOTOR_H
#define __MOTOR_H

#include "main.h" // Inclut les définitions HAL, TIM_HandleTypeDef, etc.

// Déclaration du handle du Timer PWM (déclaré dans main.c ou ailleurs)
extern TIM_HandleTypeDef htim3;

// Canaux PWM utilisés par votre TIM3 pour le contrôle moteur
#define MOTOR_CH_A_FWD  TIM_CHANNEL_2 // PA7
#define MOTOR_CH_A_REV  TIM_CHANNEL_1 // PA6
#define MOTOR_CH_B_FWD  TIM_CHANNEL_3 // PB0
#define MOTOR_CH_B_REV  TIM_CHANNEL_4 // PB1

/* --- Prototypes des fonctions --- */

// Initialise et démarre les canaux PWM pour le contrôle moteur
void MOTOR_Init(void);

// Régle la vitesse et la direction du moteur 1 (exemple)
void MOTOR_Set_Speed_A(int16_t speed);

// Régle la vitesse et la direction du moteur 2 (exemple)
void MOTOR_Set_Speed_B(int16_t speed);


#endif /* __MOTOR_H */
