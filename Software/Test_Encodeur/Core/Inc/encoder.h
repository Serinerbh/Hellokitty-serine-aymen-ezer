/*
 * ecodeur.h
 *
 *  Created on: Dec 2, 2025
 *      Author: Aymen
 */

#ifndef __ENCODER_H
#define __ENCODER_H

#include "main.h" // Inclut les définitions HAL, TIM_HandleTypeDef, etc.

// Déclaration des handles de Timers pour les encodeurs (déclarés dans main.c ou ailleurs)
extern TIM_HandleTypeDef htim2; // Encodeur 2 (32-bit)
extern TIM_HandleTypeDef htim4; // Encodeur 1 (16-bit)
extern TIM_HandleTypeDef htim6; // Timer d'échantillonnage

/* --- Paramètres et Variables de Vitesse --- */

// Paramètres de l'encodeur (doivent correspondre à vos encodeurs physiques)
#define ENCODER_PPR     2048.0f     // Ticks (impulsions) par Révolution
#define DELTA_T         0.01f       // Période d'échantillonnage en secondes (10 ms pour TIM6)

// Variables de vitesse globales
extern volatile float rpm_motor1;
extern volatile float rad_s_motor1;
extern volatile float rpm_motor2;
extern volatile float rad_s_motor2;

/* --- Prototypes des fonctions --- */

// Initialise et démarre les timers encodeurs et le timer d'échantillonnage
void ENCODER_Init(void);

// Réinitialise les compteurs et variables internes
void ENCODER_Reset(void);

// Fonction de Callback qui doit être appelée par le système HAL dans main.c
void ENCODER_Speed_Calculation(TIM_HandleTypeDef *htim);


#endif /* __ENCODER_H */
