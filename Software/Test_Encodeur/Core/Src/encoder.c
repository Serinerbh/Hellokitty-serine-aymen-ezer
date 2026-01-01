/*
 * encodeur.c
 *
 *  Created on: Dec 2, 2025
 *      Author: Aymen
 */

#include "encoder.h"

/* --- Variables privées (locales à ce fichier) --- */

// Ces lignes sont la DÉFINITION et l'initialisation uniques des variables
volatile float rpm_motor1 = 0.0f;
volatile float rad_s_motor1 = 0.0f;
volatile float rpm_motor2 = 0.0f;
volatile float rad_s_motor2 = 0.0f;


/* --- Variables privées (locales à ce fichier) --- */

static volatile uint16_t prev_count_4 = 0;
static volatile int16_t delta_ticks_4 = 0;

static volatile uint32_t prev_count_2 = 0;
static volatile int32_t delta_ticks_2 = 0;

/* --- Implémentation des fonctions publiques --- */

/**
  * @brief  Initialise et démarre les périphériques encodeur et le timer de base de temps.
  * @param  None
  * @retval None
  */
void ENCODER_Init(void)
{
    // Démarrage des encodeurs
    HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL); // Encodeur 1 (TIM4)
    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL); // Encodeur 2 (TIM2)

    // Réinitialisation des compteurs et initialisation des variables précédentes
    // Note: __HAL_TIM_GET_COUNTER() renvoie le type approprié (16-bit pour TIM4, 32-bit pour TIM2)
    __HAL_TIM_SET_COUNTER(&htim4, 0);
    __HAL_TIM_SET_COUNTER(&htim2, 0);

    prev_count_4 = __HAL_TIM_GET_COUNTER(&htim4);
    prev_count_2 = __HAL_TIM_GET_COUNTER(&htim2);

    // Démarrage du Timer d'échantillonnage en mode Interrupt
    HAL_TIM_Base_Start_IT(&htim6);
}

/**
  * @brief  Réinitialise les compteurs encodeurs et les variables internes.
  * @param  None
  * @retval None
  */
void ENCODER_Reset(void)
{
    __HAL_TIM_SET_COUNTER(&htim4, 0);
    __HAL_TIM_SET_COUNTER(&htim2, 0);
    
    prev_count_4 = 0;
    prev_count_2 = 0;
    delta_ticks_4 = 0;
    delta_ticks_2 = 0;
    
    rpm_motor1 = 0.0f;
    rad_s_motor1 = 0.0f;
    rpm_motor2 = 0.0f;
    rad_s_motor2 = 0.0f;
}

/**
  * @brief  Effectue les calculs de vitesse pour les deux moteurs.
  * Cette fonction est appelée par HAL_TIM_PeriodElapsedCallback.
  * @param  htim: Handle du Timer déclencheur (doit être TIM6)
  * @retval None
  */
void ENCODER_Speed_Calculation(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM6)
    {
        /* --- MOTEUR 1 (TIM4 - 16-bit) --- */
        uint16_t current_count_4 = __HAL_TIM_GET_COUNTER(&htim4);

        // La soustraction (int16_t) gère le débordement 16-bit.
        delta_ticks_4 = (int16_t)(current_count_4 - prev_count_4);
        prev_count_4 = current_count_4;

        // Calcul du RPM (MOTEUR 1)
        rpm_motor1 = ((float)delta_ticks_4 / ENCODER_PPR) / DELTA_T * 60.0f;

        // Calcul des Radians par seconde (MOTEUR 1)
        rad_s_motor1 = rpm_motor1 * 0.10472f;


        /* --- MOTEUR 2 (TIM2 - 32-bit) --- */
        uint32_t current_count_2 = __HAL_TIM_GET_COUNTER(&htim2);

        // La soustraction (int32_t) gère le débordement 32-bit.
        delta_ticks_2 = (int32_t)(current_count_2 - prev_count_2);
        prev_count_2 = current_count_2;

        // Calcul du RPM (MOTEUR 2)
        rpm_motor2 = (-(float)delta_ticks_2 / ENCODER_PPR) / DELTA_T * 60.0f;

        // Calcul des Radians par seconde (MOTEUR 2)
        rad_s_motor2 = rpm_motor2 * 0.10472f;
    }
}
