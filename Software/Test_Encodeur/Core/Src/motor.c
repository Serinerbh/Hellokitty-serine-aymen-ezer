/*
 * motor.c
 *
 *  Created on: Dec 2, 2025
 *      Author: Aymen
 */

#include "motor.h"

// Définition de la période PWM (doit correspondre à htim3.Init.Period dans CubeMX)
#define PWM_PERIOD 1000

/**
  * @brief  Initialise et démarre les canaux PWM.
  * @param  None
  * @retval None
  */
void MOTOR_Init(void)
{
    // Démarrage des canaux PWM utilisés
    HAL_TIM_PWM_Start(&htim3, MOTOR_CH_A_REV);
    HAL_TIM_PWM_Start(&htim3, MOTOR_CH_A_FWD);
    HAL_TIM_PWM_Start(&htim3, MOTOR_CH_B_FWD);
    HAL_TIM_PWM_Start(&htim3, MOTOR_CH_B_REV);

    // Initialisation des rapports cycliques à zéro
    __HAL_TIM_SET_COMPARE(&htim3, MOTOR_CH_A_REV, 0);
    __HAL_TIM_SET_COMPARE(&htim3, MOTOR_CH_A_FWD, 0);
    __HAL_TIM_SET_COMPARE(&htim3, MOTOR_CH_B_FWD, 0);
    __HAL_TIM_SET_COMPARE(&htim3, MOTOR_CH_B_REV, 0);
}

/**
  * @brief  Régle la vitesse et la direction du Moteur 1.
  * @param  speed: Valeur entre -1000 (arrière) et 1000 (avant).
  * @retval None
  */
void MOTOR_Set_Speed_A(int16_t speed)
{
    // Limite la vitesse aux valeurs possibles [ -PWM_PERIOD ; +PWM_PERIOD ]
    if (speed > PWM_PERIOD) speed = PWM_PERIOD;
    if (speed < -PWM_PERIOD) speed = -PWM_PERIOD;

    if (speed > 0) // Avance
    {
        // Canal 'Reverse' à zéro
        __HAL_TIM_SET_COMPARE(&htim3, MOTOR_CH_A_REV, 0);
        // Canal 'Forward' à la vitesse positive
        __HAL_TIM_SET_COMPARE(&htim3, MOTOR_CH_A_FWD, speed);
    }
    else if (speed < 0) // Arrière
    {
        // Canal 'Forward' à zéro
        __HAL_TIM_SET_COMPARE(&htim3, MOTOR_CH_A_FWD, 0);
        // Canal 'Reverse' à la valeur absolue de la vitesse (positive)
        __HAL_TIM_SET_COMPARE(&htim3, MOTOR_CH_A_REV, -speed);
    }
    else // Arrêt
    {
        __HAL_TIM_SET_COMPARE(&htim3, MOTOR_CH_A_REV, 0);
        __HAL_TIM_SET_COMPARE(&htim3, MOTOR_CH_A_FWD, 0);
    }
}

/**
  * @brief  Régle la vitesse et la direction du Moteur 2.
  * @param  speed: Valeur entre -1000 (arrière) et 1000 (avant).
  * @retval None
  */
void MOTOR_Set_Speed_B(int16_t speed)
{
    // Mêmes limites que le Moteur 1
    if (speed > PWM_PERIOD) speed = PWM_PERIOD;
    if (speed < -PWM_PERIOD) speed = -PWM_PERIOD;

    if (speed > 0) // Avance
    {
        __HAL_TIM_SET_COMPARE(&htim3, MOTOR_CH_B_REV, 0);
        __HAL_TIM_SET_COMPARE(&htim3, MOTOR_CH_B_FWD, speed);
    }
    else if (speed < 0) // Arrière
    {
        __HAL_TIM_SET_COMPARE(&htim3, MOTOR_CH_B_FWD, 0);
        __HAL_TIM_SET_COMPARE(&htim3, MOTOR_CH_B_REV, -speed);
    }
    else // Arrêt
    {
        __HAL_TIM_SET_COMPARE(&htim3, MOTOR_CH_B_REV, 0);
        __HAL_TIM_SET_COMPARE(&htim3, MOTOR_CH_B_FWD, 0);
    }
}
