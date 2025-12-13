#ifndef INC_MOTOR_H_
#define INC_MOTOR_H_

#include "stm32g4xx_hal.h"

typedef struct {
    // PWM
    TIM_HandleTypeDef* pwm_timer;
    uint32_t           channel_fwd; // Canal PWM Avant (e.g., TIM_CHANNEL_1)
    uint32_t           channel_rev; // Canal PWM Arrière (e.g., TIM_CHANNEL_2)

    // Encodeur
    TIM_HandleTypeDef* enc_timer;
    int32_t            enc_prev_counter; // Stockage précédent pour delta
    uint32_t           enc_resolution;   // PPR * 4 (Mode TI1+TI2 si utilisé)

    // Données Physiques
    float              speed_rpm;
    float              speed_rad_s;
    int32_t            total_ticks;      // Odométrie absolue

    // Ramp Control
    float              current_pwm;      // Vitesse actuelle appliquée (-100.0 à 100.0)
    float              target_pwm;       // Vitesse cible demandée
    float              pwm_ramp_step;    // Incrément max par appel de UpdatePWM (ex: 1.0)
} Motor_Handle_t;

/**
 * @brief Initializes the motor driver.
 *        Starts the PWM and encoder timers.
 * @param hmotor Pointer to the Motor_Handle_t structure.
 */
void Motor_Init(Motor_Handle_t* hmotor);

/**
 * @brief Sets the target motor speed.
 *        The actual speed will ramp up/down in Motor_UpdatePWM.
 * @param hmotor Pointer to the Motor_Handle_t structure.
 * @param pwm_percent PWM duty cycle target from -100.0 (full reverse) to +100.0 (full forward).
 */
void Motor_SetSpeed(Motor_Handle_t* hmotor, float pwm_percent);

/**
 * @brief Applies the ramping logic and writes to PWM registers.
 *        Must be called periodically (e.g. 100Hz).
 * @param hmotor Pointer to the Motor_Handle_t structure.
 */
void Motor_UpdatePWM(Motor_Handle_t* hmotor);

/**
 * @brief Updates the motor speed based on encoder readings.
 *        Should be called periodically (e.g., in a control loop task).
 * @param hmotor Pointer to the Motor_Handle_t structure.
 * @param delta_time_s Time elapsed since the last update in seconds.
 */
void Motor_UpdateSpeed(Motor_Handle_t* hmotor, float delta_time_s);

#endif /* INC_MOTOR_H_ */