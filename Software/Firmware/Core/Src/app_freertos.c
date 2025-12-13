/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : app_freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------ */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ---------------------------------------------------------- */
/* USER CODE BEGIN Includes */
#include "usart.h"
#include "i2c.h"
#include "tim.h"
#include <stdio.h>
#include "lidar.h"
#include "imu.h"
#include "tof.h"
#include "motor.h"
/* USER CODE END Includes */

/* Private typedef ----------------------------------------------------------- */
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------ */
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro ------------------------------------------------------------- */
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables --------------------------------------------------------- */
/* USER CODE BEGIN Variables */
// Task Handles
TaskHandle_t xLidarTaskHandle = NULL;
TaskHandle_t xImuTaskHandle = NULL;
TaskHandle_t xSafetyTaskHandle = NULL;
TaskHandle_t xControlTaskHandle = NULL;

// Mutex Handles
SemaphoreHandle_t xI2C1Mutex = NULL;
SemaphoreHandle_t xUARTMutex = NULL;

// Lidar Buffer
uint8_t lidar_dma_buffer[LIDAR_DMA_BUFFER_SIZE];

// Motor Handles
Motor_Handle_t hMotor1;
Motor_Handle_t hMotor2;
/* USER CODE END Variables */

/* Private function prototypes ----------------------------------------------- */
/* USER CODE BEGIN FunctionPrototypes */
void vLidarTask(void *pvParameters);
void vImuTask(void *pvParameters);
void vSafetyTask(void *pvParameters);
void vControlTask(void *pvParameters);
void MX_Motor_Init(void);
/* USER CODE END FunctionPrototypes */

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
    
  /* Create Mutexes */
  xI2C1Mutex = xSemaphoreCreateMutex();
  xUARTMutex = xSemaphoreCreateMutex();

  MX_Motor_Init();

  /* Create Tasks */
  if (xTaskCreate(vImuTask, "ImuTask", 256, NULL, 1, &xImuTaskHandle) != pdPASS)
  {
	  printf("IMU Task Creation Failed\r\n");
	  Error_Handler();
  }

  if (xTaskCreate(vLidarTask, "LidarTask", 512, NULL, 2, &xLidarTaskHandle) != pdPASS)
  {
	  printf("Lidar Task Creation Failed\r\n");
	  Error_Handler();
  }

  if (xTaskCreate(vSafetyTask, "SafetyTask", 256, NULL, 3, &xSafetyTaskHandle) != pdPASS)
  {
	  printf("Safety Task Creation Failed\r\n");
	  Error_Handler();
  }

  // Create Control Task (High Priority for Motor Control)
  if (xTaskCreate(vControlTask, "ControlTask", 256, NULL, 4, &xControlTaskHandle) != pdPASS)
  {
      printf("Control Task Creation Failed\r\n");
      Error_Handler();
  }

  /* USER CODE END Init */
}

/* Private application code -------------------------------------------------- */
/* USER CODE BEGIN Application */

void MX_Motor_Init(void)
{
  /* Initialize Motors */
  // Motor 1 (Right): TIM3 CH1/CH2, Encoder TIM2
  hMotor1.pwm_timer = &htim3;
  hMotor1.channel_fwd = TIM_CHANNEL_1;
  hMotor1.channel_rev = TIM_CHANNEL_2;
  hMotor1.enc_timer = &htim2;
  hMotor1.enc_resolution = 2000;
  Motor_Init(&hMotor1);

  // Motor 2 (Left): TIM3 CH3/CH4, Encoder TIM4
  hMotor2.pwm_timer = &htim3;
  hMotor2.channel_fwd = TIM_CHANNEL_3;
  hMotor2.channel_rev = TIM_CHANNEL_4;
  hMotor2.enc_timer = &htim4;
  hMotor2.enc_resolution = 2000;
  Motor_Init(&hMotor2);
}

void vControlTask(void *pvParameters)
{
    const float dt = 0.01f; // 10ms loop time

    for(;;)
    {
        // 1. Update Encoders (Speed Calculation)
        Motor_UpdateSpeed(&hMotor1, dt);
        Motor_UpdateSpeed(&hMotor2, dt);

        // 2. Update PWM (Ramping Logic)
        Motor_UpdatePWM(&hMotor1);
        Motor_UpdatePWM(&hMotor2);

        // 3. Loop Delay (100Hz)
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void vLidarTask(void *pvParameters)
{
  /* USER CODE BEGIN vLidarTask */
    ydlidar_init(lidar_dma_buffer, LIDAR_DMA_BUFFER_SIZE);

  uint16_t old_pos = 0;
  static uint32_t last_check = 0;

  for(;;)
  {
    // 1. Process Incoming Data from DMA
    uint16_t pos = LIDAR_DMA_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(huart2.hdmarx);

    if (pos != old_pos) {
      if (pos > old_pos) {
        ydlidar_process_data(&lidar_dma_buffer[old_pos], pos - old_pos);
      } else {
        ydlidar_process_data(&lidar_dma_buffer[old_pos], LIDAR_DMA_BUFFER_SIZE - old_pos);
        if (pos > 0) {
            ydlidar_process_data(&lidar_dma_buffer[0], pos);
        }
      }
      old_pos = pos;
    }

    // 2. Object Detection (every 0.5s)
    if ((HAL_GetTick() - last_check) > 500) {
        LidarObject_t objects[MAX_LIDAR_OBJECTS];
        uint8_t count = 0;

        // Wait for the UART to be free (Priority to Lidar)
        if (xSemaphoreTake(xUARTMutex, portMAX_DELAY) == pdTRUE) {
            ydlidar_detect_objects(objects, &count);
            xSemaphoreGive(xUARTMutex);
        }

        last_check = HAL_GetTick();
    }

    // Yield
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

void vImuTask(void *pvParameters)
{
  if (xSemaphoreTake(xI2C1Mutex, portMAX_DELAY) == pdTRUE) {
      if (ADXL343_Init(&hi2c1) != HAL_OK) {
          printf("IMU Init Failed\r\n");
      } else {
          ADXL343_ConfigShock(&hi2c1, 3.5f, 10.0f);
          printf("IMU Init OK\r\n");
      }
      xSemaphoreGive(xI2C1Mutex);
  }

  adxl343_axes_t accel_data;
  uint8_t error_count = 0;

  for(;;)
  {
    if (xSemaphoreTake(xI2C1Mutex, portMAX_DELAY) == pdTRUE) {

        uint8_t shock_detected = 0;
        HAL_StatusTypeDef status_read;

        // Read Axes
        status_read = ADXL343_ReadAxes(&hi2c1, &accel_data);

        if (status_read != HAL_OK) {
            error_count++;
        } else {
            error_count = 0;
            // Only check shock if bus is healthy
            if (ADXL343_CheckShock(&hi2c1)) {
                shock_detected = 1;
            }
        }

        if (error_count > 5) {
            printf("IMU I2C Error. Resetting...\r\n");
            HAL_I2C_DeInit(&hi2c1);
            MX_I2C1_Init();
            HAL_Delay(10); // Short hardware delay
            ADXL343_Init(&hi2c1);
            ADXL343_ConfigShock(&hi2c1, 3.5f, 10.0f);
            error_count = 0;
        }

        xSemaphoreGive(xI2C1Mutex);

        if (shock_detected) {
            for(int i=0; i<4; i++) {
                HAL_GPIO_TogglePin(GPIOC, STATUS_SOURIS_LED_Pin | STATUS_CHAT_LED_Pin);
                vTaskDelay(pdMS_TO_TICKS(100));
            }
            HAL_GPIO_WritePin(GPIOC, STATUS_SOURIS_LED_Pin | STATUS_CHAT_LED_Pin, GPIO_PIN_RESET);
        }
    }
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

void vSafetyTask(void *pvParameters)
{
  /* USER CODE BEGIN vSafetyTask */
  uint16_t distances[4];
  const uint16_t VOID_THRESHOLD = 200; // mm

  for(;;)
  {
      if (xSemaphoreTake(xI2C1Mutex, portMAX_DELAY) == pdTRUE) {
          TOF_Read_All(distances);
          xSemaphoreGive(xI2C1Mutex);

          // TOF Placement (Motor Axis = 0°):
          // 0: TOF1 (Front-Right, 45°)
          // 1: TOF2 (Rear-Right, 135°)
          // 2: TOF3 (Rear-Left, 225°)
          // 3: TOF4 (Front-Left, 315°)

          int void_fwd_right = (distances[0] > VOID_THRESHOLD);
          int void_rear_right = (distances[1] > VOID_THRESHOLD);
          int void_rear_left = (distances[2] > VOID_THRESHOLD);
          int void_fwd_left = (distances[3] > VOID_THRESHOLD);

          if (void_fwd_right || void_fwd_left || void_rear_right || void_rear_left) {
              printf("VOID DETECTED! AVOIDING...\r\n");
              HAL_GPIO_WritePin(GPIOC, STATUS_SOURIS_LED_Pin, GPIO_PIN_SET); // LED ON indicating reflex

              // Reflex Logic
              if (void_fwd_right) {
                  // Danger Front-Right -> Reverse + Turn Left
                  Motor_SetSpeed(&hMotor1, -30.0f); // Right Motor Back
                  Motor_SetSpeed(&hMotor2, -10.0f); // Left Motor Back (slower) -> Turn Left (or pivot)
              }
              else if (void_fwd_left) {
                  // Danger Front-Left -> Reverse + Turn Right
                  Motor_SetSpeed(&hMotor1, -10.0f);
                  Motor_SetSpeed(&hMotor2, -30.0f);
              }
              else if (void_rear_right || void_rear_left) {
                  // Danger Rear -> Forward
                   Motor_SetSpeed(&hMotor1, 30.0f);
                   Motor_SetSpeed(&hMotor2, 30.0f);
              }

              vTaskDelay(pdMS_TO_TICKS(500)); // Execute maneuver for 500ms
              
              // Stop after maneuver (or return to control loop control)
              Motor_SetSpeed(&hMotor1, 0.0f);
              Motor_SetSpeed(&hMotor2, 0.0f);
              HAL_GPIO_WritePin(GPIOC, STATUS_SOURIS_LED_Pin, GPIO_PIN_RESET);
          }
      }

      vTaskDelay(pdMS_TO_TICKS(50)); // Check every 50ms
  }
}
/* USER CODE END Application */
