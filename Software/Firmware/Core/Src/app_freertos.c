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
#include "../Drivers/lidar.h"
#include "../Drivers/imu.h"
#include "usart.h"
#include "i2c.h"
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef ----------------------------------------------------------- */
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------ */
/* USER CODE BEGIN PD */
#define LIDAR_DMA_BUFFER_SIZE 1024
/* USER CODE END PD */

/* Private macro ------------------------------------------------------------- */
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables --------------------------------------------------------- */
/* USER CODE BEGIN Variables */
// Task Handles
TaskHandle_t xLidarTaskHandle = NULL;
TaskHandle_t xImuTaskHandle = NULL;
TaskHandle_t xDefaultTaskHandle = NULL;

// Mutex Handles
SemaphoreHandle_t xI2C1Mutex = NULL;
SemaphoreHandle_t xUARTMutex = NULL;

// Lidar Buffer
uint8_t lidar_dma_buffer[LIDAR_DMA_BUFFER_SIZE];
/* USER CODE END Variables */

/* Private function prototypes ----------------------------------------------- */
/* USER CODE BEGIN FunctionPrototypes */
void vLidarTask(void *pvParameters);
void vImuTask(void *pvParameters);
void vDefaultTask(void *pvParameters);
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

  /* Create Tasks */
  // Priority 1 (Low) for Default Task
  xTaskCreate(vDefaultTask, "DefaultTask", 128, NULL, 1, &xDefaultTaskHandle);
  
  // Priority 2 (BelowNormal) for IMU Task
  xTaskCreate(vImuTask, "ImuTask", 256, NULL, 2, &xImuTaskHandle);

  // Priority 3 (Normal) for Lidar Task
  xTaskCreate(vLidarTask, "LidarTask", 512, NULL, 3, &xLidarTaskHandle);

  /* USER CODE END Init */
}

/* USER CODE BEGIN Header_vLidarTask */
/**
* @brief Function implementing the LidarTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_vLidarTask */
void vLidarTask(void *pvParameters)
{
  /* USER CODE BEGIN vLidarTask */
  ydlidar_init();
  
  // Start Circular DMA Reception
  if (HAL_UART_Receive_DMA(&huart2, lidar_dma_buffer, LIDAR_DMA_BUFFER_SIZE) != HAL_OK) {
      // Handle Error
      printf("LIDAR DMA Error\r\n");
  }

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

    // 2. Object Detection (every 50ms)
    if ((HAL_GetTick() - last_check) > 50) {
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
  /* USER CODE END vLidarTask */
}

/* USER CODE BEGIN Header_vImuTask */
/**
* @brief Function implementing the ImuTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_vImuTask */
void vImuTask(void *pvParameters)
{
  /* USER CODE BEGIN vImuTask */
  // Initialization with Mutex Protection
  if (xSemaphoreTake(xI2C1Mutex, portMAX_DELAY) == pdTRUE) {
      if (ADXL343_Init(&hi2c1) != HAL_OK) {
          printf("IMU Init Failed\r\n");
      } else {
          ADXL343_ConfigShock(&hi2c1, 2.5f, 10.0f);
          printf("IMU Init OK\r\n");
      }
      xSemaphoreGive(xI2C1Mutex);
  }

  adxl343_axes_t accel_data;

  for(;;)
  {
    if (xSemaphoreTake(xI2C1Mutex, portMAX_DELAY) == pdTRUE) {
        
        // Read Axes
        if (ADXL343_ReadAxes(&hi2c1, &accel_data) == HAL_OK) {
            // Data available in accel_data.x, .y, .z
        }

        // Check Shock
        if (ADXL343_CheckShock(&hi2c1)) {
            if (xSemaphoreTake(xUARTMutex, 10) == pdTRUE) {
                printf("!!! SHOCK DETECTED !!!\r\n");
                xSemaphoreGive(xUARTMutex);
            }
        }
        
        xSemaphoreGive(xI2C1Mutex);
    }

    vTaskDelay(pdMS_TO_TICKS(100)); // 10Hz
  }
  /* USER CODE END vImuTask */
}

/* USER CODE BEGIN Header_vDefaultTask */
/**
* @brief Function implementing the DefaultTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_vDefaultTask */
void vDefaultTask(void *pvParameters)
{
  /* USER CODE BEGIN vDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    HAL_GPIO_TogglePin(STATUS_SOURIS_LED_GPIO_Port, STATUS_SOURIS_LED_Pin);
    vTaskDelay(pdMS_TO_TICKS(500));
  }
  /* USER CODE END vDefaultTask */
}

/* Private application code -------------------------------------------------- */
/* USER CODE BEGIN Application */

/* USER CODE END Application */