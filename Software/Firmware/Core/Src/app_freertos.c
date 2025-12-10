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
/* USER CODE END PD */

/* Private macro ------------------------------------------------------------- */
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables --------------------------------------------------------- */
/* USER CODE BEGIN Variables */
// Task Handles
TaskHandle_t xLidarTaskHandle = NULL;
TaskHandle_t xImuTaskHandle = NULL;


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
    vTaskDelay(pdMS_TO_TICKS(100)); // 10Hz
  }
  /* USER CODE END vImuTask */
}



/* Private application code -------------------------------------------------- */
/* USER CODE BEGIN Application */

/* USER CODE END Application */
