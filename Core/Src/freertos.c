/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * File Name          : freertos.c
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

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usart.h"
#include <stdio.h>
#include <math.h>
#include <string.h>
#include "vl53l0x_api.h"
// #include "VL53L0X.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
VL53L0X_Dev_t vl53l0x_dev;
VL53L0X_RangingMeasurementData_t ranging_data;
/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 256 * 4,  // 增加堆栈大小
  .priority = (osPriority_t) osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
 * @brief  Function implementing the defaultTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  char msg[80];
  uint8_t status;
  uint32_t refSpadCount;
  uint8_t isApertureSpads;
  uint8_t is_init_ok = 0;

  // 等待I2C和其他外设就绪
  osDelay(500);

  // 初始化VL53L0X
  vl53l0x_dev.I2cDevAddr = 0x52;  // VL53L0X默认I2C地址（实际上是0x29，但需要左移1位）

  printf("Initializing VL53L0X...\r\n");
  
  status = VL53L0X_DataInit(&vl53l0x_dev);
  if (status != VL53L0X_ERROR_NONE) {
    printf("VL53L0X_DataInit failed with status: %d\r\n", status);
    goto init_failed;
  }

  status = VL53L0X_StaticInit(&vl53l0x_dev);
  if (status != VL53L0X_ERROR_NONE) {
    printf("VL53L0X_StaticInit failed with status: %d\r\n", status);
    goto init_failed;
  }

  // 执行参考校准
  uint8_t VhvSettings, PhaseCal;
  status = VL53L0X_PerformRefCalibration(&vl53l0x_dev, &VhvSettings, &PhaseCal);
  if (status != VL53L0X_ERROR_NONE) {
    printf("VL53L0X_PerformRefCalibration failed with status: %d\r\n", status);
    goto init_failed;
  }

  // 执行温度校准
  status = VL53L0X_PerformRefSpadManagement(&vl53l0x_dev, &refSpadCount, &isApertureSpads);
  if (status != VL53L0X_ERROR_NONE) {
    printf("SPAD management failed with status: %d\r\n", status);
    goto init_failed;
  }

  // 设置参考校准数据
  status = VL53L0X_SetRefCalibration(&vl53l0x_dev, VhvSettings, PhaseCal);
  if (status != VL53L0X_ERROR_NONE) {
    printf("Set reference calibration failed with status: %d\r\n", status);
    goto init_failed;
  }

  // 设置长距离高精度模式参数
  
  // 设置较低的信号率限制，以接受更弱的反射信号
  status = VL53L0X_SetLimitCheckValue(&vl53l0x_dev,
      VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE,
      (FixPoint1616_t)(0.086 * 65536)); // 降低信号率限制到0.086 Mcps
  if (status != VL53L0X_ERROR_NONE) {
    printf("Set signal rate limit failed with status: %d\r\n", status);
    goto init_failed;
  }

  // 设置较高的Sigma限制，允许接受更不确定的测量值
  status = VL53L0X_SetLimitCheckValue(&vl53l0x_dev,
      VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE,
      (FixPoint1616_t)(90 * 65536)); // 增加sigma限制到90mm
  if (status != VL53L0X_ERROR_NONE) {
    printf("Set sigma limit failed with status: %d\r\n", status);
    goto init_failed;
  }

  // 启用长距离模式
  status = VL53L0X_SetVcselPulsePeriod(&vl53l0x_dev, VL53L0X_VCSEL_PERIOD_PRE_RANGE, 18);
  if (status != VL53L0X_ERROR_NONE) {
    printf("Set pre-range VCSEL period failed with status: %d\r\n", status);
    goto init_failed;
  }

  status = VL53L0X_SetVcselPulsePeriod(&vl53l0x_dev, VL53L0X_VCSEL_PERIOD_FINAL_RANGE, 14);
  if (status != VL53L0X_ERROR_NONE) {
    printf("Set final-range VCSEL period failed with status: %d\r\n", status);
    goto init_failed;
  }

  // 设置测量时间预算为70ms
  status = VL53L0X_SetMeasurementTimingBudgetMicroSeconds(&vl53l0x_dev, 70000);
  if (status != VL53L0X_ERROR_NONE) {
    printf("Set timing budget failed with status: %d\r\n", status);
    goto init_failed;
  }

  // 设置为连续测量模式
  status = VL53L0X_SetDeviceMode(&vl53l0x_dev, VL53L0X_DEVICEMODE_CONTINUOUS_RANGING);
  if (status != VL53L0X_ERROR_NONE) {
    printf("Set continuous mode failed with status: %d\r\n", status);
    goto init_failed;
  }

  // 启动连续测量
  status = VL53L0X_StartMeasurement(&vl53l0x_dev);
  if (status != VL53L0X_ERROR_NONE) {
    printf("Start measurement failed with status: %d\r\n", status);
    goto init_failed;
  }

  printf("VL53L0X initialized successfully\r\n");
  is_init_ok = 1;
  goto init_done;

init_failed:
  printf("VL53L0X initialization failed. Check connections and power.\r\n");

init_done:

  /* Infinite loop */
  for(;;)
  {
    if (!is_init_ok) {
      printf("Waiting for sensor to be ready...\r\n");
      osDelay(1000);
      continue;
    }

    // 检查数据就绪
    uint8_t dataReady = 0;
    status = VL53L0X_GetMeasurementDataReady(&vl53l0x_dev, &dataReady);
    
    if (status == VL53L0X_ERROR_NONE && dataReady) {
      // 获取测量数据
      status = VL53L0X_GetRangingMeasurementData(&vl53l0x_dev, &ranging_data);
      
      if (status == VL53L0X_ERROR_NONE) {
        if (ranging_data.RangeStatus == 0) {
          // 打印详细的测量结果
          sprintf(msg, "Distance: %d mm, Signal: %u.%02u Mcps, Ambient: %u.%02u Mcps\r\n", 
                  ranging_data.RangeMilliMeter,
                  ranging_data.SignalRateRtnMegaCps/65536,
                  ((ranging_data.SignalRateRtnMegaCps % 65536) * 100) / 65536,
                  ranging_data.AmbientRateRtnMegaCps/65536,
                  ((ranging_data.AmbientRateRtnMegaCps % 65536) * 100) / 65536);
          HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 100);

          // 清除中断标志
          status = VL53L0X_ClearInterruptMask(&vl53l0x_dev, 0);
          if (status != VL53L0X_ERROR_NONE) {
            printf("Failed to clear interrupt with status: %d\r\n", status);
          }
        } else {
          printf("Invalid measurement (Status: %d)\r\n", ranging_data.RangeStatus);
        }
      } else {
        printf("Failed to get ranging data with status: %d\r\n", status);
      }
    } else if (status != VL53L0X_ERROR_NONE) {
      printf("Failed to check data ready with status: %d\r\n", status);
    }

    // 延时70ms，配合测量时间预算
    osDelay(70);
  }
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

