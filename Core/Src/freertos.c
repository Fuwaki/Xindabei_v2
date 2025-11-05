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
#include "cmsis_os.h"
#include "ina226.h"
#include "main.h"
#include "myiic.h"
#include "task.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "common.h"
#include "motor.h"
#include "tim.h"
#include "usart.h"
#include "vl53l0x_api.h"
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

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
uint8_t TofResult = 0;
VL53L0X_Dev_t vl53l0x_dev;
VL53L0X_RangingMeasurementData_t ranging_data;
TaskHandle_t ToFMeasureTaskHandle;
/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
    .name = "defaultTask",
    .stack_size = 256 * 4,
    .priority = (osPriority_t)osPriorityNormal,
};
/* Definitions for ToFMeasure */
osThreadId_t ToFMeasureHandle;
const osThreadAttr_t ToFMeasure_attributes = {
    .name = "ToFMeasure",
    .stack_size = 512 * 4,
    .priority = (osPriority_t)osPriorityLow,
};
/* Definitions for UartTask */
osThreadId_t UartTaskHandle;
const osThreadAttr_t UartTask_attributes = {
    .name = "UartTask",
    .stack_size = 512 * 4,
    .priority = (osPriority_t)osPriorityLow,
};
/* Definitions for MotorSpeedTask */
osThreadId_t MotorSpeedTaskHandle;
const osThreadAttr_t MotorSpeedTask_attributes = {
    .name = "MotorSpeedTask",
    .stack_size = 256 * 4,
    .priority = (osPriority_t)osPriorityLow,
};
/* Definitions for uartQueue */
osMessageQueueId_t uartQueueHandle;
const osMessageQueueAttr_t uartQueue_attributes = {.name = "uartQueue"};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void StartTask02(void *argument);
void UartTaskFunc(void *argument);
void MotorSpeedTaskFunc(void *argument);

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

  /* Create the queue(s) */
  /* creation of uartQueue */
  uartQueueHandle =
      osMessageQueueNew(128, sizeof(uint16_t), &uartQueue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle =
      osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of ToFMeasure */
  ToFMeasureHandle = osThreadNew(StartTask02, NULL, &ToFMeasure_attributes);

  /* creation of UartTask */
  UartTaskHandle = osThreadNew(UartTaskFunc, NULL, &UartTask_attributes);

  /* creation of MotorSpeedTask */
  MotorSpeedTaskHandle =
      osThreadNew(MotorSpeedTaskFunc, NULL, &MotorSpeedTask_attributes);

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
void StartDefaultTask(void *argument) {
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  i2c_gpio_init();


  bool ina_ready = ina226_init();

  for (;;) {
    if (!ina_ready)
    {
      printf("INA226 init failed, retrying...\r\n");
      ina_ready = ina226_init();
      vTaskDelay(pdMS_TO_TICKS(500));
      continue;
    }

    int32_t current_ua = 0;

    bool ok_current = ina226_read_current_ua(&current_ua);

    if (!ok_current)
    {
      printf("INA226 read error\r\n");
      ina_ready = false;
      vTaskDelay(pdMS_TO_TICKS(200));
      continue;
    }

    // printf("Current: %ld\r\n", current_ua);
    motor_a_current = current_ua;

    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
    vTaskDelay(pdMS_TO_TICKS(5));
  }

  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartTask02 */
/**
 * @brief Function implementing the ToFMeasure thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartTask02 */
void StartTask02(void *argument) {
  /* USER CODE BEGIN StartTask02 */
  uint8_t status;
  uint32_t refSpadCount;
  uint8_t isApertureSpads;
  uint8_t is_init_ok = 0;

  ToFMeasureTaskHandle = xTaskGetCurrentTaskHandle();

  // 等待I2C和其他外设就绪
  osDelay(500);

  // 初始化VL53L0X
  vl53l0x_dev.I2cDevAddr =
      0x52; // VL53L0X默认I2C地址（实际上是0x29，但需要左移1位）

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
  status = VL53L0X_PerformRefSpadManagement(&vl53l0x_dev, &refSpadCount,
                                            &isApertureSpads);
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

  // 设置较高的信号质量要求
  status = VL53L0X_SetLimitCheckValue(
      &vl53l0x_dev, VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE,
      (FixPoint1616_t)(0.3 * 65536)); // 提高信号率限制到 0.3 Mcps
  if (status != VL53L0X_ERROR_NONE) {
    printf("Set signal rate limit failed with status: %d\r\n", status);
    goto init_failed;
  }

  status = VL53L0X_SetLimitCheckValue(
      &vl53l0x_dev, VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE,
      (FixPoint1616_t)(35 * 65536)); // 增加到35mm，允许更多不确定性
  if (status != VL53L0X_ERROR_NONE) {
    printf("Set sigma limit failed with status: %d\r\n", status);
    goto init_failed;
  }

  // 设置测量时间预算为70ms
  status = VL53L0X_SetMeasurementTimingBudgetMicroSeconds(&vl53l0x_dev, 70000);
  if (status != VL53L0X_ERROR_NONE) {
    printf("Set timing budget failed with status: %d\r\n", status);
    goto init_failed;
  }

  // 设置为连续测量模式
  status = VL53L0X_SetDeviceMode(&vl53l0x_dev,
                                 VL53L0X_DEVICEMODE_CONTINUOUS_RANGING);
  if (status != VL53L0X_ERROR_NONE) {
    printf("Set device mode failed with status: %d\r\n", status);
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
  for (;;) {
    if (!is_init_ok) {
      printf("Waiting for sensor to be ready...\r\n");
      osDelay(1000);
      continue;
    }

    // 等待中断触发
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    // 获取测量数据
    status = VL53L0X_GetRangingMeasurementData(&vl53l0x_dev, &ranging_data);

    if (status == VL53L0X_ERROR_NONE) {
      // 计算信噪比 (SNR)
      float signal = (float)ranging_data.SignalRateRtnMegaCps / 65536.0f;
      float ambient = (float)ranging_data.AmbientRateRtnMegaCps / 65536.0f;
      float snr = (ambient > 0.0001f) ? (signal / ambient) : signal;

      // 检查测量的可靠性
      if (ranging_data.RangeStatus == 0 &&        // 基本状态正常
          snr >= 3.0f &&                          // 信噪比至少3:1
          signal >= 0.3f &&                       // 信号强度至少0.3 Mcps
          ranging_data.EffectiveSpadRtnCount > 0) // 确保有效SPAD数量
      {
        TofResult = ranging_data.RangeMilliMeter;
        // // 打印详细的测量结果（包括信噪比）
        // sprintf(msg, "Distance: %d mm, SNR: %.1f, Signal: %.2f Mcps\r\n",
        //         ranging_data.RangeMilliMeter,
        //         snr,
        //         signal);
        // HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 100);
      } else if (ranging_data.RangeStatus != 0) {
        // printf("Invalid measurement (Status: %d)\r\n",
        // ranging_data.RangeStatus);
      } else {
        // printf("Low confidence measurement (SNR: %.1f, Signal: %.2f
        // Mcps)\r\n",
        //        snr, signal);
      }
    } else {
      printf("Failed to get ranging data with status: %d\r\n", status);
    }

    // 无论测量是否有效，都要清除中断标志以继续测量
    status = VL53L0X_ClearInterruptMask(&vl53l0x_dev, 0);
    if (status != VL53L0X_ERROR_NONE) {
      printf("Failed to clear interrupt with status: %d\r\n", status);
      // 如果清除中断失败，尝试重新启动测量
      status = VL53L0X_StartMeasurement(&vl53l0x_dev);
      if (status != VL53L0X_ERROR_NONE) {
        printf("Failed to restart measurement with status: %d\r\n", status);
      }
    }
  }
  /* USER CODE END StartTask02 */
}

/* USER CODE BEGIN Header_UartTaskFunc */
/**
 * @brief Function implementing the UartTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_UartTaskFunc */
void UartTaskFunc(void *argument) {
  /* USER CODE BEGIN UartTaskFunc */
  HAL_UARTEx_ReceiveToIdle_DMA(&huart2, uart_rx_buf, UART_BUF_LEN);
  uint16_t len;
  while (1) {
    if (osMessageQueueGet(uartQueueHandle, &len, NULL, osWaitForever) == osOK) {
      HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
    }
  }
  /* USER CODE END UartTaskFunc */
}

/* USER CODE BEGIN Header_MotorSpeedTaskFunc */
/**
 * @brief Function implementing the MotorSpeedTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_MotorSpeedTaskFunc */
void MotorSpeedTaskFunc(void *argument) {
  /* USER CODE BEGIN MotorSpeedTaskFunc */
  /* Infinite loop */

  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
  HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);
  float omega = 0.0;

  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 60000);
  EncPLL motor_pll;
  EncPLL_Init(&motor_pll, 10.0f, 2.0f, 0.01f);
  PIDController motor_pid;
  PID_Init(&motor_pid, PID_MODE_POSITIONAL, 30.0f, 10.0f, 0.5f, 0.5f,
           0.01f); // 减小 Kf
  PID_SetOutputLimit(&motor_pid, 0, 10000.0f);

  TickType_t xLast = xTaskGetTickCount();
  const TickType_t freq = pdMS_TO_TICKS(10); // 100 Hz

  for (;;) {
    omega += 0.05;
    float speed = EncPLL_Update(&motor_pll, (uint32_t)Get_Encoder1_Count());
    float output = PID_Update_Positional(&motor_pid, 3000 + 1500 * sinf(omega),
                                         speed); // 减小 setpoint
    motor_a_current_setpoint = (int32_t)(output * 6.0f);

    printf("%ld,%d,%d,%d\n", Get_Encoder1_Count(), (int)speed, (int)output,
           (int)(3000 + 1500 * sinf(omega)));
    vTaskDelayUntil(&xLast, freq);
  }
  /* USER CODE END MotorSpeedTaskFunc */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
