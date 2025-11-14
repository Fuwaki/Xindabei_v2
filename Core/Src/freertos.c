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
#include "common.h"
#include "motor.h"
#include "tim.h"
#include "uart_command.h"
#include "usart.h"
#include "tof.h"
#include <stdint.h>
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
TaskHandle_t ToFMeasureTaskHandle;
/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for ToFCapture */
osThreadId_t ToFCaptureHandle;
const osThreadAttr_t ToFCapture_attributes = {
  .name = "ToFCapture",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for UartTask */
osThreadId_t UartTaskHandle;
const osThreadAttr_t UartTask_attributes = {
  .name = "UartTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for MotorSpeedTask */
osThreadId_t MotorSpeedTaskHandle;
const osThreadAttr_t MotorSpeedTask_attributes = {
  .name = "MotorSpeedTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for AdcCapture */
osThreadId_t AdcCaptureHandle;
const osThreadAttr_t AdcCapture_attributes = {
  .name = "AdcCapture",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for AngularSpeed */
osThreadId_t AngularSpeedHandle;
const osThreadAttr_t AngularSpeed_attributes = {
  .name = "AngularSpeed",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for uartQueue */
osMessageQueueId_t uartQueueHandle;
const osMessageQueueAttr_t uartQueue_attributes = {
  .name = "uartQueue"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void TofTask(void *argument);
void UartTaskFunc(void *argument);
void MotorSpeedTaskFunc(void *argument);
void AdcCaptureFunc(void *argument);
void AngularSpeedTaskFunc(void *argument);

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
  uartQueueHandle = osMessageQueueNew (128, sizeof(uint16_t), &uartQueue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of ToFCapture */
  ToFCaptureHandle = osThreadNew(TofTask, NULL, &ToFCapture_attributes);

  /* creation of UartTask */
  UartTaskHandle = osThreadNew(UartTaskFunc, NULL, &UartTask_attributes);

  /* creation of MotorSpeedTask */
  MotorSpeedTaskHandle = osThreadNew(MotorSpeedTaskFunc, NULL, &MotorSpeedTask_attributes);

  /* creation of AdcCapture */
  AdcCaptureHandle = osThreadNew(AdcCaptureFunc, NULL, &AdcCapture_attributes);

  /* creation of AngularSpeed */
  AngularSpeedHandle = osThreadNew(AngularSpeedTaskFunc, NULL, &AngularSpeed_attributes);

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
  /* Infinite loop */

  for (;;) {
    print_handler();
    // HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
    vTaskDelay(pdMS_TO_TICKS(50));
  }

  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_TofTask */
/**
* @brief Function implementing the ToFCapture thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_TofTask */
void TofTask(void *argument)
{
  /* USER CODE BEGIN TofTask */
  ToFMeasureTaskHandle = xTaskGetCurrentTaskHandle();
  TofInit();

  for (;;) {
    TofHandler();
  }
  /* USER CODE END TofTask */
}

/* USER CODE BEGIN Header_UartTaskFunc */
/**
 * @brief Function implementing the UartTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_UartTaskFunc */
void UartTaskFunc(void *argument)
{
  /* USER CODE BEGIN UartTaskFunc */
  HAL_UARTEx_ReceiveToIdle_DMA(&huart2, uart_rx_buf, UART_BUF_LEN);
  uint16_t len;
  while (1) {
    if (osMessageQueueGet(uartQueueHandle, &len, NULL, osWaitForever) == osOK) {
      handle_command(uart_rx_buf, len);
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
void MotorSpeedTaskFunc(void *argument)
{
  /* USER CODE BEGIN MotorSpeedTaskFunc */
  /* Infinite loop */
  MotorInit();
  TickType_t xLast = xTaskGetTickCount();
  const TickType_t freq = pdMS_TO_TICKS(10); // 100 Hz

  for (;;) {
    SpeedLoopHandler();
    vTaskDelayUntil(&xLast, freq);
  }
  /* USER CODE END MotorSpeedTaskFunc */
}

/* USER CODE BEGIN Header_AdcCaptureFunc */
/**
* @brief Function implementing the AdcCapture thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_AdcCaptureFunc */
void AdcCaptureFunc(void *argument)
{
  /* USER CODE BEGIN AdcCaptureFunc */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END AdcCaptureFunc */
}

/* USER CODE BEGIN Header_AngularSpeedTaskFunc */
/**
* @brief Function implementing the AngularSpeed thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_AngularSpeedTaskFunc */
void AngularSpeedTaskFunc(void *argument)
{
  /* USER CODE BEGIN AngularSpeedTaskFunc */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END AngularSpeedTaskFunc */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

