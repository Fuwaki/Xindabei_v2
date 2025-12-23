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
#include "car_control.h"
#include "common.h"
#include "imu.h"
#include "meg_adc.h"
#include "motor.h"
#include "oled_service.h"
#include "param_server.h"
#include "tim.h"
#include "tof.h"
#include "track.h"
#include "uart_command.h"
#include "usart.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>

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
TaskHandle_t AdcCaptureTaskHandle;
TaskHandle_t ImuCaptureTaskHandle;

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 386 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal5,
};
/* Definitions for ToFCapture */
osThreadId_t ToFCaptureHandle;
const osThreadAttr_t ToFCapture_attributes = {
  .name = "ToFCapture",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityLow7,
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
  .priority = (osPriority_t) osPriorityRealtime,
};
/* Definitions for AdcCapture */
osThreadId_t AdcCaptureHandle;
const osThreadAttr_t AdcCapture_attributes = {
  .name = "AdcCapture",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for CarControlTask */
osThreadId_t CarControlTaskHandle;
const osThreadAttr_t CarControlTask_attributes = {
  .name = "CarControlTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for GyroTask */
osThreadId_t GyroTaskHandle;
const osThreadAttr_t GyroTask_attributes = {
  .name = "GyroTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal1,
};
/* Definitions for TrackTask */
osThreadId_t TrackTaskHandle;
const osThreadAttr_t TrackTask_attributes = {
  .name = "TrackTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal4,
};
/* Definitions for uartQueue */
osMessageQueueId_t uartQueueHandle;
const osMessageQueueAttr_t uartQueue_attributes = {
  .name = "uartQueue"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void ConfigureTimerForRunTimeStats(void);
unsigned long GetRunTimeCounterValue(void);
float CalculateCpuUsage(void);

// Data Source Callbacks
int32_t GetCpuUsageInt(void)
{
    return (int32_t)CalculateCpuUsage();
}

float GetCpuUsageFloat(void)
{
    return CalculateCpuUsage();
}

int32_t GetTofDistance(void)
{
    return TofGetDistance();
}

/* 注册系统监控参数 */
void RegisterSystemParams(void)
{
    static ParamDesc params[] = {
        /* 默认：串口 + OLED 都显示 */
        {.name = "CPU",
         .type = PARAM_TYPE_FLOAT,
         .ops.f.get = GetCpuUsageFloat,
         .read_only = 1,
         .mask = PARAM_MASK_OLED},

        {.name = "TOF",
         .type = PARAM_TYPE_INT,
         .ops.i.get = (ParamGetIntCb)GetTofDistance,
         .read_only = 1,
         .mask = PARAM_MASK_OLED},
    };

    for (int i = 0; i < sizeof(params) / sizeof(params[0]); i++)
    {
        ParamServer_Register(&params[i]);
    }
}
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void TofTask(void *argument);
void UartTaskFunc(void *argument);
void MotorSpeedTaskFunc(void *argument);
void AdcCaptureFunc(void *argument);
void CarControlTaskFunc(void *argument);
void GyroTaskFunc(void *argument);
void TrackTaskFunc(void *argument);

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

  /* creation of CarControlTask */
  CarControlTaskHandle = osThreadNew(CarControlTaskFunc, NULL, &CarControlTask_attributes);

  /* creation of GyroTask */
  GyroTaskHandle = osThreadNew(GyroTaskFunc, NULL, &GyroTask_attributes);

  /* creation of TrackTask */
  TrackTaskHandle = osThreadNew(TrackTaskFunc, NULL, &TrackTask_attributes);

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
 * @retval NoneB
 */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
    /* Infinite loop */
    OledServiceInit();
    RegisterSystemParams();
    // int i = 1;
    // int j=0;
    for (;;)
    {
        // j++;
        // if (j >= 10)
        // {
        //     i=i>=5?1:i+1;
        //     j=0;
        // }
        // LED_Command(i, true);

        
        HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);

        /* KEY1: Next / Page Down */
        if (HAL_GPIO_ReadPin(KEY1_GPIO_Port, KEY1_Pin) == GPIO_PIN_RESET)
        {
            osDelay(20); // Debounce
            if (HAL_GPIO_ReadPin(KEY1_GPIO_Port, KEY1_Pin) == GPIO_PIN_RESET)
            {
                uint32_t pressTime = 0;
                bool longPressHandled = false;

                while (HAL_GPIO_ReadPin(KEY1_GPIO_Port, KEY1_Pin) == GPIO_PIN_RESET)
                {
                    osDelay(10);
                    pressTime += 10;
                    if (pressTime > 1000 && !longPressHandled)
                    { // 1s long press
                        OledServiceToggleDisplay();
                        longPressHandled = true;
                    }
                }

                if (!longPressHandled)
                {
                    OledHandleKey(OLED_KEY_NEXT);
                }
            }
        }

        /* KEY2: Start Car */
        if (HAL_GPIO_ReadPin(KEY2_GPIO_Port, KEY2_Pin) == GPIO_PIN_RESET)
        {
            osDelay(20); // Debounce
            if (HAL_GPIO_ReadPin(KEY2_GPIO_Port, KEY2_Pin) == GPIO_PIN_RESET)
            {
                printf("KEY2 Pressed: Start\n");
                TrackSetCommand(TRACK_CMD_START);
                while (HAL_GPIO_ReadPin(KEY2_GPIO_Port, KEY2_Pin) == GPIO_PIN_RESET)
                    osDelay(10);
            }
        }

        /* KEY3: Reset State Machine / Toggle Safety Check */
        if (HAL_GPIO_ReadPin(KEY3_GPIO_Port, KEY3_Pin) == GPIO_PIN_RESET)
        {
            osDelay(20); // Debounce
            if (HAL_GPIO_ReadPin(KEY3_GPIO_Port, KEY3_Pin) == GPIO_PIN_RESET)
            {
                uint32_t pressTime = 0;
                bool longPressHandled = false;

                while (HAL_GPIO_ReadPin(KEY3_GPIO_Port, KEY3_Pin) == GPIO_PIN_RESET)
                {
                    osDelay(10);
                    pressTime += 10;
                    if (pressTime > 1000 && !longPressHandled)
                    { // 1s long press - toggle safety check
                        bool currentSafetyState = TrackIsSafetyCheckEnabled();
                        TrackSetSafetyCheckEnabled(!currentSafetyState);
                        printf("Safety Check %s\n", !currentSafetyState ? "ENABLED" : "DISABLED");
                        longPressHandled = true;
                    }
                }

                if (!longPressHandled)
                {
                    // Short press - reset state machine
                    TrackSetCommand(TRACK_CMD_RESET);
                }
            }
        }

        OledServiceUpdate();
        print_handler();
        osDelay(50); // 提高刷新率以响应按键
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

    for (;;)
    {
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
    HAL_UARTEx_ReceiveToIdle_DMA(&huart6, uart_rx_buf, UART_BUF_LEN);
    uint16_t len;
    while (1)
    {
        if (osMessageQueueGet(uartQueueHandle, &len, NULL, osWaitForever) == osOK)
        {
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
    const TickType_t freq = pdMS_TO_TICKS(5); // 200 Hz

    for (;;)
    {
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
    AdcCaptureTaskHandle = xTaskGetCurrentTaskHandle();
    MegAdcInit();
    /* Infinite loop */
    for (;;)
    {
        MegAdcHandler();
    }
  /* USER CODE END AdcCaptureFunc */
}

/* USER CODE BEGIN Header_CarControlTaskFunc */
/**
 * @brief Function implementing the CarControlTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_CarControlTaskFunc */
void CarControlTaskFunc(void *argument)
{
  /* USER CODE BEGIN CarControlTaskFunc */
    /* Infinite loop */
    CarControlInit();
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(10); // 10ms 周期
    for (;;)
    {
        CarControlHandler();
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
  /* USER CODE END CarControlTaskFunc */
}

/* USER CODE BEGIN Header_GyroTaskFunc */
/**
 * @brief Function implementing the GyroTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_GyroTaskFunc */
void GyroTaskFunc(void *argument)
{
  /* USER CODE BEGIN GyroTaskFunc */
    ImuCaptureTaskHandle = xTaskGetCurrentTaskHandle();
    /* Infinite loop */
    IMUInit();
    for (;;)
    {
        // 等待IMU中断通知
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        // 收到中断后处理IMU数据
        IMUHandler();
    }
  /* USER CODE END GyroTaskFunc */
}

/* USER CODE BEGIN Header_TrackTaskFunc */
/**
 * @brief Function implementing the TrackTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_TrackTaskFunc */
void TrackTaskFunc(void *argument)
{
  /* USER CODE BEGIN TrackTaskFunc */
    /* Infinite loop */
    TrackInit();
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(20); // 20ms 周期
    for (;;)
    {
        TrackHandler();
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
  /* USER CODE END TrackTaskFunc */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void ConfigureTimerForRunTimeStats(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

unsigned long GetRunTimeCounterValue(void)
{
    return DWT->CYCCNT;
}

float CalculateCpuUsage(void)
{
    static uint32_t ulLastIdleTime = 0;
    static uint32_t ulLastTotalTime = 0;
    static float cpuUsage = 0.0f;

    TaskStatus_t *pxTaskStatusArray;
    volatile UBaseType_t uxArraySize, x;
    uint32_t ulTotalRunTime;

    uxArraySize = uxTaskGetNumberOfTasks();
    pxTaskStatusArray = pvPortMalloc(uxArraySize * sizeof(TaskStatus_t));

    if (pxTaskStatusArray != NULL)
    {
        uxArraySize = uxTaskGetSystemState(pxTaskStatusArray, uxArraySize, &ulTotalRunTime);

        for (x = 0; x < uxArraySize; x++)
        {
            if (pxTaskStatusArray[x].xHandle == xTaskGetIdleTaskHandle())
            {
                uint32_t ulIdleTime = pxTaskStatusArray[x].ulRunTimeCounter;
                uint32_t ulTotalTime = ulTotalRunTime;

                uint32_t ulIdleDelta = ulIdleTime - ulLastIdleTime;
                uint32_t ulTotalDelta = ulTotalTime - ulLastTotalTime;

                if (ulTotalDelta > 0)
                {
                    cpuUsage = 100.0f * (1.0f - ((float)ulIdleDelta / (float)ulTotalDelta));
                }

                ulLastIdleTime = ulIdleTime;
                ulLastTotalTime = ulTotalTime;
                break;
            }
        }
        vPortFree(pxTaskStatusArray);
    }

    return cpuUsage;
}
/* USER CODE END Application */

