/* app_common.h */
#pragma once
#include "cmsis_os2.h"   // 为了 osEventFlagsId_t
#include "FreeRTOS.h"    // 为了 TaskHandle_t
#include "task.h"
extern TaskHandle_t ToFMeasureTaskHandle;
extern osMessageQueueId_t uartQueueHandle;

#define UART_BUF_LEN  256 

extern uint8_t uart_rx_buf[UART_BUF_LEN];