/* app_common.h */
#pragma once
#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "task.h"
extern TaskHandle_t ToFMeasureTaskHandle;
extern osMessageQueueId_t uartQueueHandle;

#define UART_BUF_LEN  256 

extern uint8_t uart_rx_buf[UART_BUF_LEN];