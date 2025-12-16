/* app_common.h */
#pragma once
#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "task.h"
extern TaskHandle_t ToFMeasureTaskHandle;
extern osMessageQueueId_t uartQueueHandle;

#define UART_BUF_LEN  256 

extern uint8_t uart_rx_buf[UART_BUF_LEN];

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#define FLOAT_STR_BUF_SIZE 32
#define FLOAT_STR_BUF_COUNT 8

static inline char* float_to_str(float f) {
    static char bufs[FLOAT_STR_BUF_COUNT][FLOAT_STR_BUF_SIZE];
    static int buf_idx = 0;
    char* buf = bufs[buf_idx];
    buf_idx = (buf_idx + 1) % FLOAT_STR_BUF_COUNT;
    if (isnan(f)) { strcpy(buf, "NaN"); return buf; }
    if (isinf(f)) { strcpy(buf, "Inf"); return buf; }
    int is_neg = (f < 0);
    if (is_neg) f = -f;
    int int_part = (int)f;
    float frac_part = f - int_part;
    int frac_int = (int)((frac_part * 1000000.0f) + 0.5f);
    if (frac_int >= 1000000) { int_part++; frac_int -= 1000000; }
    sprintf(buf, "%s%d.%06d", is_neg ? "-" : "", int_part, frac_int);
    int len = strlen(buf);
    char* p = buf + len - 1;
    while (p > buf && *p == '0') { *p-- = '\0'; }
    if (*p == '.') { *p = '\0'; }
    return buf;
}

#define FLOAT_TO_INT(x) float_to_str(x)
#define FLOAT_FMT "%s"

// 浮点数符号函数
#define SIGN(x) ((x > 0) ? 1.0f : ((x < 0) ? -1.0f : 0.0f))
// 限幅函数
#define CLAMP(x, min, max) ((x) < (min) ? (min) : ((x) > (max) ? (max) : (x)))