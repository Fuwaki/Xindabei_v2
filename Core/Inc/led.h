#ifndef __LED_H__
#define __LED_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

/* WS2812 控制结构体 */
typedef struct{
    uint8_t head;       // 默认0xAA
    uint8_t WS2812_Mode;
    uint8_t Init_State;
    uint8_t checksum;   // head + mode + state
} __attribute__((packed)) WS2812_CtrlTypeDef;

void LED_Command(uint8_t mode,bool state);

#ifdef __cplusplus
}
#endif

#endif