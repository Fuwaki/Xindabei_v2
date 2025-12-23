#include "led.h"
#include "usart.h"
#include <stdint.h>
#include <stdbool.h>

void LED_Command(uint8_t mode,bool state){
    static WS2812_CtrlTypeDef ctrl; 
    
    // 检查串口是否忙碌，防止修改正在发送的数据
    if (HAL_UART_GetState(&huart2) != HAL_UART_STATE_READY) {
        return; 
    }

    ctrl.head=0xAA;
    ctrl.WS2812_Mode=mode;
    ctrl.Init_State=state;
    ctrl.checksum=ctrl.head + ctrl.WS2812_Mode + ctrl.Init_State;
    HAL_UART_Transmit_DMA(&huart2, (uint8_t *)&ctrl, sizeof(ctrl));
}
