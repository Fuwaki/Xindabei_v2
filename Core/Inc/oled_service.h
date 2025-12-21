#ifndef OLED_SERVICE_H
#define OLED_SERVICE_H

#include <stdint.h>
#include <stdbool.h>

// 初始化服务
void OledServiceInit(void);

// 按键事件定义
typedef enum {
    OLED_KEY_NEXT        // 翻页
} OledKeyEvent;

// 紧急情况类型定义
typedef enum {
    OLED_EMERGENCY_NONE = 0,
    OLED_EMERGENCY_OUT_OF_LINE,  // 丢线
    OLED_EMERGENCY_ROLL_OVER     // 翻车
} OledEmergencyType;

// 处理按键事件
void OledHandleKey(OledKeyEvent event);

// 切换显示开关
void OledServiceToggleDisplay(void);

// 刷新显示 (在任务循环中调用)
void OledServiceUpdate(void);

// 设置紧急情况显示
void OledServiceSetEmergency(OledEmergencyType type, const char* message);

// 清除紧急情况显示
void OledServiceClearEmergency(void);

#endif // OLED_SERVICE_H
