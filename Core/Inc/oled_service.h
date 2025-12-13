#ifndef OLED_SERVICE_H
#define OLED_SERVICE_H

#include <stdint.h>
#include <stdbool.h>

// 初始化服务
void OledServiceInit(void);

// 按键事件定义
typedef enum {
    OLED_KEY_NEXT,       // 切换选中项/翻页
    OLED_KEY_ENTER,      // 进入/退出编辑模式
    OLED_KEY_CHANGE_INC, // 正向修改（短按）
    OLED_KEY_CHANGE_DEC  // 反向修改（长按）
} OledKeyEvent;

// 处理按键事件
void OledHandleKey(OledKeyEvent event);

// 刷新显示 (在任务循环中调用)
void OledServiceUpdate(void);

#endif // OLED_SERVICE_H
