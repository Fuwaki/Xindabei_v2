#ifndef OLED_SERVICE_H
#define OLED_SERVICE_H

#include <stdint.h>
#include <stdbool.h>

// 定义数据类型枚举
typedef enum {
    DISPLAY_TYPE_INT,
    DISPLAY_TYPE_FLOAT,
    DISPLAY_TYPE_STRING
} DisplayType;

// 定义回调函数类型
typedef int32_t (*IntSourceCb)(void);
typedef float (*FloatSourceCb)(void);
typedef const char* (*StringSourceCb)(void);

// 初始化服务
void OledServiceInit(void);

// 注册数据源
// label: 显示的标签 (例如 "CPU Usage")
// callback: 获取数据的回调函数
// 返回值: true 成功, false 失败 (例如已满)
bool OledRegisterInt(const char* label, IntSourceCb callback);
bool OledRegisterFloat(const char* label, FloatSourceCb callback);
bool OledRegisterString(const char* label, StringSourceCb callback);

// 页面控制
void OledNextPage(void);
void OledPrevPage(void);

// 刷新显示 (在任务循环中调用)
void OledServiceUpdate(void);

#endif // OLED_SERVICE_H
