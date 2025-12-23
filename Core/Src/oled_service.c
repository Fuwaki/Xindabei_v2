#include "oled_service.h"
#include "ssd1306.h"
#include "ssd1306_fonts.h"
#include <stdio.h>
#include <string.h>
#include "common.h"
#include "param_server.h"

#define ITEMS_PER_PAGE 5

static int s_currentIdx = 0;
static bool s_displayOn = false;  // 默认关闭屏幕以减少CPU使用

// 紧急情况相关变量
static OledEmergencyType s_emergencyType = OLED_EMERGENCY_NONE;
static char s_emergencyMessage[32] = {0};

void OledServiceInit(void)
{
    ssd1306_Init();
    s_currentIdx = 0;
    s_displayOn = false;  // 默认关闭屏幕以减少CPU使用
    
    // 初始化时清空屏幕
    ssd1306_Fill(Black);
    ssd1306_UpdateScreen();
}

void OledServiceToggleDisplay(void)
{
    s_displayOn = !s_displayOn;
    if (!s_displayOn) {
        ssd1306_Fill(Black);
        ssd1306_UpdateScreen();
    }
}

static int GetVisibleParamCount(void)
{
    int count = ParamServer_GetCount();
    int visible = 0;
    for (int i = 0; i < count; i++) {
        const ParamDesc *item = ParamServer_GetByIndex(i);
        if (item && (item->mask & PARAM_MASK_OLED)) {
            visible++;
        }
    }
    return visible;
}

void OledHandleKey(OledKeyEvent event)
{
    int visibleCount = GetVisibleParamCount();
    if (visibleCount == 0) return;

    switch (event) {
        case OLED_KEY_NEXT:
            s_currentIdx += ITEMS_PER_PAGE;
            if (s_currentIdx >= visibleCount) {
                s_currentIdx = 0;
            }
            break;
    }
}

void OledServiceUpdate(void)
{
    static bool s_lastWasEmergency = false;

    // 检查是否有紧急情况需要显示
    if (s_emergencyType != OLED_EMERGENCY_NONE) {
        s_lastWasEmergency = true;
        // 黑色背景，仅文字区域反色
        ssd1306_Fill(Black);
        
        // 计算文本位置使其居中显示
        uint8_t textWidth = strlen(s_emergencyMessage) * 7;  // 7像素宽度每个字符
        uint8_t textHeight = 10;  // 字体高度
        
        // 限制宽度防止溢出
        if (textWidth > 120) textWidth = 120;
        
        uint8_t xPos = (128 - textWidth) / 2;
        uint8_t yPos = (64 - textHeight) / 2;
        
        // 确保位置在屏幕范围内
        if (xPos > 127) xPos = 0;
        if (yPos > 63) yPos = 0;
        
        // 绘制文字背景框 (白底) - 增加边界检查
        uint8_t rectX1 = (xPos >= 2) ? xPos - 2 : 0;
        uint8_t rectY1 = (yPos >= 2) ? yPos - 2 : 0;
        uint8_t rectX2 = xPos + textWidth + 1;
        uint8_t rectY2 = yPos + textHeight + 1;
        
        if (rectX2 > 127) rectX2 = 127;
        if (rectY2 > 63) rectY2 = 63;

        ssd1306_FillRectangle(rectX1, rectY1, rectX2, rectY2, White);
        
        ssd1306_SetCursor(xPos, yPos);
        ssd1306_WriteString(s_emergencyMessage, Font_7x10, Black);  // 黑色文字
        
        ssd1306_UpdateScreen();
        return;
    }
    
    // 如果之前是紧急情况，现在不是了，且屏幕是关闭的，需要清屏一次
    if (s_lastWasEmergency) {
        s_lastWasEmergency = false;
        if (!s_displayOn) {
            ssd1306_Fill(Black);
            ssd1306_UpdateScreen();
        }
    }
    
    // 原有的正常显示逻辑
    if (!s_displayOn) return;

    ssd1306_Fill(Black);

    int visibleCount = GetVisibleParamCount();
    if (visibleCount == 0) {
        ssd1306_SetCursor(0, 0);
        ssd1306_WriteString("No Params", Font_7x10, White);
        ssd1306_UpdateScreen();
        return;
    }

    if (s_currentIdx >= visibleCount) s_currentIdx = 0;

    /* 计算当前页 */
    int totalPages = (visibleCount + ITEMS_PER_PAGE - 1) / ITEMS_PER_PAGE;
    int currentPage = s_currentIdx / ITEMS_PER_PAGE;
    
    /* 寻找当前页第一个可见参数的原始索引 */
    int count = ParamServer_GetCount();
    int startRawIdx = 0;
    int skipped = 0;
    for (int i = 0; i < count; i++) {
        const ParamDesc *item = ParamServer_GetByIndex(i);
        if (item && (item->mask & PARAM_MASK_OLED)) {
            if (skipped == s_currentIdx) {
                startRawIdx = i;
                break;
            }
            skipped++;
        }
    }

    uint8_t yPos = 0;
    int itemsShown = 0;

    for (int i = startRawIdx; i < count && itemsShown < ITEMS_PER_PAGE; i++) {
        const ParamDesc *item = ParamServer_GetByIndex(i);
        if (!item) continue;
        if (!(item->mask & PARAM_MASK_OLED)) {
            continue;
        }

        char buffer[32];
        char valStr[20];

        if (item->type == PARAM_TYPE_FLOAT) {
            snprintf(valStr, sizeof(valStr), FLOAT_FMT, FLOAT_TO_INT(ParamServer_GetValueFloat(item)));
        } else if (item->type == PARAM_TYPE_ENUM) {
            snprintf(valStr, sizeof(valStr), "%s", ParamServer_GetString(item));
        } else {
            snprintf(valStr, sizeof(valStr), "%d", ParamServer_GetValueInt(item));
        }

        snprintf(buffer, sizeof(buffer), "%s:%s", item->name, valStr);

        ssd1306_SetCursor(0, yPos);
        ssd1306_WriteString(buffer, Font_7x10, White);

        yPos += 12;
        itemsShown++;
    }

    // Draw Page Number (Top Right) with inverted background
    char pageStr[16];  // 增加缓冲区大小以避免截断警告
    snprintf(pageStr, sizeof(pageStr), "%d/%d", currentPage + 1, totalPages);
    uint8_t pageStrLen = strlen(pageStr);
    uint8_t xPos = 128 - (pageStrLen * 7) - 2;
    uint8_t boxWidth = pageStrLen * 7 + 4;
    uint8_t boxHeight = 12;
    
    // Draw inverted rectangle background
    ssd1306_FillRectangle(xPos, 0, xPos + boxWidth - 1, boxHeight - 1, White);
    
    // Draw page number in black text on white background
    ssd1306_SetCursor(xPos + 2, 1);
    ssd1306_WriteString(pageStr, Font_7x10, Black);

    ssd1306_UpdateScreen();
}

// 设置紧急情况显示
void OledServiceSetEmergency(OledEmergencyType type, const char* message)
{
    s_emergencyType = type;
    if (message && strlen(message) < sizeof(s_emergencyMessage)) {
        strcpy(s_emergencyMessage, message);
    } else {
        // 根据类型设置默认消息
        switch (type) {
            case OLED_EMERGENCY_OUT_OF_LINE:
                strcpy(s_emergencyMessage, "OUT OF LINE!");
                break;
            case OLED_EMERGENCY_ROLL_OVER:
                strcpy(s_emergencyMessage, "ROLL OVER!");
                break;
            default:
                strcpy(s_emergencyMessage, "EMERGENCY!");
                break;
        }
    }
}

// 清除紧急情况显示
void OledServiceClearEmergency(void)
{
    s_emergencyType = OLED_EMERGENCY_NONE;
    memset(s_emergencyMessage, 0, sizeof(s_emergencyMessage));
}
