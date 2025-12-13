#include "oled_service.h"
#include "ssd1306.h"
#include "ssd1306_fonts.h"
#include <stdio.h>
#include <string.h>
#include "common.h"
#include "param_server.h"

#define ITEMS_PER_PAGE 5

static int s_currentIdx = 0;
static bool s_isEditing = false;

void OledServiceInit(void)
{
    ssd1306_Init();
    s_currentIdx = 0;
    s_isEditing = false;
}

void OledHandleKey(OledKeyEvent event)
{
    int count = ParamServer_GetCount();
    if (count == 0) return;

    const ParamDesc *p = ParamServer_GetByIndex(s_currentIdx);

    switch (event) {
        case OLED_KEY_NEXT:
            if (s_isEditing) {
                /* 编辑模式下按 NEXT，暂时定义为退出编辑并切到下一个？
                   或者定义为“确认并下一个”？
                   这里简单处理：如果正在编辑，NEXT 也可以切换焦点，自动退出编辑态 */
                s_isEditing = false;
            }
            s_currentIdx++;
            if (s_currentIdx >= count) {
                s_currentIdx = 0;
            }
            break;

        case OLED_KEY_ENTER:
            if (p && !p->read_only) {
                s_isEditing = !s_isEditing;
            } else {
                /* 只读参数不能进入编辑模式 */
                s_isEditing = false;
            }
            break;

        case OLED_KEY_CHANGE_INC:
        case OLED_KEY_CHANGE_DEC:
            if (s_isEditing && p && !p->read_only) {
                float dir = (event == OLED_KEY_CHANGE_INC) ? 1.0f : -1.0f;
                if (p->type == PARAM_TYPE_FLOAT) {
                    float v = ParamServer_GetValueFloat(p);
                    v += dir * p->step;
                    ParamServer_SetValueFloat(p, v);
                } else {
                    int v = ParamServer_GetValueInt(p);
                    v += (int)(dir * p->step);
                    ParamServer_SetValueInt(p, v);
                }
            }
            break;
    }
}

void OledServiceUpdate(void)
{
    ssd1306_Fill(Black);

    int count = ParamServer_GetCount();
    if (count == 0) {
        ssd1306_SetCursor(0, 0);
        ssd1306_WriteString("No Params", Font_7x10, White);
        ssd1306_UpdateScreen();
        return;
    }

    /* 计算当前页 */
    int totalPages = (count + ITEMS_PER_PAGE - 1) / ITEMS_PER_PAGE;
    int currentPage = s_currentIdx / ITEMS_PER_PAGE;
    
    int startIdx = currentPage * ITEMS_PER_PAGE;
    int endIdx = startIdx + ITEMS_PER_PAGE;
    if (endIdx > count) endIdx = count;

    uint8_t yPos = 0; 

    for (int i = startIdx; i < endIdx; i++) {
        const ParamDesc *item = ParamServer_GetByIndex(i);
        if (!item) continue;
        if (!(item->mask & PARAM_MASK_OLED)) {
            continue;
        }

        char buffer[32];
        char valStr[16];

        if (item->type == PARAM_TYPE_FLOAT) {
            snprintf(valStr, sizeof(valStr), FLOAT_FMT, FLOAT_TO_INT(ParamServer_GetValueFloat(item)));
        } else {
            snprintf(valStr, sizeof(valStr), "%d", ParamServer_GetValueInt(item));
        }

        /* 选中项前面加标记，编辑态加不同标记 */
        char prefix = ' ';
        if (i == s_currentIdx) {
            prefix = s_isEditing ? '*' : '>';
        }

        snprintf(buffer, sizeof(buffer), "%c%s:%s", prefix, item->name, valStr);

        ssd1306_SetCursor(0, yPos);
        ssd1306_WriteString(buffer, Font_7x10, White);

        /* 编辑模式下对当前行做轻量反白，利用现有矩形反转函数 */
        if (i == s_currentIdx && s_isEditing) {
            uint8_t y1 = yPos;
            uint8_t y2 = (uint8_t)(yPos + 10); // 字体高度大约 10
            ssd1306_InvertRectangle(0, y1, 127, y2);
        }

        yPos += 12;
    }

    // Draw Page Number (Top Right)
    char pageStr[10];
    snprintf(pageStr, sizeof(pageStr), "%d/%d", currentPage + 1, totalPages);
    uint8_t xPos = 128 - (strlen(pageStr) * 7);
    ssd1306_SetCursor(xPos, 0);
    ssd1306_WriteString(pageStr, Font_7x10, White);

    ssd1306_UpdateScreen();
}
