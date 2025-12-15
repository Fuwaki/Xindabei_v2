#include "oled_service.h"
#include "ssd1306.h"
#include "ssd1306_fonts.h"
#include <stdio.h>
#include <string.h>
#include "common.h"
#include "param_server.h"

#define ITEMS_PER_PAGE 5

static int s_currentIdx = 0;
static bool s_displayOn = true;

void OledServiceInit(void)
{
    ssd1306_Init();
    s_currentIdx = 0;
    s_displayOn = true;
}

void OledServiceToggleDisplay(void)
{
    s_displayOn = !s_displayOn;
    if (!s_displayOn) {
        ssd1306_Fill(Black);
        ssd1306_UpdateScreen();
    }
}

void OledHandleKey(OledKeyEvent event)
{
    int count = ParamServer_GetCount();
    if (count == 0) return;

    switch (event) {
        case OLED_KEY_NEXT:
        
            // 按页翻动，每页显示ITEMS_PER_PAGE个项目
            s_currentIdx += ITEMS_PER_PAGE;
            if (s_currentIdx >= count) {
                s_currentIdx = 0;
            }
            break;
    }
}

void OledServiceUpdate(void)
{
    if (!s_displayOn) return;

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
    }

    // Draw Page Number (Top Right)
    char pageStr[16];  // 增加缓冲区大小以避免截断警告
    snprintf(pageStr, sizeof(pageStr), "%d/%d", currentPage + 1, totalPages);
    uint8_t xPos = 128 - (strlen(pageStr) * 7);
    ssd1306_SetCursor(xPos, 0);
    ssd1306_WriteString(pageStr, Font_7x10, White);

    ssd1306_UpdateScreen();
}
