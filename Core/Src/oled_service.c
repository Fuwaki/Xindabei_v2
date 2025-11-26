#include "oled_service.h"
#include "ssd1306.h"
#include "ssd1306_fonts.h"
#include <stdio.h>
#include <string.h>

#define MAX_DISPLAY_ITEMS 20
#define ITEMS_PER_PAGE 5

typedef struct {
    const char* label;
    DisplayType type;
    union {
        IntSourceCb intCb;
        FloatSourceCb floatCb;
        StringSourceCb stringCb;
    } callback;
} DisplayItem;

static DisplayItem s_items[MAX_DISPLAY_ITEMS];
static uint8_t s_itemCount = 0;
static uint8_t s_currentPage = 0;

void OledServiceInit(void)
{
    ssd1306_Init();
    s_itemCount = 0;
    s_currentPage = 0;
}

bool OledRegisterInt(const char* label, IntSourceCb callback)
{
    if (s_itemCount >= MAX_DISPLAY_ITEMS) return false;
    
    s_items[s_itemCount].label = label;
    s_items[s_itemCount].type = DISPLAY_TYPE_INT;
    s_items[s_itemCount].callback.intCb = callback;
    s_itemCount++;
    return true;
}

bool OledRegisterFloat(const char* label, FloatSourceCb callback)
{
    if (s_itemCount >= MAX_DISPLAY_ITEMS) return false;
    
    s_items[s_itemCount].label = label;
    s_items[s_itemCount].type = DISPLAY_TYPE_FLOAT;
    s_items[s_itemCount].callback.floatCb = callback;
    s_itemCount++;
    return true;
}

bool OledRegisterString(const char* label, StringSourceCb callback)
{
    if (s_itemCount >= MAX_DISPLAY_ITEMS) return false;
    
    s_items[s_itemCount].label = label;
    s_items[s_itemCount].type = DISPLAY_TYPE_STRING;
    s_items[s_itemCount].callback.stringCb = callback;
    s_itemCount++;
    return true;
}

void OledNextPage(void)
{
    uint8_t totalPages = (s_itemCount + ITEMS_PER_PAGE - 1) / ITEMS_PER_PAGE;
    if (totalPages == 0) return;
    
    s_currentPage++;
    if (s_currentPage >= totalPages) {
        s_currentPage = 0;
    }
}

void OledPrevPage(void)
{
    uint8_t totalPages = (s_itemCount + ITEMS_PER_PAGE - 1) / ITEMS_PER_PAGE;
    if (totalPages == 0) return;

    if (s_currentPage == 0) {
        s_currentPage = totalPages - 1;
    } else {
        s_currentPage--;
    }
}

void OledServiceUpdate(void)
{
    ssd1306_Fill(Black);

    if (s_itemCount == 0) {
        ssd1306_SetCursor(0, 0);
        ssd1306_WriteString("No Data", Font_7x10, White);
        ssd1306_UpdateScreen();
        return;
    }

    uint8_t totalPages = (s_itemCount + ITEMS_PER_PAGE - 1) / ITEMS_PER_PAGE;
    
    // Ensure current page is valid
    if (s_currentPage >= totalPages) s_currentPage = 0;

    // Draw Items
    uint8_t startIdx = s_currentPage * ITEMS_PER_PAGE;
    uint8_t endIdx = startIdx + ITEMS_PER_PAGE;
    if (endIdx > s_itemCount) endIdx = s_itemCount;

    uint8_t yPos = 0; 

    for (uint8_t i = startIdx; i < endIdx; i++) {
        char buffer[32];
        DisplayItem* item = &s_items[i];
        
        ssd1306_SetCursor(0, yPos);
        
        switch (item->type) {
            case DISPLAY_TYPE_INT:
                snprintf(buffer, sizeof(buffer), "%s: %d", item->label, (int)item->callback.intCb());
                break;
            case DISPLAY_TYPE_FLOAT:
                snprintf(buffer, sizeof(buffer), "%s: %.4g", item->label, item->callback.floatCb());
                break;
            case DISPLAY_TYPE_STRING:
                snprintf(buffer, sizeof(buffer), "%s: %s", item->label, item->callback.stringCb());
                break;
        }
        
        ssd1306_WriteString(buffer, Font_7x10, White);
        yPos += 12;
    }

    // Draw Page Number (Top Right)
    char pageStr[10];
    snprintf(pageStr, sizeof(pageStr), "%d/%d", s_currentPage + 1, totalPages);
    uint8_t xPos = 128 - (strlen(pageStr) * 7);
    ssd1306_SetCursor(xPos, 0);
    ssd1306_WriteString(pageStr, Font_7x10, White);

    ssd1306_UpdateScreen();
}
