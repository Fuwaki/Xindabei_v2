#include "oled_display.h"
#include "ssd1306.h"
#include "ssd1306_fonts.h"
#include <stdio.h>

void OledDisplayInit(void)
{
    ssd1306_Init();
}

void OledDisplayUpdate(float cpuUsage, adc_result adcData)
{
    ssd1306_Fill(Black);

    char buffer[32];
    snprintf(buffer, sizeof(buffer), "CPU: %.1f%%", cpuUsage);
    ssd1306_SetCursor(0, 0);
    ssd1306_WriteString(buffer, Font_7x10, White);

    ssd1306_SetCursor(0, 12);
    snprintf(buffer, sizeof(buffer), "CH1: %.5f", adcData.l);
    ssd1306_WriteString(buffer, Font_7x10, White);

    ssd1306_SetCursor(0, 24);
    snprintf(buffer, sizeof(buffer), "CH2: %.5f", adcData.lm);
    ssd1306_WriteString(buffer, Font_7x10, White);

    ssd1306_SetCursor(0, 36);
    snprintf(buffer, sizeof(buffer), "CH3: %.5f", adcData.r);
    ssd1306_WriteString(buffer, Font_7x10, White);

    ssd1306_SetCursor(0, 48);
    snprintf(buffer, sizeof(buffer), "CH4: %.5f", adcData.rm);
    ssd1306_WriteString(buffer, Font_7x10, White);

    ssd1306_UpdateScreen();
}
