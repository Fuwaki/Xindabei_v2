#ifndef OLED_DISPLAY_H
#define OLED_DISPLAY_H

#include "meg_adc.h"

void OledDisplayInit(void);
void OledDisplayUpdate(float cpuUsage, adc_result adcData);

#endif // OLED_DISPLAY_H
