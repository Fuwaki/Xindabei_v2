#pragma once
typedef struct
{
    float l, lm, m, rm, r;
} adc_result;
void MegAdcInit();
void MegAdcHandler();
adc_result MegAdcGetResult();
adc_result MegAdcGetCalibratedResult();