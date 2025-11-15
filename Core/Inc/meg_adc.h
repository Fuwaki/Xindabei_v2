#pragma once
typedef struct
{
    float l, lm, rm, r;
} adc_result;
void MegAdcInit();
void MegAdcHandler();
adc_result GetAdcResult();