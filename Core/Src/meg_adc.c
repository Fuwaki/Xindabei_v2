#include "meg_adc.h"
#include "ADS1220.h"
#include "log.h"
void MegAdcInit()
{
    LOG_INFO("MegAdcInit start");
    ADS1220Init();
    LOG_INFO("MegAdcInit done");
}
void MegAdcHandler()
{
    static int first = 1;
    if (first) { LOG_INFO("MegAdcHandler loop entered"); first = 0; }
}
adc_result GetAdcResult()
{
    adc_result r = {0};
    return r;
}