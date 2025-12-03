#include "meg_adc.h"
#include "ADS1220.h"
#include "cmsis_os2.h"
#include "log.h"
#include "FreeRTOS.h"
#include "portmacro.h"
#include "task.h"

#define ADS1220_VREF 2.048f
#define ADS1220_GAIN 1.0f

static adc_result g_adc_result = {0};

typedef struct
{
    float k;
    float b;
} meg_adc_calibration;

const meg_adc_calibration adc_calibrations[4] = {
    {1.0f, 0.0f}, // Channel 1
    {1.0f, 0.0f}, // Channel 2
    {1.0f, 0.0f}, // Channel 3
    {1.0f, 0.0f}  // Channel 4
};

static float ADS1220_CodeToVoltage(long code)
{
    return ((float)code * ADS1220_VREF) / (ADS1220_GAIN * 8388608.0f);
}

void MegAdcInit()
{
    LOG_INFO("MegAdcInit start");
    ADS1220Init();
    ADS1220Config();
    LOG_INFO("MegAdcInit done");
}

void MegAdcHandler()
{
    HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);

    // Channel 1 (AIN0)
    ADS1220SetChannel(ADS1220_MUX_0_G);
    ADS1220SendStartCommand();
    ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(portMAX_DELAY));
    g_adc_result.l = ADS1220_CodeToVoltage(ADS1220ReadData());

    // Channel 2 (AIN1)
    ADS1220SetChannel(ADS1220_MUX_1_G);
    ADS1220SendStartCommand();
    ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(portMAX_DELAY));
    g_adc_result.lm = ADS1220_CodeToVoltage(ADS1220ReadData());

    // Channel 3 (AIN2)
    ADS1220SetChannel(ADS1220_MUX_2_G);
    ADS1220SendStartCommand();
    ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(portMAX_DELAY));
    g_adc_result.r = ADS1220_CodeToVoltage(ADS1220ReadData());

    // Channel 4 (AIN3)
    ADS1220SetChannel(ADS1220_MUX_3_G);
    ADS1220SendStartCommand();
    ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(portMAX_DELAY));
    g_adc_result.rm = ADS1220_CodeToVoltage(ADS1220ReadData());
}

adc_result MegAdcGetResult()
{
    return g_adc_result;
}
adc_result MegAdcGetCalibratedResult()
{
    adc_result raw = MegAdcGetResult();
    adc_result calibrated;
    calibrated.l = raw.l * adc_calibrations[0].k + adc_calibrations[0].b;
    calibrated.lm = raw.lm * adc_calibrations[1].k + adc_calibrations[1].b;
    calibrated.r = raw.r * adc_calibrations[2].k + adc_calibrations[2].b;
    calibrated.rm = raw.rm * adc_calibrations[3].k + adc_calibrations[3].b;
    return calibrated;
}