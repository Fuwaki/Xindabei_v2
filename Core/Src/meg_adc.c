#include "meg_adc.h"
#include "ADS1220.h"
#include "FreeRTOS.h"
#include "adc.h"
#include "cmsis_os2.h"
#include "log.h"
#include "param_server.h"
#include "portmacro.h"
#include "task.h"
#include <math.h>

#define ADS1220_VREF 2.048f
#define ADS1220_GAIN 1.0f

static adc_result g_adc_result = {0};
static uint16_t adc1_buffer = 0;

typedef struct
{
    float k;
    float b;
} meg_adc_calibration;

static meg_adc_calibration adc_calibrations[5] = {
    {1.f /0.783, 0.0f},
    {1.f / 0.857, 0.0f},
    {1.f/0.4, 0.0f},
    {1.f / 0.826, 0.0f}, //
    {1.f / 0.873, 0.0f}  //
};

static float ADS1220_CodeToVoltage(long code)
{
    return ((float)code * ADS1220_VREF) / (ADS1220_GAIN * 8388608.0f);
}

/* Kick single-shot conversion and wait for DRDY, clearing any stale notify first. */
static void ADS1220_StartAndWait(void)
{
    (void)ulTaskNotifyTake(pdTRUE, 0);
    ADS1220SendStartCommand();
    ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(50));
}

static float ADC1_CodeToVoltage(uint16_t code)
{
    return ((float)code * 3.3f) / 4096.0f;
}

/* 原始/校准后的读数，方便观察调参效果 */
static float GetMegLRaw(void)
{
    return g_adc_result.l;
}
static float GetMegLMRaw(void)
{
    return g_adc_result.lm;
}
static float GetMegMRaw(void)
{
    return g_adc_result.m;
}
static float GetMegRRaw(void)
{
    return g_adc_result.r;
}
static float GetMegRMRaw(void)
{
    return g_adc_result.rm;
}

static float GetMegLCal(void)
{
    adc_result c = MegAdcGetCalibratedResult();
    return c.l;
}
static float GetMegLMCal(void)
{
    adc_result c = MegAdcGetCalibratedResult();
    return c.lm;
}
static float GetMegMCal(void)
{
    adc_result c = MegAdcGetCalibratedResult();
    return c.m;
}
static float GetMegRCal(void)
{
    adc_result c = MegAdcGetCalibratedResult();
    return c.r;
}
static float GetMegRMCal(void)
{
    adc_result c = MegAdcGetCalibratedResult();
    return c.rm;
}

/* 注册 MEG 校准及观测参数 */
static void MegAdcRegisterParams(void)
{
    static ParamDesc meg_params[] = {
        // /* 原始值，只读，可选只在串口或 OLED，这里两边都看 */
        // {.name = "M_l_raw",
        //  .type = PARAM_TYPE_FLOAT,
        //  .ops.f.get = GetMegLRaw,
        //  .read_only = 1,
        //  .mask = PARAM_MASK_SERIAL | PARAM_MASK_OLED},
        // {.name = "M_lm_raw",
        //  .type = PARAM_TYPE_FLOAT,
        //  .ops.f.get = GetMegLMRaw,
        //  .read_only = 1,
        //  .mask = PARAM_MASK_SERIAL | PARAM_MASK_OLED},
        // {.name = "M_m_raw",
        //  .type = PARAM_TYPE_FLOAT,
        //  .ops.f.get = GetMegMRaw,
        //  .read_only = 1,
        //  .mask = PARAM_MASK_SERIAL | PARAM_MASK_OLED},
        // {.name = "M_rm_raw",
        //  .type = PARAM_TYPE_FLOAT,
        //  .ops.f.get = GetMegRMRaw,
        //  .read_only = 1,
        //  .mask = PARAM_MASK_SERIAL | PARAM_MASK_OLED},
        // {.name = "M_r_raw",
        //  .type = PARAM_TYPE_FLOAT,
        //  .ops.f.get = GetMegRRaw,
        //  .read_only = 1,
        //  .mask = PARAM_MASK_SERIAL | PARAM_MASK_OLED},

        // /* 校准后值，只读 */
        {.name = "M_l_cal",
         .type = PARAM_TYPE_FLOAT,
         .ops.f.get = GetMegLCal,
         .read_only = 1,
         .mask =  PARAM_MASK_SERIAL |PARAM_MASK_OLED},
        {.name = "M_lm_cal",
         .type = PARAM_TYPE_FLOAT,
         .ops.f.get = GetMegLMCal,
         .read_only = 1,
         .mask =  PARAM_MASK_SERIAL |PARAM_MASK_OLED},
        {.name = "M_m_cal",
         .type = PARAM_TYPE_FLOAT,
         .ops.f.get = GetMegMCal,
         .read_only = 1,
         .mask =  PARAM_MASK_SERIAL |PARAM_MASK_OLED},
        {.name = "M_rm_cal",
         .type = PARAM_TYPE_FLOAT,
         .ops.f.get = GetMegRMCal,
         .read_only = 1,
         .mask =  PARAM_MASK_SERIAL |PARAM_MASK_OLED},
        {.name = "M_r_cal",
         .type = PARAM_TYPE_FLOAT,
         .ops.f.get = GetMegRCal,
         .read_only = 1,
         .mask =  PARAM_MASK_SERIAL |PARAM_MASK_OLED},
    };

    for (int i = 0; i < (int)(sizeof(meg_params) / sizeof(meg_params[0])); ++i)
    {
        ParamServer_Register(&meg_params[i]);
    }
}

void MegAdcInit()
{
    LOG_INFO("MegAdcInit start");
    ADS1220Init();
    ADS1220Config();

    // 开启dma
    if (HAL_ADC_Start_DMA(&hadc1, (uint32_t *)&adc1_buffer, 1) != HAL_OK)
    {
        LOG_ERROR("ADC1 Start DMA failed");
    }
    
    MegAdcRegisterParams();
    LOG_INFO("MegAdcInit done");
}

void MegAdcHandler()
{
    HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);

    HAL_ADC_Stop_DMA(&hadc1);
    __HAL_ADC_CLEAR_FLAG(&hadc1, ADC_FLAG_OVR);
    // 触发adc采样 规则通道
    if (HAL_ADC_Start_DMA(&hadc1, (uint32_t *)&adc1_buffer, 1) != HAL_OK)
    {
        LOG_ERROR("ADC1 Start DMA failed");
    }

    // Channel 1 (AIN0)
    ADS1220SetChannel(ADS1220_MUX_0_G);
    ADS1220_StartAndWait();
    g_adc_result.lm = ADS1220_CodeToVoltage(ADS1220ReadData());

    // Channel 2 (AIN1)
    ADS1220SetChannel(ADS1220_MUX_1_G);
    ADS1220_StartAndWait();
    g_adc_result.l = ADS1220_CodeToVoltage(ADS1220ReadData());

    // Channel 3 (AIN2)
    ADS1220SetChannel(ADS1220_MUX_2_G);
    ADS1220_StartAndWait();
    g_adc_result.r = ADS1220_CodeToVoltage(ADS1220ReadData());

    // Channel 4 (AIN3)
    ADS1220SetChannel(ADS1220_MUX_3_G);
    ADS1220_StartAndWait();
    g_adc_result.rm = ADS1220_CodeToVoltage(ADS1220ReadData());

    // g_adc_result.m = ADC1_CodeToVoltage(adc1_buffer);
    g_adc_result.m=fabsf(g_adc_result.l)+fabsf(g_adc_result.lm)+fabsf(g_adc_result.r)+fabsf(g_adc_result.rm);
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
    calibrated.rm = raw.rm * adc_calibrations[3].k + adc_calibrations[3].b;
    calibrated.r = raw.r * adc_calibrations[4].k + adc_calibrations[4].b;
    // calibrated.m = raw.m * adc_calibrations[2].k + adc_calibrations[2].b;
    calibrated.m=fabsf(calibrated.l)+fabsf(calibrated.r);
    return calibrated;
}