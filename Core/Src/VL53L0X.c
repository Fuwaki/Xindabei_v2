/*
 * vl53l0x.c
 * STM32 HAL 移植版，对应 vl53l0x.h
 * I2C 接口：HAL_I2C_Mem_Read / Write，7 位地址
 * 时间戳  ：HAL_GetTick()
 */
#include "VL53L0X.h"
#include <string.h>

/* ---------- 内部常量 ---------- */
#define ADDRESS_DEFAULT 0x52          // 7 位地址（0x29 左移一位）

/* ---------- 内部宏 ---------- */
#define startTimeout(dev)           ((dev)->timeout_start_ms = HAL_GetTick())
#define checkTimeoutExpired(dev)    ((dev)->io_timeout > 0 && \
                                     (uint16_t)(HAL_GetTick() - (dev)->timeout_start_ms) > (dev)->io_timeout)

/* ---------- 内部函数前向声明 ---------- */
static bool VL53L0X_getSpadInfo(VL53L0X_Dev_t *dev, uint8_t *count, bool *type_is_aperture);
static void VL53L0X_getSequenceStepEnables(VL53L0X_Dev_t *dev, SequenceStepEnables *enables);
static void VL53L0X_getSequenceStepTimeouts(VL53L0X_Dev_t *dev,
                                             SequenceStepEnables const *enables,
                                             SequenceStepTimeouts *timeouts);
static uint16_t VL53L0X_decodeTimeout(uint16_t reg_val);
static uint16_t VL53L0X_encodeTimeout(uint16_t timeout_mclks);
static uint32_t VL53L0X_timeoutMclksToMicroseconds(uint16_t timeout_mclks, uint8_t vcsel_period_pclks);
static uint32_t VL53L0X_timeoutMicrosecondsToMclks(uint32_t timeout_us, uint8_t vcsel_period_pclks);
static bool VL53L0X_performSingleRefCalibration(VL53L0X_Dev_t *dev, uint8_t vhv_init_byte);

/* ---------- 公有函数实现 ---------- */

void VL53L0X_setAddress(VL53L0X_Dev_t *dev, uint8_t new_addr)
{
    VL53L0X_writeReg(dev, I2C_SLAVE_DEVICE_ADDRESS, new_addr & 0x7F);
    dev->address = new_addr << 1;          // 保存为 HAL 所需的 8 位格式
}

uint8_t VL53L0X_getAddress(VL53L0X_Dev_t *dev)
{
    return dev->address >> 1;
}

bool VL53L0X_init(VL53L0X_Dev_t *dev, I2C_HandleTypeDef *hi2c, bool io_2v8)
{
    dev->hi2c = hi2c;
    dev->address = ADDRESS_DEFAULT;
    dev->io_timeout = 0;
    dev->did_timeout = 0;

    /* 1V8/2V8 模式选择 */
    if (io_2v8) {
        uint8_t v = VL53L0X_readReg(dev, VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV);
        VL53L0X_writeReg(dev, VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV, v | 0x01);
    }

    /* 以下流程与 Arduino 版本完全一致，仅替换读写函数 */
    VL53L0X_writeReg(dev, 0x88, 0x00);
    VL53L0X_writeReg(dev, 0x80, 0x01);
    VL53L0X_writeReg(dev, 0xFF, 0x01);
    VL53L0X_writeReg(dev, 0x00, 0x00);
    dev->stop_variable = VL53L0X_readReg(dev, 0x91);
    VL53L0X_writeReg(dev, 0x00, 0x01);
    VL53L0X_writeReg(dev, 0xFF, 0x00);
    VL53L0X_writeReg(dev, 0x80, 0x00);

    uint8_t msrc = VL53L0X_readReg(dev, MSRC_CONFIG_CONTROL);
    VL53L0X_writeReg(dev, MSRC_CONFIG_CONTROL, msrc | 0x12);

    VL53L0X_setSignalRateLimit(dev, 0.25f);
    VL53L0X_writeReg(dev, SYSTEM_SEQUENCE_CONFIG, 0xFF);

    /* SPAD 信息 & 参考 SPAD 配置 */
    uint8_t spad_count;
    bool spad_type_is_aperture;
    if (!VL53L0X_getSpadInfo(dev, &spad_count, &spad_type_is_aperture))
        return false;

    uint8_t ref_spad_map[6];
    VL53L0X_readMulti(dev, GLOBAL_CONFIG_SPAD_ENABLES_REF_0, ref_spad_map, 6);

    VL53L0X_writeReg(dev, 0xFF, 0x01);
    VL53L0X_writeReg(dev, DYNAMIC_SPAD_REF_EN_START_OFFSET, 0x00);
    VL53L0X_writeReg(dev, DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD, 0x2C);
    VL53L0X_writeReg(dev, 0xFF, 0x00);
    VL53L0X_writeReg(dev, GLOBAL_CONFIG_REF_EN_START_SELECT, 0xB4);

    uint8_t first = spad_type_is_aperture ? 12 : 0;
    uint8_t enabled = 0;
    for (int i = 0; i < 48; ++i) {
        if (i < first || enabled == spad_count)
            ref_spad_map[i / 8] &= ~(1 << (i % 8));
        else if ((ref_spad_map[i / 8] >> (i % 8)) & 0x01)
            ++enabled;
    }
    VL53L0X_writeMulti(dev, GLOBAL_CONFIG_SPAD_ENABLES_REF_0, ref_spad_map, 6);

    /* 加载默认调参寄存器（与 Arduino 相同，略） */
    static const uint8_t tuning[][2] = {
        {0xFF,0x01},{0x00,0x00},{0xFF,0x00},{0x09,0x00},{0x10,0x00},{0x11,0x00},
        {0x24,0x01},{0x25,0xFF},{0x75,0x00},{0xFF,0x01},{0x4E,0x2C},{0x48,0x00},
        {0x30,0x20},{0xFF,0x00},{0x30,0x09},{0x54,0x00},{0x31,0x04},{0x32,0x03},
        {0x40,0x83},{0x46,0x25},{0x60,0x00},{0x27,0x00},{0x50,0x06},{0x51,0x00},
        {0x52,0x96},{0x56,0x08},{0x57,0x30},{0x61,0x00},{0x62,0x00},{0x64,0x00},
        {0x65,0x00},{0x66,0xA0},{0xFF,0x01},{0x22,0x32},{0x47,0x14},{0x49,0xFF},
        {0x4A,0x00},{0xFF,0x00},{0x7A,0x0A},{0x7B,0x00},{0x78,0x21},{0xFF,0x01},
        {0x23,0x34},{0x42,0x00},{0x44,0xFF},{0x45,0x26},{0x46,0x05},{0x40,0x40},
        {0x0E,0x06},{0x20,0x1A},{0x43,0x40},{0xFF,0x00},{0x34,0x03},{0x35,0x44},
        {0xFF,0x01},{0x31,0x04},{0x4B,0x09},{0x4C,0x05},{0x4D,0x04},{0xFF,0x00},
        {0x44,0x00},{0x45,0x20},{0x47,0x08},{0x48,0x28},{0x67,0x00},{0x70,0x04},
        {0x71,0x01},{0x72,0xFE},{0x76,0x00},{0x77,0x00},{0xFF,0x01},{0x0D,0x01},
        {0xFF,0x00},{0x80,0x01},{0x01,0xF8},{0xFF,0x01},{0x8E,0x01},{0x00,0x01},
        {0xFF,0x00},{0x80,0x00}
    };
    for (size_t i = 0; i < sizeof(tuning)/sizeof(tuning[0]); ++i)
        VL53L0X_writeReg(dev, tuning[i][0], tuning[i][1]);

    /* 中断配置 */
    VL53L0X_writeReg(dev, SYSTEM_INTERRUPT_CONFIG_GPIO, 0x04);
    uint8_t hv = VL53L0X_readReg(dev, GPIO_HV_MUX_ACTIVE_HIGH);
    VL53L0X_writeReg(dev, GPIO_HV_MUX_ACTIVE_HIGH, hv & ~0x10);
    VL53L0X_writeReg(dev, SYSTEM_INTERRUPT_CLEAR, 0x01);

    dev->measurement_timing_budget_us = VL53L0X_getMeasurementTimingBudget(dev);

    VL53L0X_writeReg(dev, SYSTEM_SEQUENCE_CONFIG, 0xE8);
    VL53L0X_setMeasurementTimingBudget(dev, dev->measurement_timing_budget_us);

    /* VHV & Phase 校准 */
    VL53L0X_writeReg(dev, SYSTEM_SEQUENCE_CONFIG, 0x01);
    if (!VL53L0X_performSingleRefCalibration(dev, 0x40)) return false;

    VL53L0X_writeReg(dev, SYSTEM_SEQUENCE_CONFIG, 0x02);
    if (!VL53L0X_performSingleRefCalibration(dev, 0x00)) return false;

    VL53L0X_writeReg(dev, SYSTEM_SEQUENCE_CONFIG, 0xE8);
    return true;
}

/* ---------- 寄存器读写 ---------- */
void VL53L0X_writeReg(VL53L0X_Dev_t *dev, uint8_t reg, uint8_t value)
{
    dev->last_status = HAL_I2C_Mem_Write(dev->hi2c, dev->address, reg,
                                         I2C_MEMADD_SIZE_8BIT, &value, 1, dev->io_timeout);
}

void VL53L0X_writeReg16Bit(VL53L0X_Dev_t *dev, uint8_t reg, uint16_t value)
{
    uint8_t buf[2] = { (uint8_t)(value >> 8), (uint8_t)value };
    dev->last_status = HAL_I2C_Mem_Write(dev->hi2c, dev->address, reg,
                                         I2C_MEMADD_SIZE_8BIT, buf, 2, dev->io_timeout);
}

void VL53L0X_writeReg32Bit(VL53L0X_Dev_t *dev, uint8_t reg, uint32_t value)
{
    uint8_t buf[4] = { (uint8_t)(value >> 24), (uint8_t)(value >> 16),
                       (uint8_t)(value >> 8),  (uint8_t)value };
    dev->last_status = HAL_I2C_Mem_Write(dev->hi2c, dev->address, reg,
                                         I2C_MEMADD_SIZE_8BIT, buf, 4, dev->io_timeout);
}

uint8_t VL53L0X_readReg(VL53L0X_Dev_t *dev, uint8_t reg)
{
    uint8_t value = 0;
    dev->last_status = HAL_I2C_Mem_Read(dev->hi2c, dev->address, reg,
                                        I2C_MEMADD_SIZE_8BIT, &value, 1, dev->io_timeout);
    return value;
}

uint16_t VL53L0X_readReg16Bit(VL53L0X_Dev_t *dev, uint8_t reg)
{
    uint8_t buf[2];
    dev->last_status = HAL_I2C_Mem_Read(dev->hi2c, dev->address, reg,
                                        I2C_MEMADD_SIZE_8BIT, buf, 2, dev->io_timeout);
    return (uint16_t)buf[0] << 8 | buf[1];
}

uint32_t VL53L0X_readReg32Bit(VL53L0X_Dev_t *dev, uint8_t reg)
{
    uint8_t buf[4];
    dev->last_status = HAL_I2C_Mem_Read(dev->hi2c, dev->address, reg,
                                        I2C_MEMADD_SIZE_8BIT, buf, 4, dev->io_timeout);
    return (uint32_t)buf[0] << 24 | (uint32_t)buf[1] << 16 |
           (uint16_t)buf[2] << 8 | buf[3];
}

void VL53L0X_writeMulti(VL53L0X_Dev_t *dev, uint8_t reg, uint8_t const *src, uint8_t count)
{
    /* 如果数据量小，直接单帧发送；HAL 不支持重复起始位连续写，这里分两步 */
    uint8_t buf[32];
    if (count <= 32) {
        memcpy(buf, src, count);
        dev->last_status = HAL_I2C_Mem_Write(dev->hi2c, dev->address, reg,
                                             I2C_MEMADD_SIZE_8BIT, buf, count, dev->io_timeout);
    } else {
        /* 超长可分多次，或用户自行 DMA */
        dev->last_status = HAL_ERROR;
    }
}

void VL53L0X_readMulti(VL53L0X_Dev_t *dev, uint8_t reg, uint8_t *dst, uint8_t count)
{
    dev->last_status = HAL_I2C_Mem_Read(dev->hi2c, dev->address, reg,
                                        I2C_MEMADD_SIZE_8BIT, dst, count, dev->io_timeout);
}

/* ---------- 功能接口 ---------- */
bool VL53L0X_setSignalRateLimit(VL53L0X_Dev_t *dev, float limit_Mcps)
{
    if (limit_Mcps < 0 || limit_Mcps > 511.99f) return false;
    uint16_t val = (uint16_t)(limit_Mcps * (1 << 7));
    VL53L0X_writeReg16Bit(dev, FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT, val);
    return dev->last_status == HAL_OK;
}

float VL53L0X_getSignalRateLimit(VL53L0X_Dev_t *dev)
{
    return (float)VL53L0X_readReg16Bit(dev, FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT) / (1 << 7);
}

bool VL53L0X_setMeasurementTimingBudget(VL53L0X_Dev_t *dev, uint32_t budget_us)
{
    SequenceStepEnables enables;
    SequenceStepTimeouts timeouts;
    const uint16_t StartOverhead = 1320, EndOverhead = 960,
                   MsrcOverhead = 660, TccOverhead = 590,
                   DssOverhead = 690, PreRangeOverhead = 660,
                   FinalRangeOverhead = 550;
    const uint32_t MinTimingBudget = 20000;
    if (budget_us < MinTimingBudget) return false;

    uint32_t used = StartOverhead + EndOverhead;
    VL53L0X_getSequenceStepEnables(dev, &enables);
    VL53L0X_getSequenceStepTimeouts(dev, &enables, &timeouts);

    if (enables.tcc)      used += timeouts.msrc_dss_tcc_us + TccOverhead;
    if (enables.dss)      used += 2 * (timeouts.msrc_dss_tcc_us + DssOverhead);
    else if (enables.msrc) used += timeouts.msrc_dss_tcc_us + MsrcOverhead;
    if (enables.pre_range) used += timeouts.pre_range_us + PreRangeOverhead;
    if (enables.final_range) {
        used += FinalRangeOverhead;
        if (used > budget_us) return false;
        uint32_t final_us = budget_us - used;
        uint16_t final_mclks = VL53L0X_timeoutMicrosecondsToMclks(final_us,
                                                                    timeouts.final_range_vcsel_period_pclks);
        if (enables.pre_range) final_mclks += timeouts.pre_range_mclks;
        VL53L0X_writeReg16Bit(dev, FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI,
                              VL53L0X_encodeTimeout(final_mclks));
        dev->measurement_timing_budget_us = budget_us;
    }
    return true;
}

uint32_t VL53L0X_getMeasurementTimingBudget(VL53L0X_Dev_t *dev)
{
    SequenceStepEnables enables;
    SequenceStepTimeouts timeouts;
    const uint16_t StartOverhead = 1910, EndOverhead = 960,
                   MsrcOverhead = 660, TccOverhead = 590,
                   DssOverhead = 690, PreRangeOverhead = 660,
                   FinalRangeOverhead = 550;
    uint32_t budget = StartOverhead + EndOverhead;
    VL53L0X_getSequenceStepEnables(dev, &enables);
    VL53L0X_getSequenceStepTimeouts(dev, &enables, &timeouts);
    if (enables.tcc)      budget += timeouts.msrc_dss_tcc_us + TccOverhead;
    if (enables.dss)      budget += 2 * (timeouts.msrc_dss_tcc_us + DssOverhead);
    else if (enables.msrc) budget += timeouts.msrc_dss_tcc_us + MsrcOverhead;
    if (enables.pre_range) budget += timeouts.pre_range_us + PreRangeOverhead;
    if (enables.final_range) budget += timeouts.final_range_us + FinalRangeOverhead;
    dev->measurement_timing_budget_us = budget;
    return budget;
}

bool VL53L0X_setVcselPulsePeriod(VL53L0X_Dev_t *dev, vcselPeriodType type, uint8_t period_pclks)
{
    uint8_t vcsel_reg = ((period_pclks >> 1) - 1);
    SequenceStepEnables enables;
    SequenceStepTimeouts timeouts;
    VL53L0X_getSequenceStepEnables(dev, &enables);
    VL53L0X_getSequenceStepTimeouts(dev, &enables, &timeouts);

    if (type == VcselPeriodPreRange) {
        switch (period_pclks) {
            case 12: VL53L0X_writeReg(dev, PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x18); break;
            case 14: VL53L0X_writeReg(dev, PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x30); break;
            case 16: VL53L0X_writeReg(dev, PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x40); break;
            case 18: VL53L0X_writeReg(dev, PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x50); break;
            default: return false;
        }
        VL53L0X_writeReg(dev, PRE_RANGE_CONFIG_VALID_PHASE_LOW, 0x08);
        VL53L0X_writeReg(dev, PRE_RANGE_CONFIG_VCSEL_PERIOD, vcsel_reg);
        uint16_t new_msrc = VL53L0X_timeoutMicrosecondsToMclks(timeouts.msrc_dss_tcc_us, period_pclks);
        VL53L0X_writeReg(dev, MSRC_CONFIG_TIMEOUT_MACROP,
                         (new_msrc > 256) ? 255 : (new_msrc - 1));
        uint16_t new_pre = VL53L0X_timeoutMicrosecondsToMclks(timeouts.pre_range_us, period_pclks);
        VL53L0X_writeReg16Bit(dev, PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI,
                              VL53L0X_encodeTimeout(new_pre));
    } else if (type == VcselPeriodFinalRange) {
        switch (period_pclks) {
            case 8:
                VL53L0X_writeReg(dev, FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x10);
                VL53L0X_writeReg(dev, FINAL_RANGE_CONFIG_VALID_PHASE_LOW,  0x08);
                VL53L0X_writeReg(dev, GLOBAL_CONFIG_VCSEL_WIDTH, 0x02);
                VL53L0X_writeReg(dev, ALGO_PHASECAL_CONFIG_TIMEOUT, 0x0C);
                VL53L0X_writeReg(dev, 0xFF, 0x01);
                VL53L0X_writeReg(dev, ALGO_PHASECAL_LIM, 0x30);
                VL53L0X_writeReg(dev, 0xFF, 0x00);
                break;
            case 10:
                VL53L0X_writeReg(dev, FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x28);
                VL53L0X_writeReg(dev, FINAL_RANGE_CONFIG_VALID_PHASE_LOW,  0x08);
                VL53L0X_writeReg(dev, GLOBAL_CONFIG_VCSEL_WIDTH, 0x03);
                VL53L0X_writeReg(dev, ALGO_PHASECAL_CONFIG_TIMEOUT, 0x09);
                VL53L0X_writeReg(dev, 0xFF, 0x01);
                VL53L0X_writeReg(dev, ALGO_PHASECAL_LIM, 0x20);
                VL53L0X_writeReg(dev, 0xFF, 0x00);
                break;
            case 12:
                VL53L0X_writeReg(dev, FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x38);
                VL53L0X_writeReg(dev, FINAL_RANGE_CONFIG_VALID_PHASE_LOW,  0x08);
                VL53L0X_writeReg(dev, GLOBAL_CONFIG_VCSEL_WIDTH, 0x03);
                VL53L0X_writeReg(dev, ALGO_PHASECAL_CONFIG_TIMEOUT, 0x08);
                VL53L0X_writeReg(dev, 0xFF, 0x01);
                VL53L0X_writeReg(dev, ALGO_PHASECAL_LIM, 0x20);
                VL53L0X_writeReg(dev, 0xFF, 0x00);
                break;
            case 14:
                VL53L0X_writeReg(dev, FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x48);
                VL53L0X_writeReg(dev, FINAL_RANGE_CONFIG_VALID_PHASE_LOW,  0x08);
                VL53L0X_writeReg(dev, GLOBAL_CONFIG_VCSEL_WIDTH, 0x03);
                VL53L0X_writeReg(dev, ALGO_PHASECAL_CONFIG_TIMEOUT, 0x07);
                VL53L0X_writeReg(dev, 0xFF, 0x01);
                VL53L0X_writeReg(dev, ALGO_PHASECAL_LIM, 0x20);
                VL53L0X_writeReg(dev, 0xFF, 0x00);
                break;
            default: return false;
        }
        VL53L0X_writeReg(dev, FINAL_RANGE_CONFIG_VCSEL_PERIOD, vcsel_reg);
        uint16_t final_mclks = VL53L0X_timeoutMicrosecondsToMclks(timeouts.final_range_us, period_pclks);
        if (enables.pre_range) final_mclks += timeouts.pre_range_mclks;
        VL53L0X_writeReg16Bit(dev, FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI,
                              VL53L0X_encodeTimeout(final_mclks));
    } else return false;

    VL53L0X_setMeasurementTimingBudget(dev, dev->measurement_timing_budget_us);

    /* phase calib */
    uint8_t seq = VL53L0X_readReg(dev, SYSTEM_SEQUENCE_CONFIG);
    VL53L0X_writeReg(dev, SYSTEM_SEQUENCE_CONFIG, 0x02);
    VL53L0X_performSingleRefCalibration(dev, 0x00);
    VL53L0X_writeReg(dev, SYSTEM_SEQUENCE_CONFIG, seq);
    return true;
}

uint8_t VL53L0X_getVcselPulsePeriod(VL53L0X_Dev_t *dev, vcselPeriodType type)
{
    uint8_t reg = (type == VcselPeriodPreRange) ?
                  VL53L0X_readReg(dev, PRE_RANGE_CONFIG_VCSEL_PERIOD) :
                  VL53L0X_readReg(dev, FINAL_RANGE_CONFIG_VCSEL_PERIOD);
    return ((reg + 1) << 1);
}

/* ---------- 连续/单次测距 ---------- */
void VL53L0X_startContinuous(VL53L0X_Dev_t *dev, uint32_t period_ms)
{
    VL53L0X_writeReg(dev, 0x80, 0x01);
    VL53L0X_writeReg(dev, 0xFF, 0x01);
    VL53L0X_writeReg(dev, 0x00, 0x00);
    VL53L0X_writeReg(dev, 0x91, dev->stop_variable);
    VL53L0X_writeReg(dev, 0x00, 0x01);
    VL53L0X_writeReg(dev, 0xFF, 0x00);
    VL53L0X_writeReg(dev, 0x80, 0x00);

    if (period_ms != 0) {
        uint16_t osc = VL53L0X_readReg16Bit(dev, OSC_CALIBRATE_VAL);
        if (osc != 0) period_ms *= osc;
        VL53L0X_writeReg32Bit(dev, SYSTEM_INTERMEASUREMENT_PERIOD, period_ms);
        VL53L0X_writeReg(dev, SYSRANGE_START, 0x04);
    } else {
        VL53L0X_writeReg(dev, SYSRANGE_START, 0x02);
    }
}

void VL53L0X_stopContinuous(VL53L0X_Dev_t *dev)
{
    VL53L0X_writeReg(dev, SYSRANGE_START, 0x01);
    VL53L0X_writeReg(dev, 0xFF, 0x01);
    VL53L0X_writeReg(dev, 0x00, 0x00);
    VL53L0X_writeReg(dev, 0x91, 0x00);
    VL53L0X_writeReg(dev, 0x00, 0x01);
    VL53L0X_writeReg(dev, 0xFF, 0x00);
}

uint16_t VL53L0X_readRangeContinuousMillimeters(VL53L0X_Dev_t *dev)
{
    startTimeout(dev);
    while ((VL53L0X_readReg(dev, RESULT_INTERRUPT_STATUS) & 0x07) == 0) {
        if (checkTimeoutExpired(dev)) {
            dev->did_timeout = true;
            return 65535;
        }
    }
    uint16_t range = VL53L0X_readReg16Bit(dev, RESULT_RANGE_STATUS + 10);
    VL53L0X_writeReg(dev, SYSTEM_INTERRUPT_CLEAR, 0x01);
   );
    else if (enables.msrc) budget += timeouts.msrc_dss_tcc_us + MsrcOverhead;
    if (enables.pre_range) budget += timeouts.pre_range_us + PreRangeOverhead;
    if (enables.final_range) budget += timeouts.final_range_us + FinalRangeOverhead;
    dev->measurement_timing_budget_us = budget;
    return budget;
}

bool VL53L0X_setVcselPulsePeriod(VL53L0X_Dev_t *dev, vcselPeriodType type, uint8_t period_pclks)
{
    uint8_t vcsel_reg = VL53L0X_encodeVcselPeriod(period_pclks);
    SequenceStepEnables enables;
    SequenceStepTimeouts timeouts;
    VL53L0X_getSequenceStepEnables(dev, &enables);
    VL53L0X_getSequenceStepTimeouts(dev, &enables, &timeouts);

    if (type == VcselPeriodPreRange) {
        switch (period_pclks) {
            case 12: VL53L0X_writeReg(dev, PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x18); break;
            case 14: VL53L0X_writeReg(dev, PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x30); break;
            case 16: VL53L0X_writeReg(dev, PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x40); break;
            case 18: VL53L0X_writeReg(dev, PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x50); break;
            default: return false;
        }
        VL53L0X_writeReg(dev, PRE_RANGE_CONFIG_VALID_PHASE_LOW, 0x08);
        VL53L0X_writeReg(dev, PRE_RANGE_CONFIG_VCSEL_PERIOD, vcsel_reg);

        uint16_t new_mclks = VL53L0X_timeoutMicrosecondsToMclks(timeouts.pre_range_us, period_pclks);
        VL53L0X_writeReg16Bit(dev, PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI,
                              VL53L0X_encodeTimeout(new_mclks));

        new_mclks = VL53L0X_timeoutMicrosecondsToMclks(timeouts.msrc_dss_tcc_us, period_pclks);
        VL53L0X_writeReg(dev, MSRC_CONFIG_TIMEOUT_MACROP,
                         (new_mclks > 256) ? 255 : (new_mclks - 1));
    }
    else if (type == VcselPeriodFinalRange) {
        switch (period_pclks) {
            case 8:
                VL53L0X_writeReg(dev, FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x10);
                VL53L0X_writeReg(dev, FINAL_RANGE_CONFIG_VALID_PHASE_LOW, 0x08);
                VL53L0X_writeReg(dev, GLOBAL_CONFIG_VCSEL_WIDTH, 0x02);
                VL53L0X_writeReg(dev, ALGO_PHASECAL_CONFIG_TIMEOUT, 0x0C);
                VL53L0X_writeReg(dev, 0xFF, 0x01);
                VL53L0X_writeReg(dev, ALGO_PHASECAL_LIM, 0x30);
                VL53L0X_writeReg(dev, 0xFF, 0x00);
                break;
            case 10:
                VL53L0X_writeReg(dev, FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x28);
                VL53L0X_writeReg(dev, FINAL_RANGE_CONFIG_VALID_PHASE_LOW, 0x08);
                VL53L0X_writeReg(dev, GLOBAL_CONFIG_VCSEL_WIDTH, 0x03);
                VL53L0X_writeReg(dev, ALGO_PHASECAL_CONFIG_TIMEOUT, 0x09);
                VL53L0X_writeReg(dev, 0xFF, 0x01);
                VL53L0X_writeReg(dev, ALGO_PHASECAL_LIM, 0x20);
                VL53L0X_writeReg(dev, 0xFF, 0x00);
                break;
            case 12:
                VL53L0X_writeReg(dev, FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x38);
                VL53L0X_writeReg(dev, FINAL_RANGE_CONFIG_VALID_PHASE_LOW, 0x08);
                VL53L0X_writeReg(dev, GLOBAL_CONFIG_VCSEL_WIDTH, 0x03);
                VL53L0X_writeReg(dev, ALGO_PHASECAL_CONFIG_TIMEOUT, 0x08);
                VL53L0X_writeReg(dev, 0xFF, 0x01);
                VL53L0X_writeReg(dev, ALGO_PHASECAL_LIM, 0x20);
                VL53L0X_writeReg(dev, 0xFF, 0x00);
                break;
            case 14:
                VL53L0X_writeReg(dev, FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x48);
                VL53L0X_writeReg(dev, FINAL_RANGE_CONFIG_VALID_PHASE_LOW, 0x08);
                VL53L0X_writeReg(dev, GLOBAL_CONFIG_VCSEL_WIDTH, 0x03);
                VL53L0X_writeReg(dev, ALGO_PHASECAL_CONFIG_TIMEOUT, 0x07);
                VL53L0X_writeReg(dev, 0xFF, 0x01);
                VL53L0X_writeReg(dev, ALGO_PHASECAL_LIM, 0x20);
                VL53L0X_writeReg(dev, 0xFF, 0x00);
                break;
            default: return false;
        }
        VL53L0X_writeReg(dev, FINAL_RANGE_CONFIG_VCSEL_PERIOD, vcsel_reg);
        uint16_t new_mclks = VL53L0X_timeoutMicrosecondsToMclks(timeouts.final_range_us, period_pclks);
        if (enables.pre_range) new_mclks += timeouts.pre_range_mclks;
        VL53L0X_writeReg16Bit(dev, FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI,
                              VL53L0X_encodeTimeout(new_mclks));
    }
    else return false;

    VL53L0X_setMeasurementTimingBudget(dev, dev->measurement_timing_budget_us);

    uint8_t seq = VL53L0X_readReg(dev, SYSTEM_SEQUENCE_CONFIG);
    VL53L0X_writeReg(dev, SYSTEM_SEQUENCE_CONFIG, 0x02);
    VL53L0X_performSingleRefCalibration(dev, 0x00);
    VL53L0X_writeReg(dev, SYSTEM_SEQUENCE_CONFIG, seq);
    return true;
}

uint8_t VL53L0X_getVcselPulsePeriod(VL53L0X_Dev_t *dev, vcselPeriodType type)
{
    uint8_t reg = (type == VcselPeriodPreRange) ?
                  PRE_RANGE_CONFIG_VCSEL_PERIOD :
                  FINAL_RANGE_CONFIG_VCSEL_PERIOD;
    return VL53L0X_decodeVcselPeriod(VL53L0X_readReg(dev, reg));
}

/* ---------- 连续/单次测距 ---------- */
void VL53L0X_startContinuous(VL53L0X_Dev_t *dev, uint32_t period_ms)
{
    VL53L0X_writeReg(dev, 0x80, 0x01);
    VL53L0X_writeReg(dev, 0xFF, 0x01);
    VL53L0X_writeReg(dev, 0x00, 0x00);
    VL53L0X_writeReg(dev, 0x91, dev->stop_variable);
    VL53L0X_writeReg(dev, 0x00, 0x01);
    VL53L0X_writeReg(dev, 0xFF, 0x00);
    VL53L0X_writeReg(dev, 0x80, 0x00);

    if (period_ms) {
        uint16_t osc = VL53L0X_readReg16Bit(dev, OSC_CALIBRATE_VAL);
        if (osc) period_ms *= osc;
        VL53L0X_writeReg32Bit(dev, SYSTEM_INTERMEASUREMENT_PERIOD, period_ms);
        VL53L0X_writeReg(dev, SYSRANGE_START, 0x04);
    } else {
        VL53L0X_writeReg(dev, SYSRANGE_START, 0x02);
    }
}

void VL53L0X_stopContinuous(VL53L0X_Dev_t *dev)
{
    VL53L0X_writeReg(dev, SYSRANGE_START, 0x01);
    VL53L0X_writeReg(dev, 0xFF, 0x01);
    VL53L0X_writeReg(dev, 0x00, 0x00);
    VL53L0X_writeReg(dev, 0x91, 0x00);
    VL53L0X_writeReg(dev, 0x00, 0x01);
    VL53L0X_writeReg(dev, 0xFF, 0x00);
}

uint16_t VL53L0X_readRangeContinuousMillimeters(VL53L0X_Dev_t *dev)
{
    dev->timeout_start_ms = HAL_GetTick();
    while ((VL53L0X_readReg(dev, RESULT_INTERRUPT_STATUS) & 0x07) == 0) {
        if (dev->io_timeout && ((uint16_t)(HAL_GetTick() - dev->timeout_start_ms) > dev->io_timeout)) {
            dev->did_timeout = true;
            return 65535;
        }
    }
    uint16_t range = VL53L0X_readReg16Bit(dev, RESULT_RANGE_STATUS + 10);
    VL53L0X_writeReg(dev, SYSTEM_INTERRUPT_CLEAR, 0x01);
    return range;
}

uint16_t VL53L0X_readRangeSingleMillimeters(VL53L0X_Dev_t *dev)
{
    VL53L0X_writeReg(dev, 0x80, 0x01);
    VL53L0X_writeReg(dev, 0xFF, 0x01);
    VL53L0X_writeReg(dev, 0x00, 0x00);
    VL53L0X_writeReg(dev, 0x91, dev->stop_variable);
    VL53L0X_writeReg(dev, 0x00, 0x01);
    VL53L0X_writeReg(dev, 0xFF, 0x00);
    VL53L0X_writeReg(dev, 0x80, 0x00);

    VL53L0X_writeReg(dev, SYSRANGE_START, 0x01);

    dev->timeout_start_ms = HAL_GetTick();
    while (VL53L0X_readReg(dev, SYSRANGE_START) & 0x01) {
        if (dev->io_timeout && ((uint16_t)(HAL_GetTick() - dev->timeout_start_ms) > dev->io_timeout)) {
            dev->did_timeout = true;
            return 65535;
        }
    }
    return VL53L0X_readRangeContinuousMillimeters(dev);
}

/* ---------- 超时管理 ---------- */
void VL53L0X_setTimeout(VL53L0X_Dev_t *dev, uint16_t timeout)
{
    dev->io_timeout = timeout;
}

uint16_t VL53L0X_getTimeout(VL53L0X_Dev_t *dev)
{
    return dev->io_timeout;
}

bool VL53L0X_timeoutOccurred(VL53L0X_Dev_t *dev)
{
    bool t = dev->did_timeout;
    dev->did_timeout = false;
    return t;
}

/* ---------- 内部静态函数 ---------- */
static bool VL53L0X_getSpadInfo(VL53L0X_Dev_t *dev, uint8_t *count, bool *type_is_aperture)
{
    uint8_t tmp;
    VL53L0X_writeReg(dev, 0x80, 0x01);
    VL53L0X_writeReg(dev, 0xFF, 0x01);
    VL53L0X_writeReg(dev, 0x00, 0x00);
    VL53L0X_writeReg(dev, 0xFF, 0x06);
    VL53L0X_writeReg(dev, 0x83, VL53L0X_readReg(dev, 0x83) | 0x04);
    VL53L0X_writeReg(dev, 0xFF, 0x07);
    VL53L0X_writeReg(dev, 0x81, 0x01);
    VL53L0X_writeReg(dev, 0x80, 0x01);
    VL53L0X_writeReg(dev, 0x94, 0x6B);
    VL53L0X_writeReg(dev, 0x83, 0x00);
    dev->timeout_start_ms = HAL_GetTick();
    while (VL53L0X_readReg(dev, 0x83) == 0x00) {
        if (dev->io_timeout && ((uint16_t)(HAL_GetTick() - dev->timeout_start_ms) > dev->io_timeout))
            return false;
    }
    VL53L0X_writeReg(dev, 0x83, 0x01);
    tmp = VL53L0X_readReg(dev, 0x92);
    *count = tmp & 0x7F;
    *type_is_aperture = (tmp >> 7) & 0x01;

    VL53L0X_writeReg(dev, 0x81, 0x00);
    VL53L0X_writeReg(dev, 0xFF, 0x06);
    VL53L0X_writeReg(dev, 0x83, VL53L0X_readReg(dev, 0x83) & ~0x04);
    VL53L0X_writeReg(dev, 0xFF, 0x01);
    VL53L0X_writeReg(dev, 0x00, 0x01);
    VL53L0X_writeReg(dev, 0xFF, 0x00);
    VL53L0X_writeReg(dev, 0x80, 0x00);
    return true;
}

static void VL53L0X_getSequenceStepEnables(VL53L0X_Dev_t *dev, SequenceStepEnables *enables)
{
    uint8_t sc = VL53L0X_readReg(dev, SYSTEM_SEQUENCE_CONFIG);
    enables->tcc         = (sc >> 4) & 0x1;
    enables->dss         = (sc >> 3) & 0x1;
    enables->msrc        = (sc >> 2) & 0x1;
    enables->pre_range   = (sc >> 6) & 0x1;
    enables->final_range = (sc >> 7) & 0x1;
}

static void VL53L0X_getSequenceStepTimeouts(VL53L0X_Dev_t *dev,
                                             const SequenceStepEnables *enables,
                                             SequenceStepTimeouts *timeouts)
{
    timeouts->pre_range_vcsel_period_pclks = VL53L0X_getVcselPulsePeriod(dev, VcselPeriodPreRange);
    timeouts->msrc_dss_tcc_mclks = VL53L0X_readReg(dev, MSRC_CONFIG_TIMEOUT_MACROP) + 1;
    timeouts->msrc_dss_tcc_us =
        VL53L0X_timeoutMclksToMicroseconds(timeouts->msrc_dss_tcc_mclks,
                                            timeouts->pre_range_vcsel_period_pclks);

    timeouts->pre_range_mclks = VL53L0X_decodeTimeout(
        VL53L0X_readReg16Bit(dev, PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI));
    timeouts->pre_range_us =
        VL53L0X_timeoutMclksToMicroseconds(timeouts->pre_range_mclks,
                                            timeouts->pre_range_vcsel_period_pclks);

    timeouts->final_range_vcsel_period_pclks = VL53L0X_getVcselPulsePeriod(dev, VcselPeriodFinalRange);
    timeouts->final_range_mclks = VL53L0X_decodeTimeout(
        VL53L0X_readReg16Bit(dev, FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI));
    if (enables->pre_range)
        timeouts->final_range_mclks -= timeouts->pre_range_mclks;
    timeouts->final_range_us =
        VL53L0X_timeoutMclksToMicroseconds(timeouts->final_range_mclks,
                                            timeouts->final_range_vcsel_period_pclks);
}

static uint16_t VL53L0X_decodeTimeout(uint16_t reg_val)
{
    return (uint16_t)((reg_val & 0x00FF) <<
                      (uint16_t)((reg_val & 0xFF00) >> 8)) + 1;
}

static uint16_t VL53L0X_encodeTimeout(uint16_t timeout_mclks)
{
    if (timeout_mclks == 0) return 0;
    uint32_t ls = timeout_mclks - 1;
    uint16_t ms = 0;
    while ((ls & 0xFFFFFF00) > 0) { ls >>= 1; ms++; }
    return (ms << 8) | (ls & 0xFF);
}

static uint32_t VL53L0X_timeoutMclksToMicroseconds(uint16_t mclks, uint8_t vcsel_period_pclks)
{
    uint32_t macro_ns = (((uint32_t)2304 * vcsel_period_pclks * 1655) + 500) / 1000;
    return (mclks * macro_ns + (macro_ns / 2)) / 1000;
}

static uint32_t VL53L0X_timeoutMicrosecondsToMclks(uint32_t us, uint8_t vcsel_period_pclks)
{
    uint32_t macro_ns = (((uint32_t)2304 * vcsel_period_pclks * 1655) + 500) / 1000;
    return ((us * 1000) + (macro_ns / 2)) / macro_ns;
}

static bool VL53L0X_performSingleRefCalibration(VL53L0X_Dev_t *dev, uint8_t vhv_init_byte)
{
    VL53L0X_writeReg(dev, SYSRANGE_START, 0x01 | vhv_init_byte);
    dev->timeout_start_ms = HAL_GetTick();
    while ((VL53L0X_readReg(dev, RESULT_INTERRUPT_STATUS) & 0x07) == 0) {
        if (dev->io_timeout && ((uint16_t)(HAL_GetTick() - dev->timeout_start_ms) > dev->io_timeout))
            return false;
    }
    VL53L0X_writeReg(dev, SYSTEM_INTERRUPT_CLEAR, 0x01);
    VL53L0X_writeReg(dev, SYSRANGE_START, 0x00);
    return true;
}