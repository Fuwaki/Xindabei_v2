
#include "myiic.h"
#include "ina226.h"

#include <limits.h>
#include <stdio.h>

/************************************************
 * 内部函数：INA226寄存器写入
*************************************************/
static bool ina226_write_reg(uint8_t reg, uint16_t value)
{
  uint8_t payload[2] = { (uint8_t)((value >> 8) & 0xFF), (uint8_t)(value & 0xFF) };
  return i2c_write_data(INA226_I2C_ADDR, reg, payload, 2);
}

/************************************************
 * 内部函数：INA226寄存器读取
*************************************************/
static bool ina226_read_reg(uint8_t reg, uint16_t *value)
{
  uint8_t payload[2];

  if (!i2c_read_data(INA226_I2C_ADDR, reg, payload, 2))
  {
    return false;
  }

  *value = ((uint16_t)payload[0] << 8) | payload[1];
  return true;
}

/************************************************
 * 函 数 名 称：ina226_init
 * 函 数 说 明：INA226初始化配置
 * 函 数 形 参：无
 * 函 数 返 回：bool 成功/失败
 * 作       者：LC
 * 备       注：无
*************************************************/
bool ina226_init(void)
{
  // 软复位
  if (!ina226_write_reg(INA226_REG_CONFIG, 0x8000))
  {
    printf("INA226: soft reset write failed\r\n");
    return false;
  }

  // 设置平均次数、转换时间、连续模式（16次平均，Vsh/Vbus 1.1ms，连续测量）
  if (!ina226_write_reg(INA226_REG_CONFIG, 0x4527))
  {
    printf("INA226: config write failed\r\n");
    return false;
  }

  // 校准寄存器，决定current/power LSB（与示例一致，LSB=0.02mA）
  if (!ina226_write_reg(INA226_REG_CALIB, 0x0A00))
  {
    printf("INA226: calib write failed\r\n");
    return false;
  }

  uint16_t man_id = 0;
  if (ina226_read_reg(0xFE, &man_id))
  {
    printf("INA226: manufacturer ID=0x%04X\r\n", man_id);
  }
  else
  {
    printf("INA226: read manufacturer ID failed\r\n");
    return false;
  }

  uint16_t die_id = 0;
  if (ina226_read_reg(0xFF, &die_id))
  {
    printf("INA226: die ID=0x%04X\r\n", die_id);
  }
  else
  {
    printf("INA226: read die ID failed\r\n");
    return false;
  }

  return true;
}

/************************************************
 * 函 数 名 称：ina226_read_bus_voltage_uv
 * 函 数 说 明：获取总线电压（单位：uV）
 * 函 数 形 参：输出指针
 * 函 数 返 回：bool 成功/失败
 * 作       者：LC
 * 备       注：无
*************************************************/
bool ina226_read_bus_voltage_uv(int32_t *bus_voltage_uv)
{
  uint16_t raw;
  if (!ina226_read_reg(INA226_REG_BUS_VOLT, &raw))
  {
    return false;
  }

  *bus_voltage_uv = (int32_t)raw * INA226_BUS_VOLTAGE_LSB_UV;
  return true;
}

/************************************************
 * 函 数 名 称：ina226_read_shunt_voltage_uv
 * 函 数 说 明：获取分流电压（单位：uV）
 * 函 数 形 参：输出指针
 * 函 数 返 回：bool 成功/失败
 * 作       者：LC
 * 备       注：无
*************************************************/
bool ina226_read_shunt_voltage_uv(int32_t *shunt_voltage_uv)
{
  uint16_t raw;
  if (!ina226_read_reg(INA226_REG_SHUNT_VOLT, &raw))
  {
    return false;
  }

  // Shunt电压寄存器是补码格式，LSB=2.5uV
  int16_t signed_raw = (int16_t)raw;
  int64_t nv = (int64_t)signed_raw * 2500; // 2.5uV = 2500nV
  if (nv >= 0)
  {
    nv += 500; // 四舍五入到uV
  }
  else
  {
    nv -= 500;
  }
  *shunt_voltage_uv = (int32_t)(nv / 1000);
  return true;
}

/************************************************
 * 函 数 名 称：ina226_read_current_ua
 * 函 数 说 明：获取电流值（单位：uA）
 * 函 数 形 参：输出指针
 * 函 数 返 回：bool 成功/失败
 * 作       者：LC
 * 备       注：无
*************************************************/
bool ina226_read_current_ua(int32_t *current_ua)
{
  uint16_t raw;
  if (!ina226_read_reg(INA226_REG_CURRENT, &raw))
  {
    return false;
  }

  int16_t signed_raw = (int16_t)raw;
  *current_ua = (int32_t)signed_raw * INA226_CURRENT_LSB_UA;
  return true;
}

/************************************************
 * 函 数 名 称：ina226_read_power_uw
 * 函 数 说 明：获取功率值（单位：uW）
 * 函 数 形 参：输出指针
 * 函 数 返 回：bool 成功/失败
 * 作       者：LC
 * 备       注：无
*************************************************/
bool ina226_read_power_uw(int32_t *power_uw)
{
  uint16_t raw;
  if (!ina226_read_reg(INA226_REG_POWER, &raw))
  {
    return false;
  }

  *power_uw = (int32_t)raw * INA226_POWER_LSB_UW;
  return true;
}