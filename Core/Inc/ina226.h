#ifndef __INA226_H
#define __INA226_H

#include "main.h"
#include <stdbool.h>
#include <stdint.h>

// INA226 I2C地址（7位，扫描结果0x44）
#define INA226_I2C_ADDR 0x44

// INA226 寄存器地址
#define INA226_REG_CONFIG       0x00
#define INA226_REG_SHUNT_VOLT   0x01
#define INA226_REG_BUS_VOLT     0x02
#define INA226_REG_POWER        0x03
#define INA226_REG_CURRENT      0x04
#define INA226_REG_CALIB        0x05

// 物理量换算常量
#define INA226_BUS_VOLTAGE_LSB_UV   1250    // 1.25mV -> 1250uV
#define INA226_CURRENT_LSB_UA       20      // 0.02mA -> 20uA（依赖校准值0x0A00）
#define INA226_POWER_LSB_UW         500     // 0.5mW -> 500uW

bool ina226_init(void);
bool ina226_read_bus_voltage_uv(int32_t *bus_voltage_uv);
bool ina226_read_shunt_voltage_uv(int32_t *shunt_voltage_uv);
bool ina226_read_current_ua(int32_t *current_ua);
bool ina226_read_power_uw(int32_t *power_uw);

#endif /* __INA226_H */
