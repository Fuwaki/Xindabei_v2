#ifndef __MYIIC_H
#define __MYIIC_H

#include "main.h"
#include <stdbool.h>

// I2C引脚定义
#define SCL_PIN GPIO_PIN_13  // SCL引脚
#define SDA_PIN  GPIO_PIN_12// SDA引脚
#define I2C_PORT GPIOB

// 函数声明
void i2c_gpio_init(void);
bool i2c_write_data(uint8_t dev_addr, uint8_t reg_addr, const uint8_t *data, uint8_t len);
bool i2c_read_data(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint8_t len);
uint8_t i2c_scan(uint8_t start_addr, uint8_t end_addr, uint8_t *found_list, uint8_t max_found);

#endif /* __MYIIC_H */