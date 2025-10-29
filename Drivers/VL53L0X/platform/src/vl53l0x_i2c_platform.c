#include "vl53l0x_i2c_platform.h"
#include "vl53l0x_platform.h"
#include "i2c.h"

int32_t VL53L0X_comms_initialise(uint8_t comms_type, uint16_t comms_speed_khz) {
    return 0;
}

int32_t VL53L0X_comms_close(void) {
    return 0;
}

int32_t VL53L0X_cycle_power(void) {
    return 0;
}

int32_t VL53L0X_write_multi(uint8_t address, uint8_t reg, uint8_t *pdata, int32_t count) {
    return HAL_I2C_Mem_Write(&hi2c1, address, reg, I2C_MEMADD_SIZE_8BIT, pdata, count, 100) == HAL_OK ? 0 : -1;
}

int32_t VL53L0X_read_multi(uint8_t address, uint8_t reg, uint8_t *pdata, int32_t count) {
    return HAL_I2C_Mem_Read(&hi2c1, address, reg, I2C_MEMADD_SIZE_8BIT, pdata, count, 100) == HAL_OK ? 0 : -1;
}

int32_t VL53L0X_write_byte(uint8_t address, uint8_t reg, uint8_t data) {
    return HAL_I2C_Mem_Write(&hi2c1, address, reg, I2C_MEMADD_SIZE_8BIT, &data, 1, 100) == HAL_OK ? 0 : -1;
}

int32_t VL53L0X_write_word(uint8_t address, uint8_t reg, uint16_t data) {
    uint8_t buffer[2];
    buffer[0] = (uint8_t)(data >> 8);
    buffer[1] = (uint8_t)(data & 0xFF);
    return HAL_I2C_Mem_Write(&hi2c1, address, reg, I2C_MEMADD_SIZE_8BIT, buffer, 2, 100) == HAL_OK ? 0 : -1;
}

int32_t VL53L0X_write_dword(uint8_t address, uint8_t reg, uint32_t data) {
    uint8_t buffer[4];
    buffer[0] = (uint8_t)(data >> 24);
    buffer[1] = (uint8_t)(data >> 16);
    buffer[2] = (uint8_t)(data >> 8);
    buffer[3] = (uint8_t)(data & 0xFF);
    return HAL_I2C_Mem_Write(&hi2c1, address, reg, I2C_MEMADD_SIZE_8BIT, buffer, 4, 100) == HAL_OK ? 0 : -1;
}

int32_t VL53L0X_read_byte(uint8_t address, uint8_t reg, uint8_t *data) {
    return HAL_I2C_Mem_Read(&hi2c1, address, reg, I2C_MEMADD_SIZE_8BIT, data, 1, 100) == HAL_OK ? 0 : -1;
}

int32_t VL53L0X_read_word(uint8_t address, uint8_t reg, uint16_t *data) {
    uint8_t buffer[2];
    int32_t status = HAL_I2C_Mem_Read(&hi2c1, address, reg, I2C_MEMADD_SIZE_8BIT, buffer, 2, 100);
    *data = ((uint16_t)buffer[0] << 8) | buffer[1];
    return status == HAL_OK ? 0 : -1;
}

int32_t VL53L0X_read_dword(uint8_t address, uint8_t reg, uint32_t *data) {
    uint8_t buffer[4];
    int32_t status = HAL_I2C_Mem_Read(&hi2c1, address, reg, I2C_MEMADD_SIZE_8BIT, buffer, 4, 100);
    *data = ((uint32_t)buffer[0] << 24) | ((uint32_t)buffer[1] << 16) | ((uint32_t)buffer[2] << 8) | buffer[3];
    return status == HAL_OK ? 0 : -1;
}
