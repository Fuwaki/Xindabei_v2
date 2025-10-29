/*******************************************************************************
Copyright (C) 2015, STMicroelectronics International N.V.
All rights reserved.
*******************************************************************************/

#include "vl53l0x_platform.h"
#include "vl53l0x_i2c_platform.h"
#include "vl53l0x_api.h"
#include "i2c.h"
#include "main.h"

#define I2C_TIME_OUT_BASE   10
#define I2C_TIME_OUT_BYTE   1

// I2C通信函数实现
VL53L0X_Error VL53L0X_WriteMulti(VL53L0X_DEV Dev, uint8_t index, uint8_t *pdata, uint32_t count) {
    return HAL_I2C_Mem_Write(&hi2c1, Dev->I2cDevAddr, index, I2C_MEMADD_SIZE_8BIT, pdata, count, 100) == HAL_OK ? 0 : 1;
}

VL53L0X_Error VL53L0X_ReadMulti(VL53L0X_DEV Dev, uint8_t index, uint8_t *pdata, uint32_t count) {
    return HAL_I2C_Mem_Read(&hi2c1, Dev->I2cDevAddr, index, I2C_MEMADD_SIZE_8BIT, pdata, count, 100) == HAL_OK ? 0 : 1;
}

VL53L0X_Error VL53L0X_WrByte(VL53L0X_DEV Dev, uint8_t index, uint8_t data) {
    return HAL_I2C_Mem_Write(&hi2c1, Dev->I2cDevAddr, index, I2C_MEMADD_SIZE_8BIT, &data, 1, 100) == HAL_OK ? 0 : 1;
}

VL53L0X_Error VL53L0X_WrWord(VL53L0X_DEV Dev, uint8_t index, uint16_t data) {
    uint8_t buffer[2];
    buffer[0] = (uint8_t)(data >> 8);
    buffer[1] = (uint8_t)(data & 0xFF);
    return HAL_I2C_Mem_Write(&hi2c1, Dev->I2cDevAddr, index, I2C_MEMADD_SIZE_8BIT, buffer, 2, 100) == HAL_OK ? 0 : 1;
}

VL53L0X_Error VL53L0X_WrDWord(VL53L0X_DEV Dev, uint8_t index, uint32_t data) {
    uint8_t buffer[4];
    buffer[0] = (uint8_t)(data >> 24);
    buffer[1] = (uint8_t)(data >> 16);
    buffer[2] = (uint8_t)(data >> 8);
    buffer[3] = (uint8_t)(data & 0xFF);
    return HAL_I2C_Mem_Write(&hi2c1, Dev->I2cDevAddr, index, I2C_MEMADD_SIZE_8BIT, buffer, 4, 100) == HAL_OK ? 0 : 1;
}

VL53L0X_Error VL53L0X_RdByte(VL53L0X_DEV Dev, uint8_t index, uint8_t *data) {
    return HAL_I2C_Mem_Read(&hi2c1, Dev->I2cDevAddr, index, I2C_MEMADD_SIZE_8BIT, data, 1, 100) == HAL_OK ? 0 : 1;
}

VL53L0X_Error VL53L0X_RdWord(VL53L0X_DEV Dev, uint8_t index, uint16_t *data) {
    uint8_t buffer[2];
    VL53L0X_Error status = HAL_I2C_Mem_Read(&hi2c1, Dev->I2cDevAddr, index, I2C_MEMADD_SIZE_8BIT, buffer, 2, 100) == HAL_OK ? 0 : 1;
    *data = ((uint16_t)buffer[0] << 8) | buffer[1];
    return status;
}

VL53L0X_Error VL53L0X_RdDWord(VL53L0X_DEV Dev, uint8_t index, uint32_t *data) {
    uint8_t buffer[4];
    VL53L0X_Error status = HAL_I2C_Mem_Read(&hi2c1, Dev->I2cDevAddr, index, I2C_MEMADD_SIZE_8BIT, buffer, 4, 100) == HAL_OK ? 0 : 1;
    *data = ((uint32_t)buffer[0] << 24) | ((uint32_t)buffer[1] << 16) | ((uint32_t)buffer[2] << 8) | buffer[3];
    return status;
}

// Update byte 函数实现
VL53L0X_Error VL53L0X_UpdateByte(VL53L0X_DEV Dev, uint8_t index, uint8_t AndData, uint8_t OrData) {
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;
    uint8_t data;

    Status = VL53L0X_RdByte(Dev, index, &data);
    if (Status == VL53L0X_ERROR_NONE) {
        data = (data & AndData) | OrData;
        Status = VL53L0X_WrByte(Dev, index, data);
    }

    return Status;
}

// 延时函数实现
VL53L0X_Error VL53L0X_PollingDelay(VL53L0X_DEV Dev) {
    HAL_Delay(2);
    return VL53L0X_ERROR_NONE;
}
