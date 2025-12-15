#include "imu.h"
#include "lsm6ds3_reg.h"
#include "log.h"
#include "spi.h"

// LSM6DS3 SPI接口实现
static int32_t lsm6ds3_spi_write(void *handle, uint8_t reg, const uint8_t *data, uint16_t len)
{
    uint8_t tx_buf[len + 1];
    
    // 添加写命令位
    tx_buf[0] = reg & 0x7F; // 清除最高位，确保是写操作
    
    // 复制数据到发送缓冲区
    for(uint16_t i = 0; i < len; i++)
    {
        tx_buf[i + 1] = data[i];
    }
    
    // 拉低CS片选
    HAL_GPIO_WritePin(ICM_CS_GPIO_Port, ICM_CS_Pin, GPIO_PIN_RESET);
    
    // 发送数据
    HAL_SPI_Transmit(&hspi1, tx_buf, len + 1, HAL_MAX_DELAY);
    
    // 释放CS片选
    HAL_GPIO_WritePin(ICM_CS_GPIO_Port, ICM_CS_Pin, GPIO_PIN_SET);
    
    return HAL_OK;
}

static int32_t lsm6ds3_spi_read(void *handle, uint8_t reg, uint8_t *data, uint16_t len)
{
    uint8_t tx_buf[1];
    uint8_t rx_buf[len];
    
    // 拉低CS片选
    HAL_GPIO_WritePin(ICM_CS_GPIO_Port, ICM_CS_Pin, GPIO_PIN_RESET);
    
    // 发送寄存器地址
    tx_buf[0] = reg | 0x80; // 设置读命令，最高位为1
    
    // 发送数据
    HAL_SPI_Transmit(&hspi1, tx_buf, 1, HAL_MAX_DELAY);
    
    // 接收数据
    HAL_SPI_Receive(&hspi1, rx_buf, len, HAL_MAX_DELAY);
    
    // 释放CS片选
    HAL_GPIO_WritePin(ICM_CS_GPIO_Port, ICM_CS_Pin, GPIO_PIN_SET);
    
    // 复制接收到的数据
    for(uint16_t i = 0; i < len; i++)
    {
        data[i] = rx_buf[i];
    }
    
    return HAL_OK;
}

// LSM6DS3设备上下文
static stmdev_ctx_t lsm6ds3_ctx = {
    .write_reg = lsm6ds3_spi_write,
    .read_reg = lsm6ds3_spi_read,
    .handle = &hspi1,
    .mdelay = HAL_Delay
};

// 全局陀螺仪数据变量
static gyro_data current_gyro_data;

// 全局加速度计数据变量
static accel_data current_accel_data;

// 全局IMU数据变量
static imu_data current_imu_data;

void GyroInit()
{
    LOG_INFO("GyroInit start");
    
    // 配置LSM6DS3陀螺仪部分
    // 设置陀螺仪量程为±1000dps
    lsm6ds3_gy_full_scale_set(&lsm6ds3_ctx, LSM6DS3_1000dps);
    
    // 设置陀螺仪数据率为104Hz
    lsm6ds3_gy_data_rate_set(&lsm6ds3_ctx, LSM6DS3_GY_ODR_104Hz);
    
    // 启用陀螺仪轴
    lsm6ds3_gy_axis_x_data_set(&lsm6ds3_ctx, PROPERTY_ENABLE);
    lsm6ds3_gy_axis_y_data_set(&lsm6ds3_ctx, PROPERTY_ENABLE);
    lsm6ds3_gy_axis_z_data_set(&lsm6ds3_ctx, PROPERTY_ENABLE);
    
    LOG_INFO("GyroInit done");
}

void AccelInit()
{
    LOG_INFO("AccelInit start");
    
    // 配置LSM6DS3加速度计部分
    // 设置加速度计量程为±4g
    lsm6ds3_xl_full_scale_set(&lsm6ds3_ctx, LSM6DS3_4g);
    
    // 设置加速度计数据率为104Hz
    lsm6ds3_xl_data_rate_set(&lsm6ds3_ctx, LSM6DS3_XL_ODR_104Hz);
    
    // 启用加速度计轴
    lsm6ds3_xl_axis_x_data_set(&lsm6ds3_ctx, PROPERTY_ENABLE);
    lsm6ds3_xl_axis_y_data_set(&lsm6ds3_ctx, PROPERTY_ENABLE);
    lsm6ds3_xl_axis_z_data_set(&lsm6ds3_ctx, PROPERTY_ENABLE);
    
    LOG_INFO("AccelInit done");
}

void IMUInit()
{
    LOG_INFO("IMUInit start");
    
    // 软件复位设备
    uint8_t rst = 0x01;
    lsm6ds3_spi_write(&lsm6ds3_ctx, LSM6DS3_CTRL3_C, &rst, 1);
    HAL_Delay(10); // 等待复位完成
    
    // 设置块数据更新
    lsm6ds3_block_data_update_set(&lsm6ds3_ctx, PROPERTY_ENABLE);
    
    // 配置SPI模式为4线
    lsm6ds3_spi_mode_set(&lsm6ds3_ctx, LSM6DS3_SPI_4_WIRE);
    
    // 禁用I2C接口
    lsm6ds3_i2c_interface_set(&lsm6ds3_ctx, LSM6DS3_I2C_DISABLE);
    
    // 检查设备ID
    uint8_t device_id;
    lsm6ds3_device_id_get(&lsm6ds3_ctx, &device_id);
    
    if(device_id != LSM6DS3_ID)
    {
        LOG_ERROR("LSM6DS3 device ID mismatch: 0x%02X", device_id);
        return;
    }
    
    LOG_INFO("LSM6DS3 device ID verified: 0x%02X", device_id);
    
    // 初始化陀螺仪和加速度计
    GyroInit();
    AccelInit();
    
    LOG_INFO("IMUInit done");
}

void GyroHandler()
{
    static int16_t gyro_raw[3];
    
    // 检查数据是否就绪
    uint8_t data_ready;
    lsm6ds3_gy_flag_data_ready_get(&lsm6ds3_ctx, &data_ready);
    
    if(data_ready)
    {
        // 读取原始陀螺仪数据
        lsm6ds3_angular_rate_raw_get(&lsm6ds3_ctx, gyro_raw);
        
        // 数据就绪，可以进行转换
        // 将原始数据转换为dps
        // LSM6DS3_1000dps的转换系数是35.0 mdps/LSB
        // 这里需要根据实际量程调整转换系数
        current_gyro_data.yaw = (float)gyro_raw[0] * 35.0f / 32768.0f;
        current_gyro_data.pitch = (float)gyro_raw[1] * 35.0f / 32768.0f;
        current_gyro_data.roll = (float)gyro_raw[2] * 35.0f / 32768.0f;
    }
}

void AccelHandler()
{
    static int16_t accel_raw[3];
    
    // 检查数据是否就绪
    uint8_t data_ready;
    lsm6ds3_xl_flag_data_ready_get(&lsm6ds3_ctx, &data_ready);
    
    if(data_ready)
    {
        // 读取原始加速度计数据
        lsm6ds3_acceleration_raw_get(&lsm6ds3_ctx, accel_raw);
        
        // 数据就绪，可以进行转换
        // 将原始数据转换为g
        // LSM6DS3_4g的转换系数是0.122 mg/LSB
        // 这里需要根据实际量程调整转换系数
        current_accel_data.ax = (float)accel_raw[0] * 0.122f / 1000.0f;
        current_accel_data.ay = (float)accel_raw[1] * 0.122f / 1000.0f;
        current_accel_data.az = (float)accel_raw[2] * 0.122f / 1000.0f;
    }
}

void IMUHandler()
{
    // 处理陀螺仪数据
    GyroHandler();
    
    // 处理加速度计数据
    AccelHandler();
    
    // 更新完整IMU数据
    current_imu_data.gyro = current_gyro_data;
    current_imu_data.accel = current_accel_data;
}

gyro_data GyroGetGyroData()
{
    return current_gyro_data;
}

accel_data AccelGetAccelData()
{
    return current_accel_data;
}

imu_data IMUGetData()
{
    return current_imu_data;
}

void IMUPrintData()
{
    imu_data data = IMUGetData();
    
    LOG_INFO("IMU Data:");
    LOG_INFO("  Gyro (dps): Yaw=%.2f, Pitch=%.2f, Roll=%.2f",
             data.gyro.yaw, data.gyro.pitch, data.gyro.roll);
    LOG_INFO("  Accel (g): Ax=%.3f, Ay=%.3f, Az=%.3f",
             data.accel.ax, data.accel.ay, data.accel.az);
}