#include "imu.h"
#include "lsm6ds3_reg.h"
#include "log.h"
#include "spi.h"
#include "param_server.h"
#include "madgwickFilter.h"

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

// 陀螺仪零漂 (手动标定值)
// 请在静止状态下观察输出，填入以下偏移量
static float gyro_bias[3] = {0.0f, 0.0f, 0.0};

// IMU初始化状态标志
static uint8_t imu_initialized = 0;

// 陀螺仪滤波器系数 (0-1之间，越小滤波效果越强)
// 竞速车需要高响应，减弱滤波强度以减少相位滞后
#define GYRO_FILTER_ALPHA 0.8f

// 滤波后的陀螺仪数据
static gyro_data filtered_gyro_data;

// 滤波器初始化标志
static uint8_t gyro_filter_initialized = 0;

/* 参数服务器回调函数 */
static float GetGyroYaw(void) { return current_gyro_data.yaw; }
static float GetGyroPitch(void) { return current_gyro_data.pitch; }
static float GetGyroRoll(void) { return current_gyro_data.roll; }
static float GetAccelX(void) { return current_accel_data.ax; }
static float GetAccelY(void) { return current_accel_data.ay; }
static float GetAccelZ(void) { return current_accel_data.az; }

// 姿态角获取函数
float IMU_GetRoll(void) { 
    float roll, pitch, yaw;
    eulerAngles(q_est, &roll, &pitch, &yaw);
    return roll; 
}
float IMU_GetPitch(void) { 
    float roll, pitch, yaw;
    eulerAngles(q_est, &roll, &pitch, &yaw);
    return pitch; 
}
float IMU_GetYaw(void) { 
    float roll, pitch, yaw;
    eulerAngles(q_est, &roll, &pitch, &yaw);
    return yaw; 
}

void GyroInit()
{
    LOG_INFO("GyroInit start");
    
    // 配置LSM6DS3陀螺仪部分
    // 设置陀螺仪量程为±2000dps (更高精度)
    lsm6ds3_gy_full_scale_set(&lsm6ds3_ctx, LSM6DS3_2000dps);
    
    lsm6ds3_gy_data_rate_set(&lsm6ds3_ctx, LSM6DS3_GY_ODR_208Hz);
    
    // 设置陀螺仪为高性能模式
    lsm6ds3_gy_power_mode_set(&lsm6ds3_ctx, LSM6DS3_GY_HIGH_PERFORMANCE);
    
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
    // 设置加速度计量程为±4g (提高精度)
    lsm6ds3_xl_full_scale_set(&lsm6ds3_ctx, LSM6DS3_4g);
    
    // 提高采样率以匹配陀螺仪 (208Hz)
    lsm6ds3_xl_data_rate_set(&lsm6ds3_ctx, LSM6DS3_XL_ODR_208Hz);
    
    // 设置加速度计为高性能模式
    lsm6ds3_xl_power_mode_set(&lsm6ds3_ctx, LSM6DS3_XL_HIGH_PERFORMANCE);
    
    // 设置抗混叠滤波器带宽 (200Hz)
    lsm6ds3_xl_filter_analog_set(&lsm6ds3_ctx, LSM6DS3_ANTI_ALIASING_200Hz);

    // 启用加速度计轴
    lsm6ds3_xl_axis_x_data_set(&lsm6ds3_ctx, PROPERTY_ENABLE);
    lsm6ds3_xl_axis_y_data_set(&lsm6ds3_ctx, PROPERTY_ENABLE);
    lsm6ds3_xl_axis_z_data_set(&lsm6ds3_ctx, PROPERTY_ENABLE);
    
    LOG_INFO("AccelInit done");
}

void IMUInit()
{
    LOG_INFO("IMUInit start");
    
    // 重置滤波器初始化标志
    gyro_filter_initialized = 0;
    
    // 尝试读取设备ID（复位前）
    uint8_t device_id = 0;
    lsm6ds3_device_id_get(&lsm6ds3_ctx, &device_id);
    LOG_INFO("LSM6DS3 device ID before reset: 0x%02X", device_id);

    // 软件复位设备
    uint8_t rst = 0x01;
    lsm6ds3_spi_write(&lsm6ds3_ctx, LSM6DS3_CTRL3_C, &rst, 1);
    HAL_Delay(100); // 等待复位完成，增加延时到100ms
    
    // 设置块数据更新
    lsm6ds3_block_data_update_set(&lsm6ds3_ctx, PROPERTY_ENABLE);
    
    // 配置SPI模式为4线
    lsm6ds3_spi_mode_set(&lsm6ds3_ctx, LSM6DS3_SPI_4_WIRE);
    
    // 禁用I2C接口
    lsm6ds3_i2c_interface_set(&lsm6ds3_ctx, LSM6DS3_I2C_DISABLE);
    
    // 检查设备ID（复位后），尝试多次
    int retry = 5;
    while (retry--)
    {
        lsm6ds3_device_id_get(&lsm6ds3_ctx, &device_id);
        if (device_id == LSM6DS3_ID)
        {
            break;
        }
        HAL_Delay(10);
    }
    
    if(device_id != LSM6DS3_ID)
    {
        LOG_ERROR("LSM6DS3 device ID mismatch: 0x%02X (Expected: 0x%02X)", device_id, LSM6DS3_ID);
        imu_initialized = 0;
        return;
    }
    
    LOG_INFO("LSM6DS3 device ID verified: 0x%02X", device_id);
    
    // 配置INT1引脚用于数据就绪中断
    lsm6ds3_int1_route_t int1_route;
    // 启用陀螺仪和加速度计数据就绪中断到INT1引脚
    int1_route.int1_drdy_g = PROPERTY_ENABLE;  // 陀螺仪数据就绪中断
    int1_route.int1_drdy_xl = PROPERTY_ENABLE; // 加速度计数据就绪中断
    lsm6ds3_pin_int1_route_set(&lsm6ds3_ctx, &int1_route);
    
    // 设置中断引脚特性
    lsm6ds3_pin_mode_set(&lsm6ds3_ctx, LSM6DS3_PUSH_PULL);  // 推挽输出
    lsm6ds3_pin_polarity_set(&lsm6ds3_ctx, LSM6DS3_ACTIVE_HIGH); // 高电平有效
    lsm6ds3_int_notification_set(&lsm6ds3_ctx, LSM6DS3_INT_PULSED); // 脉冲中断
    
    // 初始化陀螺仪和加速度计
    GyroInit();
    AccelInit();
    
    // 标记IMU初始化成功
    imu_initialized = 1;
    
    /* 注册IMU参数到服务器：陀螺仪和加速度计数据都通过串口输出 */
    static ParamDesc imu_params[] = {
        // 陀螺仪参数
        { .name = "GYRO_YAW", .type = PARAM_TYPE_FLOAT, .ops.f.get = GetGyroYaw, .read_only = 1, .mask = PARAM_MASK_SERIAL|PARAM_MASK_OLED },
        { .name = "GYRO_PITCH", .type = PARAM_TYPE_FLOAT, .ops.f.get = GetGyroPitch, .read_only = 1, .mask = PARAM_MASK_SERIAL |PARAM_MASK_OLED},
        { .name = "GYRO_ROLL", .type = PARAM_TYPE_FLOAT, .ops.f.get = GetGyroRoll, .read_only = 1, .mask = PARAM_MASK_SERIAL|PARAM_MASK_OLED },
        
        // 加速度计参数
        { .name = "ACCEL_X", .type = PARAM_TYPE_FLOAT, .ops.f.get = GetAccelX, .read_only = 1, .mask = PARAM_MASK_SERIAL|PARAM_MASK_OLED },
        { .name = "ACCEL_Y", .type = PARAM_TYPE_FLOAT, .ops.f.get = GetAccelY, .read_only = 1, .mask = PARAM_MASK_SERIAL|PARAM_MASK_OLED },
        { .name = "ACCEL_Z", .type = PARAM_TYPE_FLOAT, .ops.f.get = GetAccelZ, .read_only = 1, .mask = PARAM_MASK_SERIAL|PARAM_MASK_OLED },

        // 姿态角参数
        { .name = "ATT_ROLL", .type = PARAM_TYPE_FLOAT, .ops.f.get = IMU_GetRoll, .read_only = 1, .mask = PARAM_MASK_SERIAL|PARAM_MASK_OLED },
        { .name = "ATT_PITCH", .type = PARAM_TYPE_FLOAT, .ops.f.get = IMU_GetPitch, .read_only = 1, .mask = PARAM_MASK_SERIAL|PARAM_MASK_OLED },
        { .name = "ATT_YAW", .type = PARAM_TYPE_FLOAT, .ops.f.get = IMU_GetYaw, .read_only = 1, .mask = PARAM_MASK_SERIAL|PARAM_MASK_OLED },
    };
    // for (int i = 0; i < sizeof(imu_params)/sizeof(imu_params[0]); i++) {
    //     ParamServer_Register(&imu_params[i]);
    // }
    
    LOG_INFO("IMUInit done");
}

uint8_t GyroHandler()
{
    static int16_t gyro_raw[3];
    uint8_t updated = 0;
    
    // 检查数据是否就绪
    uint8_t data_ready;
    lsm6ds3_gy_flag_data_ready_get(&lsm6ds3_ctx, &data_ready);
    
    if(data_ready)
    {
        updated = 1;
        // 读取原始陀螺仪数据
        lsm6ds3_angular_rate_raw_get(&lsm6ds3_ctx, gyro_raw);

        // 使用库函数转换单位 (mdps -> dps)
        float gyro_x_dps = lsm6ds3_from_fs2000dps_to_mdps(gyro_raw[0]) / 1000.0f;
        float gyro_y_dps = lsm6ds3_from_fs2000dps_to_mdps(gyro_raw[1]) / 1000.0f;
        float gyro_z_dps = lsm6ds3_from_fs2000dps_to_mdps(gyro_raw[2]) / 1000.0f;

        // 减去零漂 (已转换为dps)
        // 修正映射关系：X->Roll, Y->Pitch, Z->Yaw
        // 之前代码：Yaw=X, Pitch=Y, Roll=Z (导致错配)
        current_gyro_data.roll = gyro_x_dps - gyro_bias[0];  // X轴围绕X轴旋转 -> Roll
        current_gyro_data.pitch = gyro_y_dps - gyro_bias[1]; // Y轴围绕Y轴旋转 -> Pitch
        current_gyro_data.yaw = gyro_z_dps - gyro_bias[2];   // Z轴围绕Z轴旋转 -> Yaw

        // 一阶低通滤波器实现
        if (!gyro_filter_initialized)
        {
            // 第一次运行时，用原始数据初始化滤波器输出
            filtered_gyro_data = current_gyro_data;
            gyro_filter_initialized = 1;
        }
        else
        {
            // 一阶低通滤波器公式: filtered = alpha * new + (1 - alpha) * filtered
            filtered_gyro_data.roll = GYRO_FILTER_ALPHA * current_gyro_data.roll +
                                    (1.0f - GYRO_FILTER_ALPHA) * filtered_gyro_data.roll;
            filtered_gyro_data.pitch = GYRO_FILTER_ALPHA * current_gyro_data.pitch +
                                     (1.0f - GYRO_FILTER_ALPHA) * filtered_gyro_data.pitch;
            filtered_gyro_data.yaw = GYRO_FILTER_ALPHA * current_gyro_data.yaw +
                                   (1.0f - GYRO_FILTER_ALPHA) * filtered_gyro_data.yaw;
        }

        // 更新当前陀螺仪数据为滤波后的数据
        current_gyro_data = filtered_gyro_data;

        // printf("%d,%d,%d\n",gyro_raw[0],gyro_raw[1],gyro_raw[2]);
    }
    return updated;
}

uint8_t AccelHandler()
{
    static int16_t accel_raw[3];
    uint8_t updated = 0;
    
    // 检查数据是否就绪
    uint8_t data_ready;
    lsm6ds3_xl_flag_data_ready_get(&lsm6ds3_ctx, &data_ready);
    
    if(data_ready)
    {
        updated = 1;
        // 读取原始加速度计数据
        lsm6ds3_acceleration_raw_get(&lsm6ds3_ctx, accel_raw);

        // 使用库函数转换单位 (mg -> g)
        // 注意：根据量程不同，转换函数可能需要调整，这里假设库函数会自动处理或我们手动指定
        // 如果使用 LSM6DS3_4g，LSB 约为 0.122 mg
        current_accel_data.ax = lsm6ds3_from_fs4g_to_mg(accel_raw[0]) / 1000.0f;
        current_accel_data.ay = lsm6ds3_from_fs4g_to_mg(accel_raw[1]) / 1000.0f;
        current_accel_data.az = lsm6ds3_from_fs4g_to_mg(accel_raw[2]) / 1000.0f;
    }
    return updated;
}

void IMUHandler()
{
    // 检查IMU是否已初始化，如果未初始化则尝试初始化
    if (!imu_initialized)
    {
        LOG_INFO("IMU not initialized, attempting to initialize...");
        IMUInit();
        return; // 本次不处理数据，等待下次调用
    }
    
    // 处理陀螺仪数据
    uint8_t gyro_updated = GyroHandler();
    
    // 处理加速度计数据
    AccelHandler();
    
    // 如果陀螺仪数据更新了，则更新姿态解算
    // Madgwick算法依赖于陀螺仪积分，必须在每次陀螺仪更新时调用
    if (gyro_updated)
    {
        // 转换单位：dps -> rad/s
        // 映射关系已修正：Roll->X, Pitch->Y, Yaw->Z
        float gx = current_gyro_data.roll * (PI / 180.0f);
        float gy = current_gyro_data.pitch * (PI / 180.0f);
        float gz = current_gyro_data.yaw * (PI / 180.0f);
        
        // 加速度计数据直接使用g值
        // 确保加速度计轴向与陀螺仪一致：Ax->X, Ay->Y, Az->Z
        float ax = current_accel_data.ax;
        float ay = current_accel_data.ay;
        float az = current_accel_data.az;
        
        // 简单的防除零保护，如果加速度计数据极小（可能未初始化或故障），则不进行更新
        if (fabsf(ax) > 0.001f || fabsf(ay) > 0.001f || fabsf(az) > 0.001f)
        {
            imu_filter(ax, ay, az, gx, gy, gz);
        }
    }
    
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
    LOG_INFO("  Gyro (dps): Roll(X)=%.2f, Pitch(Y)=%.2f, Yaw(Z)=%.2f",
             data.gyro.roll, data.gyro.pitch, data.gyro.yaw);
    LOG_INFO("  Accel (g): Ax=%.3f, Ay=%.3f, Az=%.3f",
             data.accel.ax, data.accel.ay, data.accel.az);
    LOG_INFO("  Attitude (deg): Roll=%.2f, Pitch=%.2f, Yaw=%.2f",
             IMU_GetRoll(), IMU_GetPitch(), IMU_GetYaw());
}