#include "imu.h"
#include "lsm6ds3_reg.h"
#include "log.h"
#include "spi.h"
#include "param_server.h"
#include "madgwickFilter.h"
#include <string.h>

// SPI 通信缓冲区 (避免VLA，使用固定大小缓冲区)
#define IMU_SPI_BUF_SIZE 16
static uint8_t spi_tx_buf[IMU_SPI_BUF_SIZE];
static uint8_t spi_rx_buf[IMU_SPI_BUF_SIZE];

// LSM6DS3 SPI接口实现 - 优化版本
static int32_t lsm6ds3_spi_write(void *handle, uint8_t reg, const uint8_t *data, uint16_t len)
{
    if (len + 1 > IMU_SPI_BUF_SIZE) return HAL_ERROR;
    
    // 添加写命令位
    spi_tx_buf[0] = reg & 0x7F; // 清除最高位，确保是写操作
    memcpy(&spi_tx_buf[1], data, len);
    
    // 拉低CS片选
    HAL_GPIO_WritePin(ICM_CS_GPIO_Port, ICM_CS_Pin, GPIO_PIN_RESET);
    
    // 发送数据
    HAL_SPI_Transmit(&hspi1, spi_tx_buf, len + 1, HAL_MAX_DELAY);
    
    // 释放CS片选
    HAL_GPIO_WritePin(ICM_CS_GPIO_Port, ICM_CS_Pin, GPIO_PIN_SET);
    
    return HAL_OK;
}

static int32_t lsm6ds3_spi_read(void *handle, uint8_t reg, uint8_t *data, uint16_t len)
{
    if (len + 1 > IMU_SPI_BUF_SIZE) return HAL_ERROR;
    
    // 准备发送缓冲区：地址 + 填充字节用于全双工收发
    spi_tx_buf[0] = reg | 0x80; // 设置读命令，最高位为1
    memset(&spi_tx_buf[1], 0xFF, len); // 填充字节
    
    // 拉低CS片选
    HAL_GPIO_WritePin(ICM_CS_GPIO_Port, ICM_CS_Pin, GPIO_PIN_RESET);
    
    // 全双工收发 - 比分开Transmit和Receive更高效
    HAL_SPI_TransmitReceive(&hspi1, spi_tx_buf, spi_rx_buf, len + 1, HAL_MAX_DELAY);
    
    // 释放CS片选
    HAL_GPIO_WritePin(ICM_CS_GPIO_Port, ICM_CS_Pin, GPIO_PIN_SET);
    
    // 跳过第一个字节（发送地址时的接收）
    memcpy(data, &spi_rx_buf[1], len);
    
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

// 缓存的欧拉角（避免重复计算）
static float cached_roll = 0.0f;
static float cached_pitch = 0.0f;
static float cached_yaw = 0.0f;
static uint8_t euler_cache_valid = 0;

// 陀螺仪零漂 (手动标定值)
// 请在静止状态下观察输出，填入以下偏移量
static float gyro_bias[3] = {0.0f, 0.0f, 0.0f};

// IMU初始化状态标志
static uint8_t imu_initialized = 0;

// 竞速车需要高响应，使用二阶巴特沃斯滤波器
// 原一阶低通 (alpha=0.8) 等效截止频率约 73Hz
// 二阶巴特沃斯滤波器系数 (fc=85Hz @ fs=208Hz)，带宽略高于原设计
#define BUTTER_B0  0.664692258f
#define BUTTER_B1  1.329384515f
#define BUTTER_B2  0.664692258f
#define BUTTER_A1  1.213601825f
#define BUTTER_A2  0.445167205f

// 滤波器状态 (每个轴需要两个历史值)
typedef struct {
    float x_prev[2];  // 输入历史
    float y_prev[2];  // 输出历史
} BiquadState;

static BiquadState gyro_filter_state[3] = {0};

// 滤波器初始化标志
static uint8_t gyro_filter_initialized = 0;

// 二阶巴特沃斯滤波器实现
static inline float biquad_filter(BiquadState *state, float input)
{
    float output = BUTTER_B0 * input 
                 + BUTTER_B1 * state->x_prev[0] 
                 + BUTTER_B2 * state->x_prev[1]
                 - BUTTER_A1 * state->y_prev[0] 
                 - BUTTER_A2 * state->y_prev[1];
    
    // 更新历史值
    state->x_prev[1] = state->x_prev[0];
    state->x_prev[0] = input;
    state->y_prev[1] = state->y_prev[0];
    state->y_prev[0] = output;
    
    return output;
}

/* 参数服务器回调函数 */
static float GetGyroYaw(void) { return current_gyro_data.yaw; }
static float GetGyroPitch(void) { return current_gyro_data.pitch; }
static float GetGyroRoll(void) { return current_gyro_data.roll; }
static float GetAccelX(void) { return current_accel_data.ax; }
static float GetAccelY(void) { return current_accel_data.ay; }
static float GetAccelZ(void) { return current_accel_data.az; }

// 更新缓存的欧拉角
static inline void update_euler_cache(void)
{
    if (!euler_cache_valid)
    {
        eulerAngles(q_est, &cached_roll, &cached_pitch, &cached_yaw);
        euler_cache_valid = 1;
    }
}

// 姿态角获取函数 - 使用缓存避免重复计算
float IMU_GetRoll(void) { 
    update_euler_cache();
    return cached_roll; 
}
float IMU_GetPitch(void) { 
    update_euler_cache();
    return cached_pitch; 
}
float IMU_GetYaw(void) { 
    update_euler_cache();
    return cached_yaw; 
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
    
    // 重置滤波器初始化标志和状态
    gyro_filter_initialized = 0;
    memset(gyro_filter_state, 0, sizeof(gyro_filter_state));
    euler_cache_valid = 0;
    
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

// 内部辅助函数：直接读取并处理陀螺仪数据（跳过数据就绪检查）
static inline void GyroReadAndProcess(void)
{
    static int16_t gyro_raw[3];
    
    // 直接读取原始陀螺仪数据
    lsm6ds3_angular_rate_raw_get(&lsm6ds3_ctx, gyro_raw);

    // 使用库函数转换单位 (mdps -> dps)
    float gyro_x_dps = lsm6ds3_from_fs2000dps_to_mdps(gyro_raw[0]) * 0.001f;
    float gyro_y_dps = lsm6ds3_from_fs2000dps_to_mdps(gyro_raw[1]) * 0.001f;
    float gyro_z_dps = lsm6ds3_from_fs2000dps_to_mdps(gyro_raw[2]) * 0.001f;

    // 减去零漂并应用轴向映射
    float roll_raw = gyro_x_dps - gyro_bias[0];   // X轴 -> Roll
    float pitch_raw = gyro_y_dps - gyro_bias[1];  // Y轴 -> Pitch
    float yaw_raw = gyro_z_dps - gyro_bias[2];    // Z轴 -> Yaw

    // 二阶巴特沃斯滤波器实现
    if (!gyro_filter_initialized)
    {
        // 第一次运行时，用原始数据初始化滤波器历史
        for (int i = 0; i < 2; i++)
        {
            gyro_filter_state[0].x_prev[i] = roll_raw;
            gyro_filter_state[0].y_prev[i] = roll_raw;
            gyro_filter_state[1].x_prev[i] = pitch_raw;
            gyro_filter_state[1].y_prev[i] = pitch_raw;
            gyro_filter_state[2].x_prev[i] = yaw_raw;
            gyro_filter_state[2].y_prev[i] = yaw_raw;
        }
        current_gyro_data.roll = roll_raw;
        current_gyro_data.pitch = pitch_raw;
        current_gyro_data.yaw = yaw_raw;
        gyro_filter_initialized = 1;
    }
    else
    {
        // 应用二阶巴特沃斯滤波器
        current_gyro_data.roll = biquad_filter(&gyro_filter_state[0], roll_raw);
        current_gyro_data.pitch = biquad_filter(&gyro_filter_state[1], pitch_raw);
        current_gyro_data.yaw = biquad_filter(&gyro_filter_state[2], yaw_raw);
    }
}

// 内部辅助函数：直接读取并处理加速度计数据（跳过数据就绪检查）
static inline void AccelReadAndProcess(void)
{
    static int16_t accel_raw[3];
    
    // 直接读取原始加速度计数据
    lsm6ds3_acceleration_raw_get(&lsm6ds3_ctx, accel_raw);

    // 使用库函数转换单位 (mg -> g)
    current_accel_data.ax = lsm6ds3_from_fs4g_to_mg(accel_raw[0]) * 0.001f;
    current_accel_data.ay = lsm6ds3_from_fs4g_to_mg(accel_raw[1]) * 0.001f;
    current_accel_data.az = lsm6ds3_from_fs4g_to_mg(accel_raw[2]) * 0.001f;
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
        float gyro_x_dps = lsm6ds3_from_fs2000dps_to_mdps(gyro_raw[0]) * 0.001f;
        float gyro_y_dps = lsm6ds3_from_fs2000dps_to_mdps(gyro_raw[1]) * 0.001f;
        float gyro_z_dps = lsm6ds3_from_fs2000dps_to_mdps(gyro_raw[2]) * 0.001f;

        // 减去零漂并应用轴向映射
        float roll_raw = gyro_x_dps - gyro_bias[0];   // X轴 -> Roll
        float pitch_raw = gyro_y_dps - gyro_bias[1];  // Y轴 -> Pitch
        float yaw_raw = gyro_z_dps - gyro_bias[2];    // Z轴 -> Yaw

        // 二阶巴特沃斯滤波器实现
        if (!gyro_filter_initialized)
        {
            // 第一次运行时，用原始数据初始化滤波器历史
            for (int i = 0; i < 2; i++)
            {
                gyro_filter_state[0].x_prev[i] = roll_raw;
                gyro_filter_state[0].y_prev[i] = roll_raw;
                gyro_filter_state[1].x_prev[i] = pitch_raw;
                gyro_filter_state[1].y_prev[i] = pitch_raw;
                gyro_filter_state[2].x_prev[i] = yaw_raw;
                gyro_filter_state[2].y_prev[i] = yaw_raw;
            }
            current_gyro_data.roll = roll_raw;
            current_gyro_data.pitch = pitch_raw;
            current_gyro_data.yaw = yaw_raw;
            gyro_filter_initialized = 1;
        }
        else
        {
            // 应用二阶巴特沃斯滤波器
            current_gyro_data.roll = biquad_filter(&gyro_filter_state[0], roll_raw);
            current_gyro_data.pitch = biquad_filter(&gyro_filter_state[1], pitch_raw);
            current_gyro_data.yaw = biquad_filter(&gyro_filter_state[2], yaw_raw);
        }
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
        current_accel_data.ax = lsm6ds3_from_fs4g_to_mg(accel_raw[0]) * 0.001f;
        current_accel_data.ay = lsm6ds3_from_fs4g_to_mg(accel_raw[1]) * 0.001f;
        current_accel_data.az = lsm6ds3_from_fs4g_to_mg(accel_raw[2]) * 0.001f;
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
    
    // 由于是中断触发，数据肯定已就绪，直接读取（跳过状态检查，节省2次SPI通信）
    GyroReadAndProcess();
    AccelReadAndProcess();
    
    // 使缓存失效，因为姿态将要更新
    euler_cache_valid = 0;
    
    // 转换单位：dps -> rad/s (使用预计算常量)
    const float deg2rad = PI / 180.0f;
    float gx = current_gyro_data.roll * deg2rad;
    float gy = current_gyro_data.pitch * deg2rad;
    float gz = current_gyro_data.yaw * deg2rad;
    
    // 加速度计数据直接使用g值
    float ax = current_accel_data.ax;
    float ay = current_accel_data.ay;
    float az = current_accel_data.az;
    
    // 计算加速度向量模长用于动态检测
    float accel_magnitude = sqrtf(ax*ax + ay*ay + az*az);
    
    // 加速度计有效性检查：模长必须大于阈值
    // 无论动态大小，都调用融合算法
    // Madgwick 算法会自动归一化加速度，大动态时加速度计方向偏差会被 BETA 参数抑制
    if (accel_magnitude > 0.1f)
    {
        imu_filter(ax, ay, az, gx, gy, gz);
    }
    else
    {
        // 加速度计数据异常（可能传感器故障或极端失重），仅用陀螺仪积分
        imu_filter_gyro_only(gx, gy, gz);
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
