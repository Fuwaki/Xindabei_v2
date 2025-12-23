#pragma once
#include <stdint.h>
// 陀螺仪数据结构
typedef struct
{
    float yaw;
    float pitch;
    float roll;
} gyro_data;

// 加速度计数据结构
typedef struct
{
    float ax;  // X轴加速度 (g)
    float ay;  // Y轴加速度 (g)
    float az;  // Z轴加速度 (g)
} accel_data;

// 完整IMU数据结构
typedef struct
{
    gyro_data gyro;   // 陀螺仪数据
    accel_data accel; // 加速度计数据
} imu_data;

// 陀螺仪相关函数
void GyroInit();
uint8_t GyroHandler();
gyro_data GyroGetGyroData();

// 加速度计相关函数
void AccelInit();
uint8_t AccelHandler();
accel_data AccelGetAccelData();

// 完整IMU相关函数
void IMUInit();
void IMUHandler();
imu_data IMUGetData();
void IMUPrintData();

// 姿态角获取接口 (单位: 度)
float IMU_GetRoll();
float IMU_GetPitch();
float IMU_GetYaw();