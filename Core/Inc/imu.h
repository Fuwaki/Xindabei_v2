#pragma once

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
void GyroHandler();
gyro_data GyroGetGyroData();

// 加速度计相关函数
void AccelInit();
void AccelHandler();
accel_data AccelGetAccelData();

// 完整IMU相关函数
void IMUInit();
void IMUHandler();
imu_data IMUGetData();
void IMUPrintData();