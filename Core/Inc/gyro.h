#pragma once
typedef struct
{
    float yaw;
    float pitch;
    float roll;
} gyro_data;
void GyroInit();
void GyroHandler();
gyro_data GyroGetGyroData();