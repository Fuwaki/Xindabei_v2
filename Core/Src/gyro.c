#include "gyro.h"
#include "icm42688.h"
void GyroInit()
{
    icm42688_config_t config = {.gyro_sample = ICM42688_GYRO_SAMPLE_SGN_1000DPS,
                                .acc_sample = ICM42688_ACC_SAMPLE_SGN_2G,
                                .sample_rate = ICM42688_SAMPLE_RATE_100,
                                .interface_type = ICM42688_INTERFACE_SPI};
    stm32_icm42688_hal_init(&config);
}
void GyroHandler()
{
    icm42688_get_gyro();
}
gyro_data GyroGetGyroData()
{
    gyro_data data = {.yaw = icm42688_gyro_transition(icm42688_gyro.x),
                      .pitch = icm42688_gyro_transition(icm42688_gyro.y),
                      .roll = icm42688_gyro_transition(icm42688_gyro.z)};
    return data;
}