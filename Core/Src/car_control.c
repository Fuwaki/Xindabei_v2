#include "car_control.h"
#include "gyro.h"
#include "motor.h"
#include "pid.h"
#include "log.h"
#include <stdint.h>

static float target_velocity = 0.0;
static float target_angular_velocity = 0.0;

static PIDController angular_velocity_pid;

void CarControlInit()
{
    LOG_INFO("CarControlInit start");
    PID_Init(&angular_velocity_pid, PID_MODE_POSITIONAL, 0.1, 0.1, 0.10, 0.1, 0.1);
    LOG_INFO("CarControlInit done");
}

void CarControlHandler()
{
    static int first = 1;
    if (first) { LOG_INFO("CarControlHandler loop entered"); first = 0; }
    gyro_data data = GyroGetGyroData();
    // TODO: 增加前馈
    // TODO: 限幅, target_angular_veloci
    // TODO: 方向矫正
    float angular_velocity_output = PID_Update_Positional(&angular_velocity_pid, target_angular_velocity, data.yaw);
    float left_motor_speed = target_velocity - angular_velocity_output;
    float right_motor_speed = target_velocity + angular_velocity_output;
    //设置电机速度
    SetTargetMotorSpeed((int32_t)(left_motor_speed * 100), (int32_t)(right_motor_speed * 100));
}
void SetTargetCarStatus(float velocity, float angular_velocity)
{
    LOG_DEBUG("SetTargetCarStatus v=%.2f ang=%.2f", velocity, angular_velocity);
    target_angular_velocity = angular_velocity;
    target_velocity = velocity;
}