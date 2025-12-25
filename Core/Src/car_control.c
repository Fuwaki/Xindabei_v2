#include "car_control.h"
// #include "gyro.h"
#include "imu.h"
#include "log.h"
#include "motor.h"
#include "pid.h"
#include "s_curve.h"
#include <math.h>
#include <stdint.h>

static float target_velocity = 0.0;
static float target_angular_velocity = 0.0;

static PIDController angular_velocity_pid;
static SCurve speed_scurve;
void CarControlInit()
{
    LOG_INFO("CarControlInit start");
    // 竞速车角速度环：高响应，低延迟
    // 移除D项（设为0），因为陀螺仪噪声经微分后会引起震荡
    // 适当提高P项以增强响应，I项用于消除稳态误差
    // 保留前馈 Kf=2.9 以提高动态响应速度
    PID_Init(&angular_velocity_pid, PID_MODE_POSITIONAL, 0.03, 0.0, 0.0, 0.9, 0.01); 
    
    // 禁用TD（微分跟踪器），减少计算延迟，直接响应
    // PID_EnableTD(&angular_velocity_pid, 0.2);
    
    PID_SetIntegralLimit(&angular_velocity_pid, -10.0, 10.0);    
    SCurve_Init(&speed_scurve, 70.0f, 10.f, 40.f, 10.f, 30.f);
    LOG_INFO("CarControlInit done");
}

void CarControlHandler()
{
    imu_data ImuData=IMUGetData();
    // // TODO: 增加前馈
    // // TODO: 限幅, target_angular_velocity
    float angular_velocity_output =
        PID_Update_Positional(&angular_velocity_pid, target_angular_velocity, -ImuData.gyro.yaw);
    // if (fabsf(angular_velocity_output) < 5.2)angular_velocity_output=0;         //平衡齿轮间隙带来的震荡
    // //正为顺时针
    //
    //
    target_velocity = SCurve_Update(&speed_scurve, 0.05f); // 10ms调用一次

    // float angular_velocity_output = target_angular_velocity; // 先用开环
    float left_motor_speed = target_velocity + angular_velocity_output;
    float right_motor_speed = target_velocity - angular_velocity_output;
    // //设置电机速度
    SetTargetMotorSpeed(left_motor_speed , right_motor_speed );
}
void SetTargetCarStatus(float velocity, float angular_velocity)
{
    // LOG_DEBUG("SetTargetCarStatus v=%.2f ang=%.2f", velocity, angular_velocity);
    SCurve_SetTarget(&speed_scurve, velocity);
    target_angular_velocity = angular_velocity;
}

CarState Car_GetState(void)
{
    const float CAR_TRACK_WIDTH = 0.155f; // 轮距 155mm
    const float MOTOR_SPEED_SCALE = 100.0f; // 与SetTargetMotorSpeed对应的比例因子
    CarState state;
    // 更新运动学状态 (正运动学)
    // v = (v_L + v_R) / 2
    // w = (v_R - v_L) / W  (假设右轮在前为正旋转方向，即CCW为正，需根据实际电机安装方向调整)
    float v_left = Motor_GetSpeed1() / MOTOR_SPEED_SCALE;
    float v_right = Motor_GetSpeed2() / MOTOR_SPEED_SCALE;

    state.linear_velocity = (v_left + v_right) / 2.0f;
    state.angular_velocity = (v_right - v_left) / CAR_TRACK_WIDTH;
    return state;
}

float Car_GetTargetVelocity(void)
{
    return target_velocity;
}
