#include "car_control.h"
// #include "gyro.h"
#include "imu.h"
#include "log.h"
#include "motor.h"
#include "param_server.h"
#include "pid.h"
#include "ladrc.h"
#include "s_curve.h"
#include <math.h>
#include <stdint.h>

// 定义角速度环控制器类型 (0: PID, 1: LADRC)
#define USE_LADRC_ANGULAR_LOOP 0

static float target_velocity = 0.0;
static float target_angular_velocity = 0.0;
static float prev_target_angular_velocity = 0.0;  // 用于计算角加速度前馈

static PIDController angular_velocity_pid;
static LADRC_Handle_t angular_velocity_ladrc;
static SCurve speed_scurve;

// 前馈参数
static const float Kf_omega = 0.12f;    // 角速度前馈 (原Kf)
static const float Kf_alpha = 0.005f;  // 角加速度前馈 (新增)

// 死区参数 (解决齿轮间隙震荡)
static const float DEADZONE_THRESHOLD = 3.0f;  // 输出死区阈值，需要根据实际调试

// 调试变量 (用于参数服务器输出)
static float debug_gyro_yaw = 0.0f;         // 陀螺仪角速度反馈
static float debug_feedforward = 0.0f;      // 总前馈输出
static float debug_feedback = 0.0f;         // PID反馈输出
static float debug_angular_output = 0.0f;   // 最终角速度环输出

// 参数服务器回调函数
static float GetTargetAngularVel(void) { return target_angular_velocity; }
static float GetGyroYaw(void) { return debug_gyro_yaw; }
static float GetFeedforward(void) { return debug_feedforward; }
static float GetFeedback(void) { return debug_feedback; }
static float GetAngularOutput(void) { return debug_angular_output; }

void CarControlInit()
{
    LOG_INFO("CarControlInit start");
    // 角速度环：提高反馈增益以加快响应
    // Kp 可以调大，死区会抑制小信号震荡
    PID_Init(&angular_velocity_pid, PID_MODE_POSITIONAL, 0.04f, 0.0f, 0.0f, 0.025f, 0.01f); 
    PID_SetIntegralLimit(&angular_velocity_pid, -50.0, 50.0);    

    // LADRC 初始化
    // b0 估算: 1/Kf_omega ≈ 1/0.07 ≈ 14.2
    // wc: 5.0 (约 0.8Hz) - 初始保守值
    // wo: 20.0
    // dt: 0.01s (100Hz)
    LADRC_Init(&angular_velocity_ladrc, 1.5f, 3.0f, 25.0f, 0.01f, 50.0f);

    SCurve_Init(&speed_scurve, 100.0f, 50.f, 50.f, 30.f, 50.f);
    
    // 注册角速度环调试参数到参数服务器
    static ParamDesc car_params[] = {
        {.name = "W_Tgt",      // 目标角速度
         .type = PARAM_TYPE_FLOAT,
         .ops.f.get = GetTargetAngularVel,
         .read_only = 1,
         .mask = PARAM_MASK_SERIAL},
        {.name = "W_Gyro",     // 陀螺仪反馈
         .type = PARAM_TYPE_FLOAT,
         .ops.f.get = GetGyroYaw,
         .read_only = 1,
         .mask = PARAM_MASK_SERIAL},
        {.name = "W_FF",       // 前馈输出
         .type = PARAM_TYPE_FLOAT,
         .ops.f.get = GetFeedforward,
         .read_only = 1,
         .mask = PARAM_MASK_SERIAL},
        {.name = "W_FB",       // 反馈输出
         .type = PARAM_TYPE_FLOAT,
         .ops.f.get = GetFeedback,
         .read_only = 1,
         .mask = PARAM_MASK_SERIAL},
        {.name = "W_Out",      // 最终输出
         .type = PARAM_TYPE_FLOAT,
         .ops.f.get = GetAngularOutput,
         .read_only = 1,
         .mask = PARAM_MASK_SERIAL},
    };
    for (int i = 0; i < sizeof(car_params) / sizeof(car_params[0]); i++)
    {
        ParamServer_Register(&car_params[i]);
    }
    
    LOG_INFO("CarControlInit done");
}

void CarControlHandler()
{
    imu_data ImuData = IMUGetData();
    
    // 保存陀螺仪反馈用于调试
    debug_gyro_yaw = -ImuData.gyro.yaw;

    // 角速度前馈
    float ff_omega = Kf_omega * target_angular_velocity;
    // 角加速度前馈
    float angular_accel = (target_angular_velocity - prev_target_angular_velocity) / 0.01f;
    float ff_alpha = Kf_alpha * angular_accel;
    prev_target_angular_velocity = target_angular_velocity;
    
    // 总前馈 = 角速度前馈 + 角加速度前馈
    float feedforward = ff_omega + ff_alpha;
    debug_feedforward = feedforward;
    
    // ========== 反馈计算 ==========
#if USE_LADRC_ANGULAR_LOOP
    // LADRC 计算
    // 传入前馈 feedforward，LADRC 会将其叠加在最终输出上
    // 注意: LADRC_Calc 返回的是包含前馈的总输出
    float total_output = LADRC_Calc(&angular_velocity_ladrc, target_angular_velocity, debug_gyro_yaw, feedforward);
    
    // 为了保持 debug_feedback 的语义 (仅反馈部分)
    debug_feedback = total_output - feedforward;
    
    float angular_velocity_output = total_output;
#else
    float feedback = PID_Update_Positional(&angular_velocity_pid, 
                                           target_angular_velocity, 
                                           debug_gyro_yaw);
    debug_feedback = feedback;
    
    // ========== 总输出 ==========
    float angular_velocity_output = feedback+feedforward ;
#endif
    
    // ========== 死区处理 (解决齿轮间隙震荡) ==========
    // 当目标角速度接近0且输出很小时，直接置零
    // 这样可以放心提高Kp而不用担心静止时震荡
    // if (fabsf(target_angular_velocity) < 1.0f && fabsf(angular_velocity_output) < DEADZONE_THRESHOLD)
    // {
    //     angular_velocity_output = 0.0f;
    // }
    debug_angular_output = angular_velocity_output;
    
    // S曲线速度规划
    target_velocity = SCurve_Update(&speed_scurve, 0.01f);

    // 差速计算
    float left_motor_speed = target_velocity + angular_velocity_output;
    float right_motor_speed = target_velocity - angular_velocity_output;
    
    SetTargetMotorSpeed(left_motor_speed, right_motor_speed);
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
