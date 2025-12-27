#include "car_control.h"
// #include "gyro.h"
#include "imu.h"
#include "log.h"
#include "motor.h"
#include "param_server.h"
#include "pid.h"
#include "s_curve.h"
#include <math.h>
#include <stdint.h>

static float target_velocity = 0.0;
static float target_angular_velocity = 0.0;
static float prev_target_angular_velocity = 0.0;  // 用于计算角加速度前馈

static PIDController angular_velocity_pid;
static SCurve speed_scurve;

// 前馈参数 (已移至 CarControlHandler 内部进行插值)
// static const float Kf_omega = 0.145f;    // 角速度前馈 
// static const float Kf_alpha = 0.004f;  // 角加速度前馈 


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
    PID_Init(&angular_velocity_pid, PID_MODE_POSITIONAL, 0.04f, 0.0f, 0.0f, 0.0f, 0.01f);
    // PID_EnableTD(&an, float r)
    PID_SetIntegralLimit(&angular_velocity_pid, -50.0, 50.0);    

    SCurve_Init(&speed_scurve, 200.0f, 80.0f, 180.0f, 400.0f, 1800.0f);
    
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
        // {.name = "W_FF",       // 前馈输出
        //  .type = PARAM_TYPE_FLOAT,
        //  .ops.f.get = GetFeedforward,
        //  .read_only = 1,
        //  .mask = PARAM_MASK_SERIAL},
        // {.name = "W_FB",       // 反馈输出
        //  .type = PARAM_TYPE_FLOAT,
        //  .ops.f.get = GetFeedback,
        //  .read_only = 1,
        //  .mask = PARAM_MASK_SERIAL},
        // {.name = "W_Out",      // 最终输出
        //  .type = PARAM_TYPE_FLOAT,
        //  .ops.f.get = GetAngularOutput,
        //  .read_only = 1,
        //  .mask = PARAM_MASK_SERIAL},
    };
    // for (int i = 0; i < sizeof(car_params) / sizeof(car_params[0]); i++)
    // {
    //     ParamServer_Register(&car_params[i]);
    // }
    
    LOG_INFO("CarControlInit done");
}

void CarControlHandler()
{
    imu_data ImuData = IMUGetData();
    
        // 保存陀螺仪反馈用于调试
    debug_gyro_yaw = -ImuData.gyro.yaw;

    // ========== 误差计算与插值系数 ==========
    float error_abs = fabsf(target_angular_velocity - debug_gyro_yaw);
    const float ERR_MIN = 5.0f;   // 误差 < 5.0 使用 params_low
    const float ERR_MAX = 50.0f;  // 误差 > 20.0 使用 params_high
    // 计算插值系数 t = (error - min) / (max - min)
    float t = (error_abs - ERR_MIN) / (ERR_MAX - ERR_MIN);
    // 限制 t 在 [0, 1]
    if (t < 0.0f) t = 0.0f;
    if (t > 1.0f) t = 1.0f;

    // ========== 前馈参数插值 ==========
    // 定义前馈参数范围
    const float KF_OMEGA_LOW = 0.145f;
    const float KF_OMEGA_HIGH = 0.155f; 
    const float KF_ALPHA_LOW = 0.000f;
    const float KF_ALPHA_HIGH = 0.00f;

    // 线性插值
    float kf_omega = KF_OMEGA_LOW + t * (KF_OMEGA_HIGH - KF_OMEGA_LOW);
    float kf_alpha = KF_ALPHA_LOW + t * (KF_ALPHA_HIGH - KF_ALPHA_LOW);

    // 角速度前馈
    float ff_omega = kf_omega * target_angular_velocity;
    // 角加速度前馈
    float angular_accel = (target_angular_velocity - prev_target_angular_velocity) / 0.01f;
    float ff_alpha = kf_alpha * angular_accel;
    prev_target_angular_velocity = target_angular_velocity;
    
    // 总前馈 = 角速度前馈 + 角加速度前馈
    float feedforward = ff_omega + ff_alpha;
    debug_feedforward = feedforward;
    
    // ========== 分段PID参数调整 (插值) ==========
    // 定义参数: 低误差用原参数，高误差适当增加Kp
    PIDParams params_low = { .kp = 0.03f, .ki = 0.0f, .kd = 0.0f, .Kf = 0.0f };
    PIDParams params_high = { .kp = 0.06f, .ki = 0.0f, .kd = 0.0f, .Kf = 0.0f };
    
    // 使用现成的插值函数 (ClampedLerp 会自动限制 t 在 [0,1])
    PIDParams curr_params = PIDParams_ClampedLerp(&params_low, &params_high, t);
    PID_SetParams(&angular_velocity_pid, &curr_params);

    // ========== 反馈计算 ==========
    float feedback = PID_Update_Positional(&angular_velocity_pid, 
                                           target_angular_velocity, 
                                           debug_gyro_yaw);
    debug_feedback = feedback;
    
    // ========== 总输出 ==========
    float angular_velocity_output = feedback + feedforward;
    
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
    // angular_velocity_output=0;
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
