#include "motor.h"
#include "adc.h"
#include "common.h"
#include "log.h"
#include "main.h"
#include "pid.h"
#include "stm32f4xx_hal_adc_ex.h"
#include "stm32f4xx_hal_gpio.h"
#include "tim.h"
#include <math.h>
#include <stdint.h>
#include <stdio.h>

// 定义是否仅使用速度环 (1: 仅速度环, 0: 速度环+电流环)
#define USE_SPEED_LOOP_ONLY 1

// 前向声明
void apply_signed_pwm_with_slow_decay(TIM_HandleTypeDef *htim, float value);
void apply_signed_pwm_with_fast_decay(TIM_HandleTypeDef *htim, float value);

// 使用差分法扩展编码器值：记录上一原始计数并根据差值调整扩展计数，避免依赖更新中断
static int32_t encoder1_extended = 0;
static uint16_t encoder1_last_raw = 0;
static uint8_t encoder1_inited = 0;

static int32_t encoder2_extended = 0;
static uint16_t encoder2_last_raw = 0;
static uint8_t encoder2_inited = 0;

// 速度pid
static PIDController motor1_speed_pid;
static PIDController motor2_speed_pid;

// 电流pid
static PIDController motor1_current_pid;
static PIDController motor2_current_pid;

// 测速pll
static EncPLL motor1_pll;
static EncPLL motor2_pll;

static float motor_1_speed_setpoint = 100.0;
static float motor_2_speed_setpoint = -300.0;

static int32_t motor_1_current = 0;
static int32_t motor_1_current_setpoint = 0;

static int32_t motor_2_current = 0;
static int32_t motor_2_current_setpoint = 0;

// 低通滤波器结构体 (Exponential Moving Average)
// 优点: 计算快，内存少，适合高频控制环路
// 缺点: 存在相位滞后
typedef struct
{
    float alpha;  // 滤波系数 [0, 1]，越小滤波越强
    float output; // 上一次的输出值
} LowPassFilter;

static LowPassFilter motor1_current_lpf;
static LowPassFilter motor2_current_lpf;

// 滤波器类型定义
#define FILTER_NONE 0
#define FILTER_LPF 1

// 选择使用的滤波器类型: FILTER_NONE (无滤波), FILTER_LPF (低通滤波)
#define FILTER_TYPE FILTER_NONE

void LPF_Init(LowPassFilter *lpf, float alpha)
{
    lpf->alpha = alpha;
    lpf->output = 0.0f;
}

float LPF_Update(LowPassFilter *lpf, float input)
{
    lpf->output = lpf->alpha * input + (1.0f - lpf->alpha) * lpf->output;
    return lpf->output;
}

// 获取滤波后的电流值
int32_t GetMotor1Current(void)
{
    return motor_1_current;
}

int32_t GetMotor2Current(void)
{
    return motor_2_current;
}

// 更新电流反馈值 (供中断回调调用)
void Motor_UpdateCurrentFeedback(uint32_t raw1, uint32_t raw2)
{
    int32_t current_feedback_1, current_feedback_2;

#if FILTER_TYPE == FILTER_LPF
    // 使用低通滤波 (LPF) - 推荐用于电流环控制
    // alpha = 0.1 表示新值权重0.1，历史权重0.9
    float filtered1 = LPF_Update(&motor1_current_lpf, (float)raw1);
    float filtered2 = LPF_Update(&motor2_current_lpf, (float)raw2);
    current_feedback_1 = (int32_t)filtered1;
    current_feedback_2 = (int32_t)filtered2;
#else
    // 不使用滤波 (FILTER_NONE)
    current_feedback_1 = (int32_t)raw1;
    current_feedback_2 = (int32_t)raw2;
#endif

    motor_1_current = current_feedback_1;
    motor_2_current = current_feedback_2;
}

// 电机模块初始化
void MotorInit()
{
    LOG_INFO("MotorInit start");

    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);

    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);

    // adc采样定时器
    HAL_TIM_Base_Start(&htim5);

    // 编码器初始化
    HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);

    // 初始化滤波器
    LPF_Init(&motor1_current_lpf, 0.1f); // LPF系数
    LPF_Init(&motor2_current_lpf, 0.1f);

    // 电流环adc采样 (开启中断)
    HAL_ADCEx_InjectedStart_IT(&hadc1);

    // EncPLL 调参入口：
    // f_min_hz: 稳态/低速时的观测带宽（越小越平滑）
    // f_max_hz: 动态/大误差时的观测带宽（越大越跟得快）
    // zeta    : 阻尼比（1.0~1.2 建议，无过冲且响应快）
    // error_min: 误差阈值下限 (counts)，小于此值认为噪声
    // error_max: 误差阈值上限 (counts)，大于此值认为运动
    // dt      : 采样周期（由速度环调用频率决定，这里是 0.01s 对应 100Hz）
    EncPLL_Init(&motor1_pll, 0.5f, 5.0f, 1.3f, 50.0f, 80.0f, 0.01f);
    EncPLL_Init(&motor2_pll, 0.5f, 5.0f, 1.3f, 1.0f, 3.0f, 0.01f);

    PID_Init(&motor1_speed_pid, PID_MODE_INCREMENTAL, 0.004f, 0.0001f, 0.0f, 0.0015f, 0.01f);
    PID_SetOutputLimit(&motor1_speed_pid, -1.f, 1.f);

    PID_Init(&motor2_speed_pid, PID_MODE_INCREMENTAL, 0.008f, 0.04f, 0.0002f, 0.001f, 0.01f);
    PID_SetOutputLimit(&motor1_speed_pid, -1.f, 1.f);

    PID_Init(&motor1_current_pid, PID_MODE_POSITIONAL, 0.0, 0.0, 0.0, 1.f, 0.01f);
    PID_SetOutputLimit(&motor1_speed_pid, -1.f, 1.0f);

    PID_Init(&motor2_current_pid, PID_MODE_POSITIONAL, 0.0, 0.0, 0.0, 1.f, 0.01f);
    PID_SetOutputLimit(&motor1_speed_pid, -1.f, 1.0f);
    LOG_INFO("MotorInit done");
}

void SetTargetMotorSpeed(float motor_1, float motor_2)
{
    motor_1_speed_setpoint = motor_1;
    motor_2_speed_setpoint = motor_2;
}
void SetTargetMotorCurrent(int32_t motor_1, int32_t motor_2)
{
    motor_1_current_setpoint = motor_1;
    motor_2_current_setpoint = motor_2;
}
// 和a4950的引脚定义对应
void apply_signed_pwm_with_fast_decay(TIM_HandleTypeDef *htim, float value)
{
    // 限幅到 [-1.0, 1.0]
    value = value > 1.0f ? 1.0f : (value < -1.0f ? -1.0f : value);

    if (value > 0)
    {
        __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_1, __HAL_TIM_GET_AUTORELOAD(htim));
        __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_2,
                              __HAL_TIM_GET_AUTORELOAD(htim) -
                                  (uint32_t)(fabsf(value) * __HAL_TIM_GET_AUTORELOAD(htim)));
    }
    else
    {
        __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_1,
                              __HAL_TIM_GET_AUTORELOAD(htim) -
                                  (uint32_t)(fabsf(value) * __HAL_TIM_GET_AUTORELOAD(htim)));
        __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_2, __HAL_TIM_GET_AUTORELOAD(htim));
    }
}
void apply_signed_pwm_with_slow_decay(TIM_HandleTypeDef *htim, float value)
{
    // 限幅到 [-1.0, 1.0]
    value = value > 1.0f ? 1.0f : (value < -1.0f ? -1.0f : value);
    if (value > 0)
    {
        __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_1, (uint32_t)(fabsf(value) * __HAL_TIM_GET_AUTORELOAD(htim)));
        __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_2, 0);
    }
    else
    {
        __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_1, 0);
        __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_2, (uint32_t)(fabsf(value) * __HAL_TIM_GET_AUTORELOAD(htim)));
    }
}

void CurrentLoopTimerHandler()
{
#if USE_SPEED_LOOP_ONLY
    // 仅使用速度环时，电流环不进行控制
    return;
#else
    // 运行电流环PID
    float output_1 =
        PID_Update_Positional(&motor1_current_pid, (float)motor_1_current_setpoint, (float)motor_1_current);
    float output_2 =
        PID_Update_Positional(&motor2_current_pid, (float)motor_2_current_setpoint, (float)motor_2_current);

    // 应用PWM输出
    apply_signed_pwm_with_fast_decay(&htim4, output_1);
    apply_signed_pwm_with_fast_decay(&htim2, output_2);
#endif
}

void SpeedLoopHandler()
{
    static uint32_t tick = 0;
    tick++;
    motor_1_speed_setpoint=motor_2_speed_setpoint = 300.0f * sinf((float)tick * 0.05f);

    float speed_1 = EncPLL_Update(&motor1_pll, (uint32_t)Get_Encoder1_Count()) / 10.0f;
    float speed_2 = EncPLL_Update(&motor2_pll, (uint32_t)Get_Encoder2_Count()) / 10.0f;
    float output_1 = PID_Update_Incremental(&motor1_speed_pid, motor_1_speed_setpoint, speed_1);
    float output_2 = PID_Update_Incremental(&motor2_speed_pid, motor_2_speed_setpoint, speed_2);

    // // 打印格式：目标速度, 实际速度1, 实际速度2, PID输出1, PID输出2, 编码器1, 观测器误差1, 观测器带宽1
    // printf(FLOAT_FMT "," FLOAT_FMT "," FLOAT_FMT "," FLOAT_FMT "," FLOAT_FMT ",%d\n",
    //        FLOAT_TO_INT(motor_2_speed_setpoint), FLOAT_TO_INT(speed_1), FLOAT_TO_INT(speed_2), FLOAT_TO_INT(output_1),
    //        FLOAT_TO_INT(output_2), (uint32_t)Get_Encoder2_Count());

#if USE_SPEED_LOOP_ONLY
    // 直接将速度环PID输出作为PWM占空比应用
    apply_signed_pwm_with_fast_decay(&htim4, output_1);
    apply_signed_pwm_with_fast_decay(&htim2, output_2);
#else
    SetTargetMotorCurrent((int32_t)output_1, (int32_t)output_2);
#endif
}

// 处理单次读取，利用差分纠正
static inline int32_t DiffUpdate(uint16_t raw, uint16_t *last_raw, uint8_t *inited, int32_t *extended)
{
    if (!*inited)
    {
        *last_raw = raw;
        *extended = 0;
        *inited = 1;
        return *extended;
    }

    // 优化：利用 int16_t 的强制转换自动处理 16 位溢出
    // 只要两次采样间的变化量在 -32768 到 32767 之间，这种写法就是数学上等价的
    // 且没有 if-else 分支，效率更高
    int16_t diff = (int16_t)(raw - *last_raw);

    *extended += diff;
    *last_raw = raw;
    return *extended;
}

int32_t Get_Encoder1_Count(void)
{
    int32_t val;
    __disable_irq();
    uint16_t raw = (uint16_t)(htim1.Instance->CNT);
    val = DiffUpdate(raw, &encoder1_last_raw, &encoder1_inited, &encoder1_extended);
    __enable_irq();
    return val;
}

int32_t Get_Encoder2_Count(void)
{
    int32_t val;
    __disable_irq();
    uint16_t raw = (uint16_t)(htim3.Instance->CNT);
    val = DiffUpdate(raw, &encoder2_last_raw, &encoder2_inited, &encoder2_extended);
    __enable_irq();
    return val;
}

void EncPLL_Init(EncPLL *pll, float f_min_hz, float f_max_hz, float zeta, float error_min, float error_max, float dt)
{
    pll->dt = dt;
    pll->theta = 0.0f;
    pll->omega = 0.0f;
    pll->offset = 0;

    // 存储物理含义更明确的参数，便于上层直接调参
    pll->f_min_hz = f_min_hz;
    pll->f_max_hz = f_max_hz;
    pll->zeta = zeta;
    pll->error_min = error_min;
    pll->error_max = error_max;

    // 预计算优化常数，避免在中断/高频循环中进行除法和重复计算
    pll->opt_omega_n_min = 2.0f * (float)M_PI * f_min_hz;
    pll->opt_omega_n_max = 2.0f * (float)M_PI * f_max_hz;

    if (error_max > error_min)
    {
        pll->opt_slope = (pll->opt_omega_n_max - pll->opt_omega_n_min) / (error_max - error_min);
    }
    else
    {
        pll->opt_slope = 0.0f;
    }

    pll->opt_two_zeta = 2.0f * zeta;
}
float EncPLL_Update(EncPLL *pll, int32_t now_theta)
{
    // 计算局部坐标系下的当前位置，避免大数吃小数
    // int32_t 减法自动处理溢出，只要 offset 紧跟 now_theta，结果就在 int32 范围内
    int32_t local_now_theta_i = now_theta - pll->offset;
    float local_now_theta = (float)local_now_theta_i;

    // 计算位置误差
    float error = local_now_theta - pll->theta;

    // --- 自适应带宽观测器 (Adaptive Bandwidth Observer) ---
    // 严谨性说明：
    // 这是一个基于误差大小动态调整系统带宽的观测器，类似于 One Euro Filter 的思想，
    // 但基于 Luenberger 观测器的极点配置理论。
    //
    // 核心策略：保持阻尼比(zeta)恒定，根据误差动态调整自然频率(omega_n)。
    // 1. 阻尼比 zeta = 1.2 (恒定)：保证系统始终处于过阻尼状态，无论带宽如何，都不会发生震荡和过冲。
    // 2. 自然频率 omega_n (动态)：
    //    - 误差小 (稳态) -> 低带宽 -> 强滤波，极度平滑。
    //    - 误差大 (动态) -> 高带宽 -> 强跟踪，无滞后。

    float error_abs = fabsf(error);
    float omega_n;

    // 设定误差阈值 (counts)
    // < error_min: 认为是纯噪声，使用最小带宽
    // > error_max: 认为是明确运动，使用最大带宽
    // 中间: 线性插值平滑过渡
    if (error_abs <= pll->error_min)
    {
        omega_n = pll->opt_omega_n_min;
    }
    else if (error_abs >= pll->error_max)
    {
        omega_n = pll->opt_omega_n_max;
    }
    else
    {
        // 线性插值优化：使用预计算的斜率，将除法转换为乘法
        // omega_n = omega_n_min + (error_abs - error_min) * slope
        omega_n = pll->opt_omega_n_min + (error_abs - pll->error_min) * pll->opt_slope;
    }

    // 保存调试信息
    pll->debug_error = error;
    pll->debug_omega_n = omega_n;

    // 根据极点配置公式实时计算增益
    // Ki = omega_n^2
    // Kp = 2 * zeta * omega_n
    float current_ki = omega_n * omega_n;
    float current_kp = pll->opt_two_zeta * omega_n;

    // 1. 速度 omega 仅由积分项驱动
    pll->omega += current_ki * error * pll->dt;

    // 2. 位置 theta 由 速度 omega 和 比例项 (kp * error) 共同驱动
    pll->theta += (pll->omega + current_kp * error) * pll->dt;

    // 定期重置坐标系，保持浮点数在小范围内 (例如 +/- 20000)
    // 这样可以保证 float 的精度足以分辨 1 个 count 的变化
    if (fabsf(pll->theta) > 20000.0f)
    {
        int32_t shift = (int32_t)pll->theta;
        pll->offset += shift;
        pll->theta -= (float)shift;
    }

    return pll->omega;
}