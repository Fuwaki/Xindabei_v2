#include "motor.h"
#include "adc.h"
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

static int32_t motor_1_speed_setpoint = 1000;
static int32_t motor_2_speed_setpoint = 0;

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

    EncPLL_Init(&motor1_pll, 8.0f, 2.0f, 0.01f);
    EncPLL_Init(&motor2_pll, 8.0f, 2.0f, 0.01f);

    PID_Init(&motor1_speed_pid, PID_MODE_INCREMENTAL, 0.000f, 0.0f, 0.0f, 0.0f, 0.01f);
    PID_SetOutputLimit(&motor1_speed_pid, -1.f, 1.f);

    PID_Init(&motor2_speed_pid, PID_MODE_INCREMENTAL, 0.1f, 0.0f, 0.0f, 0.0f, 0.01f);
    PID_SetOutputLimit(&motor1_speed_pid, -1.f, 1.f);

    PID_Init(&motor1_current_pid, PID_MODE_POSITIONAL, 0.0, 0.0, 0.0, 1.f, 0.01f);
    PID_SetOutputLimit(&motor1_speed_pid, -1.f, 1.0f);

    PID_Init(&motor2_current_pid, PID_MODE_POSITIONAL, 0.0, 0.0, 0.0, 1.f, 0.01f);
    PID_SetOutputLimit(&motor1_speed_pid, -1.f, 1.0f);
    LOG_INFO("MotorInit done");
}

void SetTargetMotorSpeed(int32_t motor_1, int32_t motor_2)
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
    float speed_1 = EncPLL_Update(&motor1_pll, (uint32_t)Get_Encoder1_Count());
    float speed_2 = EncPLL_Update(&motor2_pll, (uint32_t)Get_Encoder2_Count());
    float output_1 = PID_Update_Incremental(&motor1_speed_pid, (float)motor_1_speed_setpoint, speed_1);
    float output_2 = PID_Update_Incremental(&motor2_speed_pid, (float)motor_2_speed_setpoint, speed_2);

    printf("%f,%f,%f,%f,%d\n", speed_1, speed_2,output_1,output_2,(uint32_t)Get_Encoder1_Count());

#if USE_SPEED_LOOP_ONLY
    // 直接将速度环PID输出作为PWM占空比应用
    apply_signed_pwm_with_fast_decay(&htim4, 0.2);
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
    int32_t diff = (int32_t)raw - (int32_t)(*last_raw);
    // 处理正向溢出 (raw 小, last 大) 或反向溢出
    if (diff > 32767)
    {
        diff -= 65536; // 说明发生了反向 wrap（raw 比 last 小很多）
    }
    else if (diff < -32768)
    {
        diff += 65536; // 说明发生了正向 wrap（raw 比 last 大很多为负大值）
    }
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

void EncPLL_Init(EncPLL *pll, float kp, float ki, float dt)
{
    pll->kp = kp;
    pll->ki = ki;
    pll->dt = dt;
    pll->theta = 0.0f;
    pll->omega = 0.0f;
    pll->offset = 0;
    // 初始化内部 PID 为增量式，禁用 D
    PID_Init(&pll->pid, PID_MODE_INCREMENTAL, kp, ki, 0.0f, 0.0f, dt);
    // 限幅可选：这里设置一个较大的范围，用户可后续调整
    PID_SetOutputLimit(&pll->pid, -1e6f, 1e6f);
}
float EncPLL_Update(EncPLL *pll, int32_t now_theta)
{
    // 计算局部坐标系下的当前位置，避免大数吃小数
    // int32_t 减法自动处理溢出，只要 offset 紧跟 now_theta，结果就在 int32 范围内
    int32_t local_now_theta_i = now_theta - pll->offset;
    float local_now_theta = (float)local_now_theta_i;

    float theta_pred = pll->theta + pll->omega * pll->dt;
    // 使用增量式 PID：setpoint=local_now_theta, measurement=theta_pred
    float new_omega = PID_Update_Incremental(&pll->pid, local_now_theta, theta_pred);
    pll->omega = new_omega;  // omega 与 PID 输出绑定
    pll->theta = theta_pred; // 仅推进，不额外相位比例修正

    // 定期重置坐标系，保持浮点数在小范围内 (例如 +/- 20000)
    // 这样可以保证 float 的精度足以分辨 1 个 count 的变化
    if (fabsf(pll->theta) > 20000.0f)
    {
        int32_t shift = (int32_t)pll->theta;
        pll->offset += shift;
        pll->theta -= (float)shift;
        // 注意：PID 内部存储的是误差(error)，坐标系平移不改变误差值，因此无需重置 PID
    }

    return pll->omega;
}