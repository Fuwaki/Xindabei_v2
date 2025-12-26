#pragma once
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
  PID_MODE_POSITIONAL = 0, // 位置式（直接计算输出）
  PID_MODE_INCREMENTAL     // 增量式（计算增量Δu）
} PIDMode;

// PID参数结构体，用于参数打包和在线调整
typedef struct {
  float kp;
  float ki;
  float kd;
  float Kf; // 前馈增益
} PIDParams;

typedef struct {
  float r;  // 速度因子
  float x1; // 跟踪信号
  float x2; // 微分信号
} SimpleTD;

typedef struct {
  SimpleTD core;
  uint8_t enable;
  float prev_derivative; // 用于增量式PID计算 D项增量
} PID_TD_Context;

typedef struct {
  // 参数
  float kp;
  float ki;
  float kd; // 可以为 0 禁用 D
  float Kf; // 前馈增益
  float dt;

  // 状态 (位置式)
  float integral;     // 积分累积
  float prev_error;   // 上一次误差
  float prev_error2;  // 上上次误差（用于增量式 D）
  float output;       // 最新输出（位置式保留）

  // 限幅
  float out_min;
  float out_max;
  float integral_min;
  float integral_max;

  // 模式
  PIDMode mode;
  uint8_t enableD; // 是否启用 D 项

  // TD 微分跟踪器
  PID_TD_Context td;
} PIDController;

void PID_Init(PIDController *pid, PIDMode mode, float kp, float ki, float kd, float Kf, float dt);
void PID_EnableTD(PIDController *pid, float r);
void PID_SetOutputLimit(PIDController *pid, float out_min, float out_max);
void PID_SetIntegralLimit(PIDController *pid, float i_min, float i_max);
void PID_Reset(PIDController *pid);

// 位置式更新：返回新的输出值
float PID_Update_Positional(PIDController *pid, float setpoint, float measurement);
// 增量式更新：返回输出值（内部累加）
float PID_Update_Incremental(PIDController *pid, float setpoint, float measurement);

// ============================================================================
// 在线参数调整接口
// ============================================================================

// 设置PID参数（在线调整）
void PID_SetParams(PIDController *pid, const PIDParams *params);

// 获取当前PID参数
PIDParams PID_GetParams(const PIDController *pid);

// 单独设置各参数
void PID_SetKp(PIDController *pid, float kp);
void PID_SetKi(PIDController *pid, float ki);
void PID_SetKd(PIDController *pid, float kd);
void PID_SetKf(PIDController *pid, float Kf);

// 使用参数结构体初始化PID
void PID_InitWithParams(PIDController *pid, PIDMode mode, const PIDParams *params, float dt);

// ============================================================================
// PID参数插值
// ============================================================================

// PID参数线性插值: a + t * (b - a), t ∈ [0, 1]
static inline PIDParams PIDParams_Lerp(const PIDParams *a, const PIDParams *b, float t) {
  PIDParams result;
  result.kp = a->kp + t * (b->kp - a->kp);
  result.ki = a->ki + t * (b->ki - a->ki);
  result.kd = a->kd + t * (b->kd - a->kd);
  result.Kf = a->Kf + t * (b->Kf - a->Kf);
  return result;
}

// PID参数限幅线性插值: t 自动限制在 [0, 1] 范围内
static inline PIDParams PIDParams_ClampedLerp(const PIDParams *a, const PIDParams *b, float t) {
  if (t < 0.0f) t = 0.0f;
  if (t > 1.0f) t = 1.0f;
  return PIDParams_Lerp(a, b, t);
}

// PID参数平滑插值 (Hermite曲线: 3t² - 2t³)
static inline PIDParams PIDParams_SmoothLerp(const PIDParams *a, const PIDParams *b, float t) {
  if (t < 0.0f) t = 0.0f;
  if (t > 1.0f) t = 1.0f;
  t = t * t * (3.0f - 2.0f * t);
  return PIDParams_Lerp(a, b, t);
}

#ifdef __cplusplus
}
#endif
