#pragma once
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
  PID_MODE_POSITIONAL = 0, // 位置式（直接计算输出）
  PID_MODE_INCREMENTAL     // 增量式（计算增量Δu）
} PIDMode;

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
} PIDController;

void PID_Init(PIDController *pid, PIDMode mode, float kp, float ki, float kd, float Kf, float dt);
void PID_SetOutputLimit(PIDController *pid, float out_min, float out_max);
void PID_SetIntegralLimit(PIDController *pid, float i_min, float i_max);
void PID_Reset(PIDController *pid);

// 位置式更新：返回新的输出值
float PID_Update_Positional(PIDController *pid, float setpoint, float measurement);
// 增量式更新：返回输出值（内部累加）
float PID_Update_Incremental(PIDController *pid, float setpoint, float measurement);

#ifdef __cplusplus
}
#endif
