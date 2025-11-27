#include "pid.h"

static inline float clampf(float v, float mn, float mx) { return v < mn ? mn : (v > mx ? mx : v); }

void PID_Init(PIDController *pid, PIDMode mode, float kp, float ki, float kd, float Kf, float dt) {
  pid->kp = kp;
  pid->ki = ki;
  pid->kd = kd;
  pid->Kf = Kf;
  pid->dt = dt;
  pid->mode = mode;
  pid->integral = 0.0f;
  pid->prev_error = 0.0f;
  pid->prev_error2 = 0.0f;
  pid->output = 0.0f;
  pid->out_min = -1e9f;
  pid->out_max = 1e9f;
  pid->integral_min = -1e9f;
  pid->integral_max = 1e9f;
  pid->enableD = (kd != 0.0f) ? 1 : 0;
}

void PID_SetOutputLimit(PIDController *pid, float out_min, float out_max) {
  pid->out_min = out_min;
  pid->out_max = out_max;
}

void PID_SetIntegralLimit(PIDController *pid, float i_min, float i_max) {
  pid->integral_min = i_min;
  pid->integral_max = i_max;
}

void PID_Reset(PIDController *pid) {
  pid->integral = 0.0f;
  pid->prev_error = 0.0f;
  pid->prev_error2 = 0.0f;
  pid->output = 0.0f;
}

float PID_Update_Positional(PIDController *pid, float setpoint, float measurement) {
  float err = setpoint - measurement;
  pid->integral += err * pid->dt;
  pid->integral = clampf(pid->integral, pid->integral_min, pid->integral_max);
  float derivative = pid->enableD ? (err - pid->prev_error) / pid->dt : 0.0f;
  float out = pid->kp * err + pid->ki * pid->integral + pid->kd * derivative;
  out = clampf(out, pid->out_min, pid->out_max);
  pid->prev_error2 = pid->prev_error;
  pid->prev_error = err;
  pid->output = out;
  return clampf(out + pid->Kf * setpoint, pid->out_min, pid->out_max); // 加前馈并限幅总输出
}

float PID_Update_Incremental(PIDController *pid, float setpoint, float measurement) {
  float e = setpoint - measurement;
  float e1 = pid->prev_error;      // e(k-1)
  float e2 = pid->prev_error2;     // e(k-2)
  // 增量式 PID 差分离散：Δu = Kp*(e - e1) + Ki*e*dt + Kd*(e - 2e1 + e2)/dt
  float delta = pid->kp * (e - e1) + pid->ki * e * pid->dt;
  if (pid->enableD) {
    float d_term = pid->kd * (e - 2.0f * e1 + e2) / pid->dt;
    delta += d_term;
  }
  float out = pid->output + delta;
  out = clampf(out, pid->out_min, pid->out_max);
  pid->output = out;
  pid->prev_error2 = e1;
  pid->prev_error = e;
  return clampf(out + pid->Kf * setpoint, pid->out_min, pid->out_max);
}
