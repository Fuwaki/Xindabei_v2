#pragma once
#include <stdint.h>
#include "tim.h"



// 读取扩展后的编码器计数值
int32_t Get_Encoder1_Count(void);
int32_t Get_Encoder2_Count(void);
typedef struct {
  float dt;               // 采样周期
  float theta;            // 估计位置
  float omega;            // 估计速度
  int32_t offset;         // 坐标系偏移量

  // 自适应带宽参数（单位均为物理意义）：
  // f_min, f_max: 观测器带宽范围 (Hz)
  // zeta: 阻尼比（一般取 1.0~1.2）
  float f_min_hz;
  float f_max_hz;
  float zeta;

  // 自适应阈值 (counts)
  // error_min: 误差小于此值时使用 f_min_hz (认为是噪声)
  // error_max: 误差大于此值时使用 f_max_hz (认为是运动)
  float error_min;
  float error_max;
  
  // 调试变量
  float debug_error;      // 观测器位置误差 (counts)
  float debug_omega_n;    // 当前自适应带宽 (rad/s)

  // 运行时优化常数 (预计算以减少运行时开销)
  float opt_omega_n_min;
  float opt_omega_n_max;
  float opt_slope;        // 用于线性插值的斜率
  float opt_two_zeta;     // 2 * zeta
} EncPLL;

float EncPLL_Update(EncPLL *pll, int32_t now_theta);
// 观测器初始化：f_min_hz/f_max_hz/zeta/error_min/error_max 全部在这里调参
void EncPLL_Init(EncPLL *pll, float f_min_hz, float f_max_hz, float zeta, float error_min, float error_max, float dt);

// 设置目标电机电流
void SetTargetMotorSpeed(float motor_1, float motor_2);

// 电流环任务
void CurrentLoopTimerHandler();

// 速度环任务
void SpeedLoopHandler();

void MotorInit();

// 更新电流反馈值 (供中断回调调用)
void Motor_UpdateCurrentFeedback(uint32_t raw1, uint32_t raw2);

// 获取滤波后的电流值
int32_t GetMotor1Current(void);
int32_t GetMotor2Current(void);

// 获取电机速度 (单位: 与SetTargetMotorSpeed对应)
float Motor_GetSpeed1(void);
float Motor_GetSpeed2(void);
