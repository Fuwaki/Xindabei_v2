#pragma once
#include <stdint.h>
#include "tim.h"



// 读取扩展后的编码器计数值
int32_t Get_Encoder1_Count(void);
int32_t Get_Encoder2_Count(void);
// 前向声明 PIDController
#include "pid.h"
typedef struct {
  float kp, ki;   // 仅保留原始 PI 参数，便于兼容旧接口
  float dt;
  float theta;    // 估计相位（仍可选用）
  float omega;    // 估计速度/频率
  PIDController pid; // 内嵌增量式 PID 控制器用于 omega 更新
} EncPLL;
float EncPLL_Update(EncPLL *pll, int32_t now_theta);
void EncPLL_Init(EncPLL *pll, float kp, float ki, float dt);
void CurrentLoopInit();

// 设置目标电机电流
void SetTargetMotorSpeed(int32_t motor_1, int32_t motor_2);

// 电流环任务
void CurrentLoopTimerHandler();

// 速度环任务
void SpeedLoopHandler();

void MotorInit();
