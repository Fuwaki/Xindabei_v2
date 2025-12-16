#pragma once
void CarControlInit();
void CarControlHandler();
void SetTargetCarStatus(float velocity, float angular_velocity);

typedef struct {
    float linear_velocity;
    float angular_velocity;
} CarState;

// 获取小车当前状态 (实时计算)
CarState Car_GetState(void);

// 获取目标线速度
float Car_GetTargetVelocity(void);
