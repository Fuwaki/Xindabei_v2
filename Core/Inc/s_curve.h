#ifndef S_CURVE_H
#define S_CURVE_H

#include <stdint.h>

// S曲线控制器对象结构体
typedef struct {
    // --- 状态变量 (只读，不要手动修改) ---
    float current_speed;    // 当前速度 (m/s)
    float current_accel;    // 当前加速度 (m/s^2)
    float target_speed;     // 目标速度 (m/s)
    
    // --- 参数配置 (调试时可修改) ---
    float max_speed;        // 最大物理限速 (m/s)
    float min_speed;        // 最小速度 (m/s)
    
    float max_accel;        // 最大加速度 (m/s^2) -> 限制油门深度
    float max_decel;        // 最大减速度 (m/s^2) -> 限制刹车力度 (建议比max_accel大)
    
    float jerk_accel;       // 加速时的加加速度 (m/s^3) -> 决定起步柔和度 (越小越柔)
    float jerk_decel;       // 减速时的加加速度 (m/s^3) -> 决定刹车响应速度 (越大越快)
    
} SCurve;

/**
 * @brief 初始化S曲线控制器
 * @param handle 对象指针
 * @param max_v 最大速度
 * @param max_a 最大加速度
 * @param max_d 最大减速度 (建议设大点，保证入弯能刹住)
 * @param jerk_a 加速平滑度 (建议小点，防抬头)
 * @param jerk_d 减速平滑度 (建议大点，快速响应)
 */
void SCurve_Init(SCurve *handle, float max_v, float max_a, float max_d, float jerk_a, float jerk_d);

/**
 * @brief 设置目标速度 (通常在主循环或判断逻辑中调用)
 * @param handle 对象指针
 * @param target 期望达到的速度
 */
void SCurve_SetTarget(SCurve *handle, float target);

/**
 * @brief 核心计算函数 (必须在定时中断中调用，保证dt恒定)
 * @param handle 对象指针
 * @param dt_seconds 采样周期 (例如 10ms = 0.01f)
 * @return float 规划后的当前BaseSpeed
 */
float SCurve_Update(SCurve *handle, float dt_seconds);

/**
 * @brief 紧急复位 (比如冲出赛道保护触发时)
 */
void SCurve_Reset(SCurve *handle);

#endif // S_CURVE_H