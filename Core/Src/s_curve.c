#include "s_curve.h"
#include <math.h>
#include "common.h"

void SCurve_Init(SCurve *handle, float max_v, float max_a, float max_d, float jerk_a, float jerk_d) {
    // 清空状态

    handle->current_speed = 0.0f;
    handle->current_accel = 0.0f;
    handle->target_speed  = 0.0f;    
    // 加载参数
    handle->max_speed = max_v;
    handle->min_speed = 0.1f; // 默认给个很小的起步速度，防止完全死区
    
    handle->max_accel = max_a;
    handle->max_decel = max_d;
    
    handle->jerk_accel = jerk_a;
    handle->jerk_decel = jerk_d;
}

void SCurve_SetTarget(SCurve *handle, float target) {
    // 输入限幅保护
    handle->target_speed = CLAMP(target, 0.0f, handle->max_speed);
}

void SCurve_Reset(SCurve *handle) {
    handle->current_speed = 0.0f;
    handle->current_accel = 0.0f;
    handle->target_speed = 0.0f;
}

float SCurve_Update(SCurve *handle, float dt) {
    // 1. 计算速度误差
    float v_err = handle->target_speed - handle->current_speed;
    
    // 如果误差极小，直接锁定（防止末端抖动）
    if (fabsf(v_err) < 0.05f) {
        handle->current_accel = 0.0f;
        handle->current_speed = handle->target_speed;
        return handle->current_speed;
    }

    // 2. 决策期望加速度 (P控制思想，系数越大追得越快，但也容易震荡)
    // 这里系数取 5.0 是经验值，可以理解为"追赶增益"
    float desired_accel = v_err * 5.0f;

    // 3. 对期望加速度进行限幅 (梯形规划层)
    // 注意：如果是减速过程(desired_accel < 0)，用 max_decel 限制
    if (desired_accel > 0) {
        desired_accel = CLAMP(desired_accel, 0.0f, handle->max_accel);
    } else {
        desired_accel = CLAMP(desired_accel, -handle->max_decel, 0.0f);
    }

    // 4. 核心：S曲线层 (限制加速度的变化率 - Jerk)
    float accel_diff = desired_accel - handle->current_accel;
    
    // 区分加速和减速阶段使用不同的 Jerk
    // 这是一个非对称 S 曲线的关键
    float current_jerk_limit;
    
    // 简单的状态机判断逻辑：
    // 如果想要更大的加速度 -> 正在加速 -> 用 jerk_accel
    // 如果想要更小的加速度 (更负) -> 正在刹车 -> 用 jerk_decel
    if (desired_accel > handle->current_accel) {
         // 正在试图加油门，或者正在试图减少刹车力度
         // 这里简化处理：只要是"正向变动"都视为加速柔和处理
         current_jerk_limit = handle->jerk_accel;
    } else {
         // 正在试图踩刹车
         current_jerk_limit = handle->jerk_decel;
    }

    // 计算这一帧加速度最大允许的变化量
    float max_accel_change = current_jerk_limit * dt;

    // 5. 更新加速度 (积分 Jerk)
    if (fabsf(accel_diff) > max_accel_change) {
        handle->current_accel += SIGN(accel_diff) * max_accel_change;
    } else {
        handle->current_accel = desired_accel; // 接近目标加速度时直接吸附
    }

    // 6. 更新速度 (积分加速度)
    handle->current_speed += handle->current_accel * dt;

    // 7. 最终安全限幅
    handle->current_speed = CLAMP(handle->current_speed, 0.0f, handle->max_speed);

    return handle->current_speed;
}