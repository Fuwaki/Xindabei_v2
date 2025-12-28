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
    handle->min_speed = 0.1f; // 保留字段但暂不使用
    
    handle->max_accel = max_a;
    handle->max_decel = max_d;
    
    handle->jerk_accel = jerk_a;
    handle->jerk_decel = jerk_d;
}

void SCurve_SetTarget(SCurve *handle, float target) {
    // 输入限幅保护
    handle->target_speed = CLAMP(target, 0.0f, handle->max_speed);
}

float SCurve_GetAccel(SCurve *handle) {
    return handle->current_accel;
}

void SCurve_Reset(SCurve *handle) {
    handle->current_speed = 0.0f;
    handle->current_accel = 0.0f;
    handle->target_speed = 0.0f;
}

float SCurve_Update(SCurve *handle, float dt) {
    // 计算速度误差
    float v_err = handle->target_speed - handle->current_speed;
    
    // 如果误差极小，直接锁定（防止末端抖动）
    // 减小死区范围，提高精度
    if (fabsf(v_err) < 0.01f) {
        handle->current_accel = 0.0f;
        handle->current_speed = handle->target_speed;
        return handle->current_speed;
    }

    // 决策期望加速度 
    
    float safe_accel = sqrtf(2.0f * handle->jerk_decel * fabsf(v_err));
    
    // 施加方向
    float desired_accel = SIGN(v_err) * safe_accel;


    if (desired_accel > 0) {
        desired_accel = CLAMP(desired_accel, 0.0f, handle->max_accel);
    } else {
        desired_accel = CLAMP(desired_accel, -handle->max_decel, 0.0f);
    }

    // 限制加速度的变化率 
    float accel_diff = desired_accel - handle->current_accel;
    
    // 区分加速和减速阶段使用不同的 Jerk
    float current_jerk_limit;
    

    if (desired_accel > handle->current_accel) {
         if (handle->current_accel < 0) {

             current_jerk_limit = handle->jerk_decel; 
         } else {

             current_jerk_limit = handle->jerk_accel;
         }
    } else {

         current_jerk_limit = handle->jerk_decel;
    }

    // 计算这一帧加速度最大允许的变化量
    float max_accel_change = current_jerk_limit * dt;

    // 更新加速度
    if (fabsf(accel_diff) > max_accel_change) {
        handle->current_accel += SIGN(accel_diff) * max_accel_change;
    } else {
        handle->current_accel = desired_accel; // 接近目标加速度时直接吸附
    }
    
    // [Safety] 强制加速度限幅 (防止积分漂移导致超出物理极限)
    handle->current_accel = CLAMP(handle->current_accel, -handle->max_decel, handle->max_accel);

    // 更新速度 (积分加速度)
    handle->current_speed += handle->current_accel * dt;

    // 最终安全限
    // 如果速度达到边界，必须切断加速度，防止"顶着墙跑"导致内部状态错误
    if (handle->current_speed > handle->max_speed) {
        handle->current_speed = handle->max_speed;
        if (handle->current_accel > 0.0f) handle->current_accel = 0.0f;
    } else if (handle->current_speed < 0.0f) {
        handle->current_speed = 0.0f;
        if (handle->current_accel < 0.0f) handle->current_accel = 0.0f;
    }

    return handle->current_speed;
}