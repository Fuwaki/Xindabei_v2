/**
 * @file    ladrc.c
 * @brief   Implementation of 1st Order LADRC
 * 
 * 一阶线性自抗扰控制器实现
 */

#include "ladrc.h"

/**
 * @brief 初始化
 */
void LADRC_Init(LADRC_Handle_t *handle, float wc, float wo, float b0, float dt, float max_out) {
    if (handle == 0) return;

    handle->wc = wc;
    handle->wo = wo;
    handle->b0 = b0;
    handle->dt = dt;
    handle->max_out = max_out;

    LADRC_Reset(handle);
}

/**
 * @brief 重置状态
 */
void LADRC_Reset(LADRC_Handle_t *handle) {
    handle->z1 = 0.0f;
    handle->z2 = 0.0f;
    handle->last_u = 0.0f;
}

/**
 * @brief 动态更新参数
 */
void LADRC_SetParams(LADRC_Handle_t *handle, float wc, float wo, float b0) {
    handle->wc = wc;
    handle->wo = wo;
    handle->b0 = b0;
}

/**
 * @brief 核心计算 - 针对 FPU 优化
 * @note  使用 'float' 字面量 (如 2.0f) 确保使用 FPU 指令
 */
float LADRC_Calc(LADRC_Handle_t *handle, float expect, float real, float ff) {
    /* 1. 计算观测器增益 (Beta) */
    /* 注: 如果 wo 是静态的可以预计算，但 F4 实时计算够快 */
    float beta1 = 2.0f * handle->wo;
    float beta2 = handle->wo * handle->wo;

    /* 2. 更新 ESO (扩展状态观测器) */
    /* z1: 估计状态 (速度) */
    /* z2: 估计扰动 */
    /* 估计值与实际值的误差 */
    float error = handle->z1 - real;

    /* 离散迭代 (欧拉法) */
    /* 关键: 使用 'last_u'，它包含了前馈项！ */
    float dz1 = handle->z2 + (handle->b0 * handle->last_u) - (beta1 * error);
    float dz2 = -(beta2 * error);

    handle->z1 += dz1 * handle->dt;
    handle->z2 += dz2 * handle->dt;

    /* 3. 计算控制律 (LSEF) */
    /* 对估计状态的纯 P 控制 */
    float u0 = handle->wc * (expect - handle->z1);

    /* 4. 扰动补偿 */
    /* u_ladrc = (u0 - disturbance) / b0 */
    float u_ladrc = (u0 - handle->z2) / handle->b0;

    /* 5. 注入前馈 */
    /* 这是最终的执行器输出 */
    float final_out = u_ladrc + ff;

    /* 6. 输出限幅 & 状态存储 */
    if (final_out > handle->max_out) {
        final_out = handle->max_out;
    } else if (final_out < -handle->max_out) {
        final_out = -handle->max_out;
    }

    /* 存储总输出用于下次 ESO 更新 */
    handle->last_u = final_out;

    return final_out;
}
