#ifndef LADRC_H
#define LADRC_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @file    ladrc.h
 * @brief   1st Order Linear Active Disturbance Rejection Control
 * 
 * 一阶LADRC控制器，适用于速度环等一阶系统
 * 
 * 参数说明：
 *   wc (控制器带宽): 决定响应速度，越大响应越快，但噪声敏感
 *   wo (观测器带宽): 决定扰动估计速度，通常 wo = 3~5 * wc
 *   b0 (控制增益):   系统增益估计值，u -> y 的比例系数
 * 
 * 调参指南：
 *   1. 先设 b0：开环测试，施加单位输入，测量稳态输出变化率
 *   2. 再调 wc：从小到大，直到响应满足要求
 *   3. 最后调 wo：通常 wo = 3*wc ~ 5*wc，观测器要比控制器快
 */

typedef struct {
    /* 参数 */
    float wc;       /* 控制器带宽 (rad/s) */
    float wo;       /* 观测器带宽 (rad/s) */
    float b0;       /* 系统增益估计 */
    float dt;       /* 采样周期 (s) */
    float max_out;  /* 输出限幅 */
    
    /* 状态 */
    float z1;       /* ESO: 估计状态 (跟踪实际值) */
    float z2;       /* ESO: 估计扰动 (总扰动) */
    float last_u;   /* 上一次总输出 (含前馈) */
} LADRC_Handle_t;

/**
 * @brief 初始化 LADRC 控制器
 * @param handle   控制器句柄
 * @param wc       控制器带宽 (rad/s)，推荐 10~50
 * @param wo       观测器带宽 (rad/s)，推荐 3*wc ~ 5*wc
 * @param b0       系统增益估计
 * @param dt       采样周期 (s)
 * @param max_out  输出限幅
 */
void LADRC_Init(LADRC_Handle_t *handle, float wc, float wo, float b0, float dt, float max_out);

/**
 * @brief 重置控制器状态
 */
void LADRC_Reset(LADRC_Handle_t *handle);

/**
 * @brief 动态更新参数
 */
void LADRC_SetParams(LADRC_Handle_t *handle, float wc, float wo, float b0);

/**
 * @brief 控制器计算
 * @param handle   控制器句柄
 * @param expect   期望值 (目标)
 * @param real     实际值 (反馈)
 * @param ff       前馈量 (可设为0)
 * @return         控制输出
 */
float LADRC_Calc(LADRC_Handle_t *handle, float expect, float real, float ff);

#ifdef __cplusplus
}
#endif

#endif /* LADRC_H */
