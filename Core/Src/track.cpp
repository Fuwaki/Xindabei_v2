#include "track.h"
#include "common.h"
#include "fsm.hpp"
#include "led.h"
#include <cmath>
#include <stdint.h>

extern "C"
{
#include "car_control.h"
#include "imu.h"
#include "led.h"
#include "log.h"
#include "main.h" // Add this
#include "meg_adc.h"
#include "oled_service.h"
#include "param_server.h"
#include "pid.h"
#include "tof.h"
}

// ============================================================================
// 业务配置
// ============================================================================
namespace Config
{
// 保守模式参数 (提速前)
struct ConservativeParams {
    static constexpr float VEL_TRACKING = 80.0f;
    static constexpr PIDParams TRACKING_PID = {.kp = 190.0f, .ki = 0.0f, .kd = 0.13f, .Kf = 0.0f};
    static constexpr float SPEED_ATTENUATION_A = 0.0f;  // 不使用速度衰减
    static constexpr float PRE_RING_OFFSET = -160.0f;
    static constexpr float RING_VEL = 80.0f;
    static constexpr float RING_OFFSET = 0.0f;
    static constexpr uint32_t RING_EXIT_TIME = 60;
    static constexpr PIDParams RING_PID = {.kp = 190.0f, .ki = 0.0f, .kd = 0.13f, .Kf = 0.0f};
    // 避障参数
    static constexpr float OA_TURN_ANGLE = 67.0f;       // 避障转出角度
    static constexpr uint32_t OA_TURN_OUT_TIME = 400;   // 避障转出时间 (ms)
    static constexpr uint32_t OA_PARALLEL_TIME = 550;   // 避障平行时间 (ms)
    static constexpr float OA_RETURN_ANGLE = 40.0f;     // 避障返回角度
    static constexpr uint32_t OA_TIMEOUT = 1300;        // 避障超时时间 (ms)
    static constexpr uint32_t STOP_AFTER_OBSTACLE_DELAY = 800; // 第二次避障后停车延迟 (ms)
};

// 激进模式参数 (当前)
struct AggressiveParams {
    static constexpr float VEL_TRACKING = 108.0f;
    static constexpr PIDParams TRACKING_PID = {.kp = 230.0f, .ki = 0.0f, .kd = 0.15f, .Kf = 0.0f};
    static constexpr float SPEED_ATTENUATION_A = 0.08f;  // 速度衰减系数
    static constexpr float PRE_RING_OFFSET = -220.0f;
    static constexpr float RING_VEL = 95.0f;
    static constexpr float RING_OFFSET = -100.0f;
    static constexpr uint32_t RING_EXIT_TIME = 60;
    static constexpr PIDParams RING_PID = {.kp = 200.0f, .ki = 0.0f, .kd = 0.13f, .Kf = 0.0f};
    static constexpr float OA_TURN_ANGLE = 70.0f;       // 避障转出角度
    static constexpr uint32_t OA_TURN_OUT_TIME = 400;   // 避障转出时间 (ms)
    static constexpr uint32_t OA_PARALLEL_TIME = 500;   // 避障平行时间 (ms)
    static constexpr float OA_RETURN_ANGLE = 42.0f;     // 避障返回角度
    static constexpr uint32_t OA_TIMEOUT = 1300;        // 避障超时时间 (ms)
    static constexpr uint32_t STOP_AFTER_OBSTACLE_DELAY = 800; // 第二次避障后停车延迟 (ms)
};

constexpr float OUT_OF_LINE_THRESHOLD = 0.006;
// 出线检测
} // namespace Config

// 运行时参数结构体
struct RuntimeParams {
    float velTracking;
    PIDParams trackingPid;
    float speedAttenuationA;
    float preRingOffset;
    float ringVel;
    float ringOffset;
    uint32_t ringExitTime;
    PIDParams ringPid;
    // 避障参数
    float oaTurnAngle;
    uint32_t oaTurnOutTime;
    uint32_t oaParallelTime;
    float oaReturnAngle;
    uint32_t oaTimeout;
    uint32_t stopAfterObstacleDelay;
};

// 全局运行时参数
static RuntimeParams g_params = {};
static TrackMode g_currentMode = TRACK_MODE_AGGRESSIVE;

// 根据模式设置运行时参数
static void SetRuntimeParams(TrackMode mode)
{
    g_currentMode = mode;
    if (mode == TRACK_MODE_CONSERVATIVE)
    {
        g_params.velTracking = Config::ConservativeParams::VEL_TRACKING;
        g_params.trackingPid = Config::ConservativeParams::TRACKING_PID;
        g_params.speedAttenuationA = Config::ConservativeParams::SPEED_ATTENUATION_A;
        g_params.preRingOffset = Config::ConservativeParams::PRE_RING_OFFSET;
        g_params.ringVel = Config::ConservativeParams::RING_VEL;
        g_params.ringOffset = Config::ConservativeParams::RING_OFFSET;
        g_params.ringExitTime = Config::ConservativeParams::RING_EXIT_TIME;
        g_params.ringPid = Config::ConservativeParams::RING_PID;
        // 避障参数
        g_params.oaTurnAngle = Config::ConservativeParams::OA_TURN_ANGLE;
        g_params.oaTurnOutTime = Config::ConservativeParams::OA_TURN_OUT_TIME;
        g_params.oaParallelTime = Config::ConservativeParams::OA_PARALLEL_TIME;
        g_params.oaReturnAngle = Config::ConservativeParams::OA_RETURN_ANGLE;
        g_params.oaTimeout = Config::ConservativeParams::OA_TIMEOUT;
        g_params.stopAfterObstacleDelay = Config::ConservativeParams::STOP_AFTER_OBSTACLE_DELAY;
    }
    else
    {
        g_params.velTracking = Config::AggressiveParams::VEL_TRACKING;
        g_params.trackingPid = Config::AggressiveParams::TRACKING_PID;
        g_params.speedAttenuationA = Config::AggressiveParams::SPEED_ATTENUATION_A;
        g_params.preRingOffset = Config::AggressiveParams::PRE_RING_OFFSET;
        g_params.ringVel = Config::AggressiveParams::RING_VEL;
        g_params.ringOffset = Config::AggressiveParams::RING_OFFSET;
        g_params.ringExitTime = Config::AggressiveParams::RING_EXIT_TIME;
        g_params.ringPid = Config::AggressiveParams::RING_PID;
        // 避障参数
        g_params.oaTurnAngle = Config::AggressiveParams::OA_TURN_ANGLE;
        g_params.oaTurnOutTime = Config::AggressiveParams::OA_TURN_OUT_TIME;
        g_params.oaParallelTime = Config::AggressiveParams::OA_PARALLEL_TIME;
        g_params.oaReturnAngle = Config::AggressiveParams::OA_RETURN_ANGLE;
        g_params.oaTimeout = Config::AggressiveParams::OA_TIMEOUT;
        g_params.stopAfterObstacleDelay = Config::AggressiveParams::STOP_AFTER_OBSTACLE_DELAY;
    }
}

// ============================================================================
// 循迹上下文 (业务数据)
// ============================================================================
struct TrackContext
{
    float trackErr = 0.0f;
    float trackOutput = 0.0f;

    // 安全检测
    uint32_t safetyTimer = 0;
    bool safetyCheckEnabled = true; // 安全检测开关，默认开启

    // 环岛变量
    float enterAngle = 0.0;
    float turnTargetYaw = 0.0;

    // 避障使能窗口 (直角弯后一段时间内才能触发避障)
    uint32_t obstacleEnableTimer = 0;                       // 避障使能剩余时间 (ms)
    static constexpr uint32_t OBSTACLE_ENABLE_WINDOW = 200; // 直角弯后200ms内允许避障

    // 避障计数器 (用于第二次避障后停车)
    uint32_t obstacleCount = 0;
    uint32_t stopAfterObstacleTimer = 0;                       // 第二次避障后延迟停车计时器 (ms)
    bool stopAfterObstaclePending = false;                     // 是否等待延迟停车

    // 命令缓冲
    bool hasCmd = false;
    TrackCommand pendingCmd = TRACK_CMD_STOP;

    // 设置小车运动状态
    void SetCarStatus(float velocity, float angle)
    {
        SetTargetCarStatus(velocity, angle);
    }
};

using TrackStateMachine = StateMachine<TrackState, TrackContext>;
using TrackStateBase = IState<TrackContext, TrackState>;

class TrackingUtils
{
  public:
    static float NormalizeAngle(float angle)
    {
        while (angle > 180.0f)
            angle -= 360.0f;
        while (angle < -180.0f)
            angle += 360.0f;
        return angle;
    }

    // 计数滤波器 (Leaky Bucket)
    struct CountFilter
    {
        int count = 0;
        bool Update(bool cond, int threshold, int inc = 1, int dec = 2)
        {
            if (cond)
                count += inc;
            else
            {
                count -= dec;
                if (count < 0)
                    count = 0;
            }
            return count >= threshold;
        }
        void Reset()
        {
            count = 0;
        }
    };

    // 时间滤波器
    struct TimeFilter
    {
        uint32_t timer = 0;
        bool Update(bool cond, uint32_t dt, uint32_t threshold_ms)
        {
            if (cond)
                timer += dt;
            else
                timer = 0;
            return timer >= threshold_ms;
        }
        void Reset()
        {
            timer = 0;
        }
    };

    static void CalcTrack(TrackContext &ctx, PIDController &pid, float trackA = 1.0, float trackB = 1.23,
                          float k1 = 1.0, float k2 = 0.17)
    {
        auto res = MegAdcGetCalibratedResult();
        float denom = trackA * (res.l + res.r) + trackB * (res.lm + res.rm);

        ctx.trackErr = std::isnan(denom) ? 0.0f : (trackA * (res.l - res.r) + trackB * (res.lm - res.rm)) / denom;
        // constexpr float A = 1.0f, B = 0.2f;
        // k2=std::clamp(fabsf(ctx.trackErr)-0.4,0.0,0.4)*2;
        ctx.trackErr = k1 * ctx.trackErr + k2 * pow(ctx.trackErr, 3.0); // 误差整形

        // // err/speed 来达成不同速度下的自适应控制效果 只减小增益不扩大增益
        // ctx.trackErr =
        //     Car_GetTargetVelocity() < 30.0 ? ctx.trackErr / 2.5 : ctx.trackErr * 20.0 / Car_GetTargetVelocity();

        ctx.trackOutput = PID_Update_Positional(&pid, 0.0f, ctx.trackErr);
        // ctx.SetCarStatus(0.0, 0.0);
    }
};

// 停止状态
class StopState : public TrackStateBase
{
  public:
    StopState()
    {
        PID_Init(&this->pid, PID_MODE_POSITIONAL, 0.0, 0.0, 0.0, 0.0, 0.002);
    }
    void Enter(TrackContext &ctx) override
    {
    }
    TrackState Update(TrackContext &ctx, uint32_t) override
    {
        // static uint32_t tick = 0;
        // tick++;
        // float    // static uint32_t tick = 0;
        // tick++;
        // float a = 360.0f * sinf((float)tick * 0.02f);a = 360.0f * sinf((float)tick * 0.02f);
        TrackingUtils::CalcTrack(ctx, this->pid);

        LED_Command(1, true);
        ctx.SetCarStatus(0, 0);
        return TRACK_STATE_STOP;
    }
    const char *Name() const override
    {
        return "STOP";
    }

  private:
    PIDController pid = {};
};

// 循迹状态 (核心算法)
class TrackingState : public TrackStateBase
{
  public:
    static constexpr float DT = 0.01f;

    TrackingState()
    {
        // 初始化时使用默认参数，进入状态时会根据模式重新设置
        PID_InitWithParams(&this->pid, PID_MODE_POSITIONAL, &Config::AggressiveParams::TRACKING_PID, DT);
        PID_SetOutputLimit(&this->pid, -400.0f, 400.0f);
    }

    void Enter(TrackContext &) override
    {
        LED_Command(3, true);
        m_timer = 0;
        // 根据当前模式重新初始化PID
        PID_InitWithParams(&this->pid, PID_MODE_POSITIONAL, &g_params.trackingPid, DT);
        PID_SetOutputLimit(&this->pid, -400.0f, 400.0f);
    }

    TrackState Update(TrackContext &ctx, uint32_t dt) override
    {

        TrackingUtils::CalcTrack(ctx, this->pid);
        
        // 根据运行时参数计算速度
        float v = g_params.velTracking;
        if (g_params.speedAttenuationA > 0.0f)
        {
            v = g_params.velTracking / (1 + fabsf(g_params.speedAttenuationA * ctx.trackErr));
        }

        ctx.SetCarStatus(v, ctx.trackOutput);

        auto res = MegAdcGetCalibratedResult();

        // 检查第二次避障后延迟停车
        if (ctx.stopAfterObstaclePending)
        {
            if (ctx.stopAfterObstacleTimer > dt)
            {
                ctx.stopAfterObstacleTimer -= dt;
            }
            else
            {
                ctx.stopAfterObstaclePending = false;
                ctx.stopAfterObstacleTimer = 0;
                LOG_INFO("Stop after second obstacle!");
                return TRACK_STATE_STOP;
            }
        }

        // 更新避障使能窗口计时器
        if (ctx.obstacleEnableTimer > 0)
        {
            ctx.obstacleEnableTimer = (ctx.obstacleEnableTimer >= dt) ? (ctx.obstacleEnableTimer - dt) : 0;
        }

        // 避障检测 (仅在直角弯后指定时间窗口内有效)
        if (ctx.obstacleEnableTimer > 0 && m_obstacleFilter.Update(true, 7))
        {
            m_obstacleFilter.Reset();                                                                                                        
            return TRACK_STATE_OBSTACLE_AVOIDANCE;
        }

        // 直角弯检测
        if (m_rightAngleFilter.Update(
                fabsf(res.l - res.r) <= 0.4f && fabsf(res.lm - res.rm) >= 0.35f && (res.lm > 0.6 || res.rm > 0.6), 5))
        {
            m_rightAngleFilter.Reset();
            float currentYaw = IMU_GetYaw();
            // 判断转向方向: lm > rm -> 左转, rm > lm -> 右转
            if (res.lm < res.rm)
            {
                ctx.turnTargetYaw = currentYaw + 90.0f;
            }
            else
            {
                ctx.turnTargetYaw = currentYaw - 90.0f;
            }

            // 归一化目标角度到 [-180, 180]
            ctx.turnTargetYaw = TrackingUtils::NormalizeAngle(ctx.turnTargetYaw);

            return TRACK_STATE_RIGHT_ANGLE;
        }

        // 环岛检测
        if (m_ringFilter.Update(res.m >= 1.5 || m_ringFilter.count >= 5, 5))
        {
            m_ringFilter.Reset();
            return TRACK_STATE_PRE_RING;
        }
        return TRACK_STATE_TRACKING;
    }

    const char *Name() const override
    {
        return "TRACKING";
    }

  private:
    uint32_t m_timer = 0;
    PIDController pid = {};
    TrackingUtils::CountFilter m_ringFilter;
    TrackingUtils::CountFilter m_obstacleFilter = {};

    TrackingUtils::CountFilter m_rightAngleFilter;
    constexpr static float OBSTACLE_DIST_THRESHOLD = 400;
};

// 预环岛状态 通过打角让小车进入环岛 退出条件是时间到达或者中路电感不再检测到环岛入口
class PreRingState : public TrackStateBase
{
  public:
    static constexpr float DT = 0.01f;

    PreRingState()
    {
        PID_InitWithParams(&this->pid, PID_MODE_POSITIONAL, &Config::AggressiveParams::TRACKING_PID, DT);
        PID_SetOutputLimit(&this->pid, -200.0f, 200.0f);
    }

    void Enter(TrackContext &ctx) override
    {
        LED_Command(4, true);

        ctx.enterAngle = IMU_GetYaw();
        printf("Enter Ring at Angle: %s\n", float_to_str(ctx.enterAngle));
        m_timer.Reset();
    }

    TrackState Update(TrackContext &ctx, uint32_t dt) override
    {
        TrackingUtils::CalcTrack(ctx, this->pid);
        ctx.SetCarStatus(g_params.velTracking, ctx.trackOutput + g_params.preRingOffset);
        return m_timer.Update(true, dt, 500) ? TRACK_STATE_RING : TRACK_STATE_PRE_RING;
    }

    const char *Name() const override
    {
        return "PRE_RING";
    }

  private:
    PIDController pid = {};
    TrackingUtils::TimeFilter m_timer;
};

class RingState : public TrackStateBase
{
  public:
    RingState()
    {
        PID_InitWithParams(&this->pid, PID_MODE_POSITIONAL, &Config::AggressiveParams::RING_PID, 0.01);
        PID_SetOutputLimit(&this->pid, -400.0f, 400.0f);
    }

    void Enter(TrackContext &ctx) override
    {
        LED_Command(2, true);
        m_timer.Reset();
        // 根据当前模式重新初始化PID
        PID_InitWithParams(&this->pid, PID_MODE_POSITIONAL, &g_params.ringPid, 0.01);
        PID_SetOutputLimit(&this->pid, -400.0f, 400.0f);
    }
    TrackState Update(TrackContext &ctx, uint32_t dt) override
    {

        // 复用循迹状态的逻辑
        TrackingUtils::CalcTrack(ctx, this->pid);
        ctx.SetCarStatus(g_params.ringVel, ctx.trackOutput + g_params.ringOffset);  // 使用运行时参数


        // 角度判断
        float currentAngle = IMU_GetYaw();
        float diff = fabsf(TrackingUtils::NormalizeAngle(currentAngle - (ctx.enterAngle + 20)));
        printf("%s,%s\n", float_to_str(currentAngle), float_to_str(ctx.enterAngle));
        // 还需要加一个最小时间限制，防止刚进环岛就误判退出
        if (m_timer1.Update(m_timer.Update(true, dt, 450) && diff < 50.0f, dt, g_params.ringExitTime))
        {
            return TRACK_STATE_EXIT_RING;
        }

        return TRACK_STATE_RING;
    }
    const char *Name() const override
    {
        return "RING";
    }

  private:
    PIDController pid = {};
    TrackingUtils::TimeFilter m_timer;
    TrackingUtils::TimeFilter m_timer1;

};

// 出环岛状态 同预环岛 打角一段时间后退出
class ExitRingState : public TrackStateBase
{
  public:
    static constexpr PIDParams DEFAULT_PARAMS = {.kp = 20.0f, .ki = 0.0f, .kd = 0.0f, .Kf = 0.0f};

    ExitRingState()
    {
        PID_InitWithParams(&this->angle_pid, PID_MODE_POSITIONAL, &DEFAULT_PARAMS, 0.01);
        PID_SetOutputLimit(&this->angle_pid, -700.0f, 700.0f);
    }

    void Enter(TrackContext &ctx) override
    {
        ctx.safetyCheckEnabled = false;

        LED_Command(4, true);

        m_timer.Reset();
    }

    TrackState Update(TrackContext &ctx, uint32_t dt) override
    {
        // TrackingUtils::CalcTrack(ctx, this->pid);
        // ctx.SetCarStatus(g_params.velTracking, ctx.trackOutput - 20);

        // 处理角度跨越 180 度的问题
        float currentYaw = IMU_GetYaw();
        float error = TrackingUtils::NormalizeAngle(ctx.enterAngle - currentYaw);

        float angular_output = PID_Update_Positional(&angle_pid, error, 0.0f);
        ctx.SetCarStatus(g_params.velTracking + 15, angular_output);
        return m_timer.Update(true, dt, 900) ? TRACK_STATE_TRACKING : TRACK_STATE_EXIT_RING;
        // return TRACK_STATE_EXIT_RING;
    }
    void Exit(TrackContext &ctx) override
    {
        ctx.safetyCheckEnabled = true;
    }

    const char *Name() const override
    {
        return "EXIT_RING";
    }

  private:
    PIDController angle_pid = {};

    TrackingUtils::TimeFilter m_timer;
};

// 直角弯状态
class RightAngleTurnState : public TrackStateBase
{
  public:
    static constexpr PIDParams DEFAULT_PARAMS = {.kp = 20.0f, .ki = 0.0f, .kd = 0.0f, .Kf = 0.0f};

    RightAngleTurnState()
    {
        PID_InitWithParams(&this->angle_pid, PID_MODE_POSITIONAL, &DEFAULT_PARAMS, 0.01);
        PID_SetOutputLimit(&this->angle_pid, -500.0f, 500.0f);
    }

    void Enter(TrackContext &ctx) override
    {
        LED_Command(5, true);
        m_timer.Reset();
    }

    TrackState Update(TrackContext &ctx, uint32_t dt) override
    {
        // 处理角度跨越 180 度的问题
        float currentYaw = IMU_GetYaw();
        float error = TrackingUtils::NormalizeAngle(ctx.turnTargetYaw - currentYaw);

        float angular_output = PID_Update_Positional(&angle_pid, error, 0.0f);
        ctx.SetCarStatus(g_params.velTracking, angular_output);

        // 持续一段时间后退出
        if (m_timer.Update(true, dt, 500))
        {
            return TRACK_STATE_TRACKING;
        }
        return TRACK_STATE_RIGHT_ANGLE;
    }

    void Exit(TrackContext &ctx) override
    {
        // 直角弯完成后，开启避障检测窗口
        ctx.obstacleEnableTimer = ctx.OBSTACLE_ENABLE_WINDOW;
    }

    const char *Name() const override
    {
        return "RIGHT_ANGLE";
    }

  private:
    PIDController angle_pid = {};
    TrackingUtils::TimeFilter m_timer;
};

// 避障状态 退出条件检测时间或者误差重新归0
class ObstacleAvoidanceState : public TrackStateBase
{
  public:
    static constexpr PIDParams DEFAULT_PARAMS = {.kp = 20.0f, .ki = 0.0f, .kd = 0.0f, .Kf = 0.0f};

    ObstacleAvoidanceState()
    {
        PID_InitWithParams(&this->angle_pid, PID_MODE_POSITIONAL, &DEFAULT_PARAMS, 0.01);
        PID_SetOutputLimit(&this->angle_pid, -300.0f, 300.0f);
    }

    void Enter(TrackContext &ctx) override
    {
        // 关闭安全检测
        ctx.safetyCheckEnabled = false;
        m_timer = 0;
        this->origin_angle = IMU_GetYaw();
        this->phase = PHASE_TURN_OUT;
        LED_Command(6, true);
    }

    void Exit(TrackContext &ctx) override
    {
        // 恢复安全检测
        ctx.safetyCheckEnabled = true;
        ctx.obstacleEnableTimer = 0;

        // 避障计数器递增
        ctx.obstacleCount++;
        LOG_INFO("Obstacle avoidance completed, count: %lu", ctx.obstacleCount);

        // 第二次避障成功后，启动延迟停车计时器
        if (ctx.obstacleCount >= 2)
        {
            ctx.stopAfterObstaclePending = true;
            ctx.stopAfterObstacleTimer = g_params.stopAfterObstacleDelay;
            LOG_INFO("Second obstacle done, will stop after %lu ms", g_params.stopAfterObstacleDelay);
        }
    }

    TrackState Update(TrackContext &ctx, uint32_t dt) override
    {
        m_timer += dt;
        float currentYaw = IMU_GetYaw();
        float targetYaw = origin_angle;

        switch (phase)
        {
        case PHASE_TURN_OUT:
            // 第一阶段 向右转出
            targetYaw = origin_angle - g_params.oaTurnAngle;
            if (m_timer >= g_params.oaTurnOutTime)
            {
                phase = PHASE_PARALLEL;
                m_timer = 0;
            }
            break;

        case PHASE_PARALLEL:
            // 回正平行直行 (回到初始角度)
            targetYaw = origin_angle;
            if (m_timer >= g_params.oaParallelTime)
            {
                phase = PHASE_RETURN;
                m_timer = 0;
            }
            break;

        case PHASE_RETURN:
            // 向左切回
            targetYaw = origin_angle + g_params.oaReturnAngle;

            // 检测是否回到线上
            {
                auto res = MegAdcGetCalibratedResult();
                // 简单的回线判断，可根据实际情况调整
                float sum = res.l + res.r + res.lm + res.rm;
                printf("OA sum: %s\n", float_to_str(sum ));
                if (sum > 0.1f)
                {
                    // return TRACK_STATE_STOP;
                    return TRACK_STATE_TRACKING;
                }
            }
            // 超时强制停车
            if (m_timer >= g_params.oaTimeout)
            {
                return TRACK_STATE_STOP;
            }
            break;
        }

        // 计算角度误差并归一化
        float error = TrackingUtils::NormalizeAngle(targetYaw - currentYaw);

        float angular_output = PID_Update_Positional(&angle_pid, error, 0.0f);
        ctx.SetCarStatus(g_params.velTracking, angular_output);

        return TRACK_STATE_OBSTACLE_AVOIDANCE;
    }

    const char *Name() const override
    {
        return "OBSTACLE_AVOIDANCE";
    }

  private:
    enum Phase
    {
        PHASE_TURN_OUT,
        PHASE_PARALLEL,
        PHASE_RETURN
    };

    uint32_t m_timer = 0;
    float origin_angle = 0;
    Phase phase = PHASE_TURN_OUT;
    PIDController angle_pid = {};
};

// ============================================================================
// 状态机实例
// ============================================================================
static TrackStateMachine s_fsm;

// ============================================================================
// 内部辅助函数声明
// ============================================================================
static void HandleCommand();
static bool CheckSafety(uint32_t dt);
static void RegisterParameters();

// ============================================================================
// C 接口实现
// ============================================================================
extern "C"
{

    void TrackInit(void)
    {
        // 初始化运行时参数（默认激进模式）
        SetRuntimeParams(TRACK_MODE_AGGRESSIVE);

        // 注册所有状态
        s_fsm.RegisterState(TRACK_STATE_STOP, std::make_unique<StopState>());
        s_fsm.RegisterState(TRACK_STATE_TRACKING, std::make_unique<TrackingState>());
        s_fsm.RegisterState(TRACK_STATE_PRE_RING, std::make_unique<PreRingState>());
        s_fsm.RegisterState(TRACK_STATE_RING, std::make_unique<RingState>());
        s_fsm.RegisterState(TRACK_STATE_EXIT_RING, std::make_unique<ExitRingState>());
        s_fsm.RegisterState(TRACK_STATE_RIGHT_ANGLE, std::make_unique<RightAngleTurnState>());
        s_fsm.RegisterState(TRACK_STATE_OBSTACLE_AVOIDANCE, std::make_unique<ObstacleAvoidanceState>());

        // 注册参数
        RegisterParameters();

        // 设置初始状态
        s_fsm.ChangeState(TRACK_STATE_STOP);
    }

    void TrackHandler(void)
    {
        uint32_t dt = 10; // 假设调用周期为10ms
        HandleCommand();
        if (CheckSafety(dt))
        {
            s_fsm.ChangeState(TRACK_STATE_STOP);
            return;
        }

        // 更新当前状态
        s_fsm.Update(dt);
    }

    void TrackSetCommand(TrackCommand cmd)
    {
        auto &ctx = s_fsm.GetContext();
        ctx.pendingCmd = cmd;
        ctx.hasCmd = true;
    }

    TrackState TrackGetCurrentState(void)
    {
        return s_fsm.GetCurrentStateID();
    }

    const char *TrackGetStateName(TrackState state)
    {
        if (state == s_fsm.GetCurrentStateID())
        {
            return s_fsm.GetCurrentStateName();
        }

        static const char *names[] = {"STOP",      "TRACKING",    "PRE_RING",          "RING",
                                      "EXIT_RING", "RIGHT_ANGLE", "OBSTACLE_AVOIDANCE"};
        return (state < TRACK_STATE_COUNT) ? names[state] : "UNKNOWN";
    }

} // extern "C"

// ============================================================================
// 内部辅助函数实现
// ============================================================================
static void HandleCommand()
{
    auto &ctx = s_fsm.GetContext();
    if (!ctx.hasCmd)
        return;
    ctx.hasCmd = false;

    LOG_INFO("Cmd: %d", ctx.pendingCmd);

    switch (ctx.pendingCmd)
    {
    case TRACK_CMD_START:
        if (s_fsm.GetCurrentStateID() == TRACK_STATE_STOP)
        {
            LOG_INFO("Start Tracking (Aggressive Mode)");
            SetRuntimeParams(TRACK_MODE_AGGRESSIVE);
            ctx.obstacleCount = 0; // 清空避障计数器
            s_fsm.ChangeState(TRACK_STATE_TRACKING);
        }
        break;
    case TRACK_CMD_START_CONSERVATIVE:
        if (s_fsm.GetCurrentStateID() == TRACK_STATE_STOP)
        {
            LOG_INFO("Start Tracking (Conservative Mode)");
            SetRuntimeParams(TRACK_MODE_CONSERVATIVE);
            ctx.obstacleCount = 0; // 清空避障计数器
            s_fsm.ChangeState(TRACK_STATE_TRACKING);
        }
        break;
    case TRACK_CMD_STOP:
    case TRACK_CMD_RESET:
        LOG_INFO("Stop/Reset");
        s_fsm.ChangeState(TRACK_STATE_STOP);
        break;
    }
}

static bool CheckSafety(uint32_t dt)
{
    auto &ctx = s_fsm.GetContext();

    // 如果安全检测被禁用，直接返回false
    if (!ctx.safetyCheckEnabled)
    {
        return false;
    }

    // 停止状态下不进行安全检测，避免误报
    if (s_fsm.GetCurrentStateID() == TRACK_STATE_STOP)
    {
        return false;
    }

    bool hasEmergency = false;

    // 1. 出线检测
    auto adc = MegAdcGetCalibratedResult();
    float sum = adc.l + adc.r + adc.lm + adc.rm;
    if (sum < Config::OUT_OF_LINE_THRESHOLD)
    {
        static uint32_t lastLog = 0;
        if (HAL_GetTick() - lastLog > 100)
        {
            LOG_INFO("Out of line!");
            lastLog = HAL_GetTick();
        }
        // 触发紧急情况显示 - 丢线
        OledServiceSetEmergency(OLED_EMERGENCY_OUT_OF_LINE, "OUT OF LINE!");
        hasEmergency = true;
    }

    // 2. 翻车检测
    float roll = IMU_GetRoll();
    float pitch = IMU_GetPitch();
    // 设定翻车阈值，例如 60 度
    if (fabsf(roll) > 60.0f || fabsf(pitch) > 60.0f)
    {
        static uint32_t lastLog = 0;
        if (HAL_GetTick() - lastLog > 1000)
        {
            LOG_INFO("Car Flipped!");
            lastLog = HAL_GetTick();
        }
        OledServiceSetEmergency(OLED_EMERGENCY_ROLL_OVER, "CAR FLIPPED!");
        hasEmergency = true;
    }
    // 只有在没有检测到任何紧急情况时才清除紧急显示
    if (!hasEmergency)
    {
        OledServiceClearEmergency();
    }

    return hasEmergency;
}

// 安全检测开关功能实现
void TrackSetSafetyCheckEnabled(bool enabled)
{
    auto &ctx = s_fsm.GetContext();
    ctx.safetyCheckEnabled = enabled;

    // 如果禁用安全检测，清除当前的紧急显示
    if (!enabled)
    {
        OledServiceClearEmergency();
    }
}

bool TrackIsSafetyCheckEnabled(void)
{
    auto &ctx = s_fsm.GetContext();
    return ctx.safetyCheckEnabled;
}

static void RegisterParameters()
{
    static ParamDesc params[] = {
        {"State",
         PARAM_TYPE_ENUM,
         {.e = {nullptr, nullptr, []() -> const char * { return TrackGetStateName(TrackGetCurrentState()); }}},
         0,
         1,
         PARAM_MASK_OLED},
        {"Track_Err",
         PARAM_TYPE_FLOAT,
         {.f = {[]() { return s_fsm.GetContext().trackErr; }, nullptr}},
         0,
         1,
         PARAM_MASK_SERIAL},
        {"Track_Out",
         PARAM_TYPE_FLOAT,
         {.f = {[]() { return s_fsm.GetContext().trackOutput; }, nullptr}},
         0,
         1,
         PARAM_MASK_SERIAL},
        {"Mode",
         PARAM_TYPE_ENUM,
         {.e = {nullptr, nullptr, []() -> const char * { return g_currentMode == TRACK_MODE_CONSERVATIVE ? "CONS" : "AGGR"; }}},
         0,
         1,
         PARAM_MASK_OLED},
    };

    // for (auto &p : params)
    // {
    //     ParamServer_Register(&p);
    // }
}

// 获取当前运行模式
TrackMode TrackGetCurrentMode(void)
{
    return g_currentMode;
}