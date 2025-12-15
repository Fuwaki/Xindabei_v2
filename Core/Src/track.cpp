#include "track.h"
#include "fsm.hpp"
#include <cmath>

extern "C"
{
#include "car_control.h"
#include "imu.h"
#include "log.h"
#include "meg_adc.h"
#include "param_server.h"
#include "pid.h"
#include "tof.h"
}

// ============================================================================
// 业务配置
// ============================================================================
namespace Config
{
constexpr uint32_t RING_DETECT_MS = 500;
constexpr uint32_t IN_RING_MS = 1000;
constexpr uint32_t ROLL_OVER_MS = 200;

constexpr float VEL_PRE_RING = 2.0f;
constexpr float ANG_PRE_RING = 1.0f;
constexpr float VEL_RING = 1.5f;
constexpr float ANG_RING = 0.8f;
constexpr float VEL_EXIT = 2.0f;
constexpr float ANG_EXIT = -0.5f;
constexpr float VEL_TRACKING = 3.0f;

constexpr float ROLL_OVER_ANGLE = 45.0f;

// 避障参数
constexpr uint16_t OBSTACLE_DIST_THRESHOLD = 200; // mm
constexpr float VEL_OBSTACLE = 1.0f;
constexpr float ANG_OBSTACLE = 1.5f;        // 半圆角速度
constexpr uint32_t OBSTACLE_TIME_MS = 2000; // 避障动作持续时间

// 出线检测
constexpr float OUT_OF_LINE_THRESHOLD = 0.5f;
} // namespace Config

// ============================================================================
// 循迹上下文 (业务数据)
// ============================================================================
struct TrackContext
{
    // PID控制器
    PIDController pid;

    // 循迹参数
    float trackA = 1.0f;
    float trackB = 0.5f;
    float trackErr = 0.0f;
    float trackOutput = 0.0f;
    float outOfLineThreshold = Config::OUT_OF_LINE_THRESHOLD;

    // 安全检测
    uint32_t safetyTimer = 0;

    // 命令缓冲
    bool hasCmd = false;
    TrackCommand pendingCmd = TRACK_CMD_STOP;

    // 设置小车运动状态
    void SetCarStatus(float velocity, float angle)
    {
        SetTargetCarStatus(velocity, angle);
    }
};

namespace Detection
{
// TODO: 实现环岛检测逻辑
bool IsRingDetected(TrackContext &)
{
    return false;
}
bool IsInRingDetected(TrackContext &)
{
    return false;
}
bool IsExitRingDetected(TrackContext &)
{
    return false;
}
} // namespace Detection

using TrackStateMachine = StateMachine<TrackState, TrackContext>;
using TrackStateBase = IState<TrackContext, TrackState>;

// 停止状态
class StopState : public TrackStateBase
{
  public:
    void Enter(TrackContext &ctx) override
    {
        ctx.SetCarStatus(0, 0);
    }
    TrackState Update(TrackContext &, uint32_t) override
    {
        return TRACK_STATE_STOP;
    }
    const char *Name() const override
    {
        return "STOP";
    }
};

// 循迹状态 (核心算法)
class TrackingState : public TrackStateBase
{
  public:
    void Enter(TrackContext &) override
    {
        m_timer = 0;
    }

    TrackState Update(TrackContext &ctx, uint32_t dt) override
    {
        // 1. 循迹控制
        auto res = MegAdcGetResult();
        float denom = ctx.trackA * (res.l + res.r) + ctx.trackB * (res.lm + res.rm);

        ctx.trackErr =
            (fabsf(denom) > 1e-5f) ? (ctx.trackA * (res.l - res.r) + ctx.trackB * (res.lm - res.rm)) / denom : 0.0f;

        ctx.trackOutput = PID_Update_Positional(&ctx.pid, 0.0f, ctx.trackErr);
        ctx.SetCarStatus(Config::VEL_TRACKING, ctx.trackOutput);

        // 2. 避障检测
        if (TofGetDistance() < Config::OBSTACLE_DIST_THRESHOLD)
        {
            return TRACK_STATE_OBSTACLE_AVOIDANCE;
        }

        // 3. 环岛检测
        if (Detection::IsRingDetected(ctx))
        {
            if ((m_timer += dt) >= Config::RING_DETECT_MS)
                return TRACK_STATE_PRE_RING;
        }
        else
        {
            m_timer = 0;
        }
        return TRACK_STATE_TRACKING;
    }

    const char *Name() const override
    {
        return "TRACKING";
    }

  private:
    uint32_t m_timer = 0;
};

// 预环岛状态
class PreRingState : public TrackStateBase
{
  public:
    void Enter(TrackContext &ctx) override
    {
        ctx.SetCarStatus(Config::VEL_PRE_RING, Config::ANG_PRE_RING);
        m_timer = 0;
    }

    TrackState Update(TrackContext &, uint32_t dt) override
    {
        return ((m_timer += dt) >= 1000) ? TRACK_STATE_RING : TRACK_STATE_PRE_RING;
    }

    const char *Name() const override
    {
        return "PRE_RING";
    }

  private:
    uint32_t m_timer = 0;
};

// 环岛状态
class RingState : public TrackStateBase
{
  public:
    void Enter(TrackContext &ctx) override
    {
        ctx.SetCarStatus(Config::VEL_RING, Config::ANG_RING);
        m_timer = 0;
    }

    TrackState Update(TrackContext &ctx, uint32_t dt) override
    {
        if (Detection::IsInRingDetected(ctx))
        {
            if ((m_timer += dt) >= Config::IN_RING_MS)
                return TRACK_STATE_EXIT_RING;
        }
        else
        {
            m_timer = 0;
        }
        return TRACK_STATE_RING;
    }

    const char *Name() const override
    {
        return "RING";
    }

  private:
    uint32_t m_timer = 0;
};

// 出环岛状态
class ExitRingState : public TrackStateBase
{
  public:
    void Enter(TrackContext &ctx) override
    {
        ctx.SetCarStatus(Config::VEL_EXIT, Config::ANG_EXIT);
        m_timer = 0;
    }

    TrackState Update(TrackContext &ctx, uint32_t dt) override
    {
        if (Detection::IsExitRingDetected(ctx))
        {
            if ((m_timer += dt) >= 500)
                return TRACK_STATE_TRACKING;
        }
        else
        {
            m_timer = 0;
        }
        return TRACK_STATE_EXIT_RING;
    }

    const char *Name() const override
    {
        return "EXIT_RING";
    }

  private:
    uint32_t m_timer = 0;
};

// 避障状态
class ObstacleAvoidanceState : public TrackStateBase
{
  public:
    void Enter(TrackContext &ctx) override
    {
        ctx.SetCarStatus(Config::VEL_OBSTACLE, Config::ANG_OBSTACLE);
        m_timer = 0;
    }

    TrackState Update(TrackContext &, uint32_t dt) override
    {
        if ((m_timer += dt) >= Config::OBSTACLE_TIME_MS)
        {
            return TRACK_STATE_TRACKING;
        }
        return TRACK_STATE_OBSTACLE_AVOIDANCE;
    }

    const char *Name() const override
    {
        return "OBSTACLE_AVOIDANCE";
    }

  private:
    uint32_t m_timer = 0;
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
        // 注册所有状态
        s_fsm.RegisterState(TRACK_STATE_STOP, std::make_unique<StopState>());
        s_fsm.RegisterState(TRACK_STATE_TRACKING, std::make_unique<TrackingState>());
        s_fsm.RegisterState(TRACK_STATE_PRE_RING, std::make_unique<PreRingState>());
        s_fsm.RegisterState(TRACK_STATE_RING, std::make_unique<RingState>());
        s_fsm.RegisterState(TRACK_STATE_EXIT_RING, std::make_unique<ExitRingState>());
        s_fsm.RegisterState(TRACK_STATE_OBSTACLE_AVOIDANCE, std::make_unique<ObstacleAvoidanceState>());

        // 初始化PID
        auto &ctx = s_fsm.GetContext();
        PID_Init(&ctx.pid, PID_MODE_POSITIONAL, 0.85f, 0.0f, 0.0f, 0.0f, 0.1f);
        PID_SetOutputLimit(&ctx.pid, -2.0f, 2.0f);

        // 注册参数
        RegisterParameters();

        // 设置初始状态
        s_fsm.ChangeState(TRACK_STATE_STOP);
    }

    void TrackHandler(void)
    {
        uint32_t dt = 20; // 假设调用周期为20ms
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

        static const char *names[] = {"STOP", "TRACKING", "PRE_RING", "RING", "EXIT_RING", "OBSTACLE_AVOIDANCE"};
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

    switch (ctx.pendingCmd)
    {
    case TRACK_CMD_START:
        if (s_fsm.GetCurrentStateID() == TRACK_STATE_STOP)
        {
            s_fsm.ChangeState(TRACK_STATE_TRACKING);
        }
        break;
    case TRACK_CMD_STOP:
    case TRACK_CMD_RESET:
        s_fsm.ChangeState(TRACK_STATE_STOP);
        break;
    }
}

static bool CheckSafety(uint32_t dt)
{
    auto &ctx = s_fsm.GetContext();

    // 1. 出线检测
    auto adc = MegAdcGetResult();
    float sum = adc.l + adc.r + adc.lm + adc.rm;
    if (sum < ctx.outOfLineThreshold)
    {
        return true;
    }

    // 2. 翻车检测
    imu_data imu = IMUGetData();
    float roll = atan2f(imu.accel.ay, imu.accel.az) * 180.0f / M_PI;
    float pitch =
        atan2f(-imu.accel.ax, sqrtf(imu.accel.ay * imu.accel.ay + imu.accel.az * imu.accel.az)) * 180.0f / M_PI;

    if (fabsf(roll) > Config::ROLL_OVER_ANGLE || fabsf(pitch) > Config::ROLL_OVER_ANGLE)
    {
        ctx.safetyTimer += dt;
        if (ctx.safetyTimer >= Config::ROLL_OVER_MS)
        {
            LOG_WARN("ROLL OVER DETECTED!");
            ctx.safetyTimer = 0;
            return true;
        }
    }
    else
    {
        ctx.safetyTimer = 0;
    }
    return false;
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
        {"Track_A",
         PARAM_TYPE_FLOAT,
         {.f = {[]() { return s_fsm.GetContext().trackA; }, [](float v) { s_fsm.GetContext().trackA = v; }}},
         0.1f,
         0,
         0},
        {"Track_B",
         PARAM_TYPE_FLOAT,
         {.f = {[]() { return s_fsm.GetContext().trackB; }, [](float v) { s_fsm.GetContext().trackB = v; }}},
         0.1f,
         0,
         0},
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
    };

    for (auto &p : params)
    {
        ParamServer_Register(&p);
    }
}