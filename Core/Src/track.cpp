#include "track.h"
#include "common.h"
#include "fsm.hpp"
#include "led.h"
#include <cmath>

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
constexpr uint32_t RING_DETECT_MS = 500;
constexpr uint32_t IN_RING_MS = 1000;
constexpr uint32_t ROLL_OVER_MS = 200;

constexpr float VEL_PRE_RING = 2.0f;
constexpr float ANG_PRE_RING = 1.0f;
constexpr float VEL_RING = 1.5f;
constexpr float ANG_RING = 0.8f;
constexpr float VEL_EXIT = 2.0f;
constexpr float ANG_EXIT = -0.5f;
constexpr float VEL_TRACKING = 40.0f;

constexpr float ROLL_OVER_ANGLE = 40.0f;

// 避障参数
constexpr uint16_t OBSTACLE_DIST_THRESHOLD = 400; // mm
constexpr float VEL_OBSTACLE = 30.0f;
constexpr float OBSTACLE_RADIUS = -0.67f; // 避障半径
constexpr float ANG_OBSTACLE_PRE_TURN = 30.0f; // 预偏转角速度
constexpr uint32_t OBSTACLE_PRE_TURN_TIME_MS = 100; // 预偏转时间

// 出线检测
constexpr float OUT_OF_LINE_THRESHOLD = 0.005f;
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
    bool safetyCheckEnabled = true; // 安全检测开关，默认开启

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
    }
    TrackState Update(TrackContext &ctx, uint32_t) override
    {
        LED_Command(1, true);
        ctx.SetCarStatus(0, 0.0);
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
        LED_Command(2, true);

        m_timer = 0;
    }
    // 循迹主体 输出目标角速度来让小车在线上
    void Track(TrackContext &ctx, uint32_t dt)
    {
        auto res = MegAdcGetCalibratedResult();
        float denom = ctx.trackA * (res.l + res.r) + ctx.trackB * (res.lm + res.rm);

        ctx.trackErr =
            std::isnan(denom) ? 0.0f : (ctx.trackA * (res.l - res.r) + ctx.trackB * (res.lm - res.rm)) / denom;
        // LOG_INFO("%s\n",float_to_str(ctx.trackErr));

        constexpr float A = 1.0f, B = 0.5f;
        ctx.trackErr = A * ctx.trackErr + B * pow(ctx.trackErr, 3.0); // 误差整形

        // err/speed 来达成不同速度下的自适应控制效果 只减小增益不扩大增益
        ctx.trackErr = Car_GetTargetVelocity() < 20.0 ? ctx.trackErr
                                                      : ctx.trackErr * Config::VEL_TRACKING / Car_GetTargetVelocity();

        ctx.trackOutput = PID_Update_Positional(&ctx.pid, 0.0f, ctx.trackErr);
        // ctx.SetCarStatus(0.0, 0.0);
        ctx.SetCarStatus(Config::VEL_TRACKING, ctx.trackOutput);
    }
    TrackState Update(TrackContext &ctx, uint32_t dt) override
    {
        Track(ctx, dt);
        // // 2. 避障检测
        static int TofCount = 0;
        if (TofGetDistance() < Config::OBSTACLE_DIST_THRESHOLD)
        {
            TofCount++;
        }
        else
        {
            TofCount -= 2;
            if (TofCount < 0)
                TofCount = 0;
        }
        if (TofCount >= 20)
        {
            return TRACK_STATE_OBSTACLE_AVOIDANCE;
        }

        // // 3. 环岛检测
        // if (Detection::IsRingDetected(ctx))
        // {
        //     if ((m_timer += dt) >= Config::RING_DETECT_MS)
        //         return TRACK_STATE_PRE_RING;
        // }
        // else
        // {
        //     m_timer = 0;
        // }
        return TRACK_STATE_TRACKING;
    }

    const char *Name() const override
    {
        return "TRACKING";
    }

  private:
    uint32_t m_timer = 0;
};

// 预环岛状态 通过打角让小车进入环岛 退出条件是时间到达或者中路电感不再检测到环岛入口
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

class RingState : public TrackingState
{
  public:
    void Enter(TrackContext &ctx) override
    {
    }
    TrackState Update(TrackContext &ctx, uint32_t dt) override
    {
        // 复用循迹状态的逻辑
        TrackingState::Track(ctx, dt);
        // TODO: 完成退出环岛的逻辑

        return TRACK_STATE_RING;
    }
    const char *Name() const override
    {
        return "RING";
    }
};

// 出环岛状态 同预环岛 打角一段时间后退出
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

// 避障状态 退出条件检测时间或者误差重新归0
class ObstacleAvoidanceState : public TrackStateBase
{
  public:
    ObstacleAvoidanceState()
    {
        PID_Init(&angle_pid, PID_MODE_POSITIONAL, 1.0f, 0.0f, 0.0f, 0.0f, 0.02f);
        PID_SetOutputLimit(&angle_pid, -100.0f, 100.0f);
    }
    void Enter(TrackContext &ctx) override
    {
        // 关闭安全检测
        ctx.safetyCheckEnabled = false;

        // 进入预偏转阶段
        m_isPreTurn = true;
        m_timer = 0;
        this->origin_angle=IMU_GetYaw();
    }

    void Exit(TrackContext &ctx) override
    {
        // 恢复安全检测
        ctx.safetyCheckEnabled = true;
    }

    TrackState Update(TrackContext &ctx, uint32_t dt) override
    {
        // if (m_isPreTurn)
        // {
        //     ctx.SetCarStatus(Config::VEL_OBSTACLE, Config::ANG_OBSTACLE_PRE_TURN);
        //     if ((m_timer += dt) >= Config::OBSTACLE_PRE_TURN_TIME_MS)
        //     {
        //         m_isPreTurn = false;
        //         m_timer = 0;
        //     }
        //     return TRACK_STATE_OBSTACLE_AVOIDANCE;
        // }

        // // 持续设置速度，防止被其他地方覆盖
        // float omega = Config::VEL_OBSTACLE / Config::OBSTACLE_RADIUS;
        // ctx.SetCarStatus(Config::VEL_OBSTACLE, omega);

        // if ((m_timer += dt) >= m_targetTime)
        // {
        //     return TRACK_STATE_STOP;
        // }
        float angular_output= PID_Update_Positional(&angle_pid, this->target_angle, IMU_GetYaw());
        ctx.SetCarStatus(0,angular_output );
        return TRACK_STATE_OBSTACLE_AVOIDANCE;
    }

    const char *Name() const override
    {
        return "OBSTACLE_AVOIDANCE";
    }

  private:
    uint32_t m_timer = 0;
    uint32_t m_targetTime = 2000;
    float target_angle = 0;
    float origin_angle=0;
    bool m_isPreTurn = true;
    PIDController angle_pid={};
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
        PID_Init(&ctx.pid, PID_MODE_POSITIONAL, 18.00f, 0.0f, 0.0f, 0.0f, 0.02f);
        PID_SetOutputLimit(&ctx.pid, -100.0f, 50.0f);

        // 注册参数
        RegisterParameters();

        // 设置初始状态
        s_fsm.ChangeState(TRACK_STATE_OBSTACLE_AVOIDANCE);
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

    LOG_INFO("Cmd: %d", ctx.pendingCmd);

    switch (ctx.pendingCmd)
    {
    case TRACK_CMD_START:
        if (s_fsm.GetCurrentStateID() == TRACK_STATE_STOP)
        {
            LOG_INFO("Start Tracking");
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

    bool hasEmergency = false;

    // 1. 出线检测
    auto adc = MegAdcGetResult();
    float sum = adc.l + adc.r + adc.lm + adc.rm;
    if (sum < ctx.outOfLineThreshold)
    {
        static uint32_t lastLog = 0;
        if (HAL_GetTick() - lastLog > 1000)
        {
            LOG_INFO("Out of line!");
            lastLog = HAL_GetTick();
        }
        // 触发紧急情况显示 - 丢线
        OledServiceSetEmergency(OLED_EMERGENCY_OUT_OF_LINE, "OUT OF LINE!");
        hasEmergency = true;
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
            // 触发紧急情况显示 - 翻车
            OledServiceSetEmergency(OLED_EMERGENCY_ROLL_OVER, "ROLL OVER!");
            ctx.safetyTimer = 0;
            hasEmergency = true;
        }
    }
    else
    {
        ctx.safetyTimer = 0;
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
        // {"Track_Out",
        //  PARAM_TYPE_FLOAT,
        //  {.f = {[]() { return s_fsm.GetContext().trackOutput; }, nullptr}},
        //  0,
        //  1,
        //  PARAM_MASK_SERIAL},
    };

    // for (auto &p : params)
    // {
    //     ParamServer_Register(&p);
    // }
}