#include "track.h"
#include "common.h"
#include "fsm.hpp"
#include "led.h"
#include <algorithm>
#include <cmath>
#include <stdint.h>
#include <type_traits>

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
constexpr uint32_t IN_RING_MS = 1000;
constexpr uint32_t ROLL_OVER_MS = 200;

constexpr float VEL_PRE_RING = 2.0f;
constexpr float ANG_PRE_RING = 1.0f;
constexpr float VEL_RING = 1.5f;
constexpr float ANG_RING = 0.8f;
constexpr float VEL_EXIT = 2.0f;
constexpr float ANG_EXIT = -0.5f;
constexpr float VEL_TRACKING = 42.0f;
constexpr float OUT_OF_LINE_THRESHOLD = 0.007;
// 出线检测
} // namespace Config

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

    static void CalcTrack(TrackContext &ctx, PIDController &pid, float trackA = 1.0, float trackB = 1.5, float k1 = 1.0,
                          float k2 = 0.2)
    {
        auto res = MegAdcGetCalibratedResult();
        float denom = trackA * (res.l + res.r) + trackB * (res.lm + res.rm);

        ctx.trackErr = std::isnan(denom) ? 0.0f : (trackA * (res.l - res.r) + trackB * (res.lm - res.rm)) / denom;

        // constexpr float A = 1.0f, B = 0.2f;
        // k2=std::clamp(fabsf(ctx.trackErr)-0.4,0.0,0.4)*2;

        ctx.trackErr = k1 * ctx.trackErr + k2 * pow(ctx.trackErr, 3.0); // 误差整形

        // err/speed 来达成不同速度下的自适应控制效果 只减小增益不扩大增益
        ctx.trackErr = Car_GetTargetVelocity() < 20.0 ? ctx.trackErr
                                                      : ctx.trackErr * Config::VEL_TRACKING / Car_GetTargetVelocity();

        ctx.trackOutput = PID_Update_Positional(&pid, 0.0f, ctx.trackErr);
        // ctx.SetCarStatus(0.0, 0.0);
    }
};

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
    TrackingState()
    {
        PID_Init(&this->pid, PID_MODE_POSITIONAL, 13.00f, 0.0f, 0.3f, 0.0f, 0.02f); // good for v under 37
        PID_SetOutputLimit(&this->pid, -100.0f, 100.0f);
    }
    void Enter(TrackContext &) override
    {
        LED_Command(3, true);

        m_timer = 0;
    }

    TrackState Update(TrackContext &ctx, uint32_t dt) override
    {

        TrackingUtils::CalcTrack(ctx, this->pid);
        ctx.SetCarStatus(Config::VEL_TRACKING, ctx.trackOutput);

        auto res = MegAdcGetCalibratedResult();

        // 直角弯检测
        if (m_rightAngleFilter.Update(
                fabsf(res.l - res.r) <= 0.3f && fabsf(res.lm - res.rm) >= 0.3f && (res.lm > 0.6 || res.rm > 0.6), 5))
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

        // 3. 环岛检测
        if (m_ringFilter.Update(res.m >= 1.6 || m_ringFilter.count >= 5, 20))
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
    TrackingUtils::CountFilter m_rightAngleFilter;
    constexpr static float OBSTACLE_DIST_THRESHOLD = 400;
};

// 预环岛状态 通过打角让小车进入环岛 退出条件是时间到达或者中路电感不再检测到环岛入口
class PreRingState : public TrackStateBase
{
  public:
    PreRingState()
    {
        PID_Init(&this->pid, PID_MODE_POSITIONAL, 13.50f, 0.0f, 0.1f, 0.0f, 0.02f); // good for v under 37
        PID_SetOutputLimit(&this->pid, -100.0f, 100.0f);
    }
    void Enter(TrackContext &ctx) override
    {
        LED_Command(4, true);

        ctx.enterAngle = IMU_GetYaw() + 15;
        m_timer = 0;
    }

    TrackState Update(TrackContext &ctx, uint32_t dt) override
    {
        this->m_timer += dt;
        TrackingUtils::CalcTrack(ctx, this->pid);
        ctx.SetCarStatus(Config::VEL_TRACKING, ctx.trackOutput + 14);
        return ((m_timer += dt) >= 500) ? TRACK_STATE_RING : TRACK_STATE_PRE_RING;
    }

    const char *Name() const override
    {
        return "PRE_RING";
    }

  private:
    PIDController pid = {};
    uint32_t m_timer = 0;
};

class RingState : public TrackStateBase
{
  public:
    RingState()
    {
        PID_Init(&this->pid, PID_MODE_POSITIONAL, 13.50f, 0.0f, 0.1f, 0.0f, 0.02f); // good for v under 37
        PID_SetOutputLimit(&this->pid, -100.0f, 100.0f);
    }
    void Enter(TrackContext &ctx) override
    {
        LED_Command(2, true);
    }
    TrackState Update(TrackContext &ctx, uint32_t dt) override
    {

        // 复用循迹状态的逻辑
        TrackingUtils::CalcTrack(ctx, this->pid);
        ctx.SetCarStatus(38, ctx.trackOutput);

        // static int RingExitDetectCount = 0;
        // auto res = MegAdcGetResult();
        // if (res.m >= 2.0)
        // {
        //     RingExitDetectCount++;
        // }
        // else
        // {
        //     RingExitDetectCount -= 2;
        //     if (RingExitDetectCount < 0)
        //         RingExitDetectCount = 0;
        // }
        // if (RingExitDetectCount >= 10)
        // {
        //     RingExitDetectCount = 0;
        //     return TRACK_STATE_EXIT_RING;
        // }

        // 方法2: 角度判断
        float currentAngle = IMU_GetYaw();
        float diff = fabsf(TrackingUtils::NormalizeAngle(currentAngle - ctx.enterAngle));

        // 如果角度差小于阈值（例如 15 度），则认为已经转了一圈回到入口方向
        // 还需要加一个最小时间限制，防止刚进环岛就误判退出
        m_timer += dt;
        if (m_timer > 500 && diff < 15.0f)
        {
            m_timer = 0;
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
    uint32_t m_timer = 0;
};

// 出环岛状态 同预环岛 打角一段时间后退出
class ExitRingState : public TrackStateBase
{
  public:
    ExitRingState()
    {
        PID_Init(&this->pid, PID_MODE_POSITIONAL, 14.00f, 0.0f, 0.0f, 0.0f, 0.02f); // good for v under 37
        PID_SetOutputLimit(&this->pid, -100.0f, 100.0f);
        PID_Init(&angle_pid, PID_MODE_POSITIONAL, 1.0f, 0.0f, 0.0f, 0.0f, 0.02f);
        PID_SetOutputLimit(&angle_pid, -100.0f, 100.0f);
    }
    void Enter(TrackContext &ctx) override
    {
        LED_Command(4, true);

        m_timer = 0;
    }

    TrackState Update(TrackContext &ctx, uint32_t dt) override
    {
        this->m_timer += dt;
        // TrackingUtils::CalcTrack(ctx, this->pid);
        // ctx.SetCarStatus(Config::VEL_TRACKING, ctx.trackOutput - 20);

        // 处理角度跨越 180 度的问题
        float currentYaw = IMU_GetYaw();
        float error = TrackingUtils::NormalizeAngle(ctx.enterAngle - currentYaw);

        float angular_output = PID_Update_Positional(&angle_pid, error, 0.0f);
        ctx.SetCarStatus(38, angular_output);
        return ((m_timer += dt) >= 500) ? TRACK_STATE_TRACKING : TRACK_STATE_EXIT_RING;
        // return TRACK_STATE_EXIT_RING;
    }

    const char *Name() const override
    {
        return "EXIT_RING";
    }

  private:
    PIDController pid = {};
    PIDController angle_pid = {};

    uint32_t m_timer = 0;
};

// 直角弯状态
class RightAngleTurnState : public TrackStateBase
{
  public:
    RightAngleTurnState()
    {
        PID_Init(&this->angle_pid, PID_MODE_POSITIONAL, 1.0f, 0.0f, 0.0f, 0.0f, 0.02f);
        PID_SetOutputLimit(&this->angle_pid, -100.0f, 100.0f);
    }
    void Enter(TrackContext &ctx) override
    {
        LED_Command(4, true);
        m_timer = 0;
    }

    TrackState Update(TrackContext &ctx, uint32_t dt) override
    {
        m_timer += dt;

        // 处理角度跨越 180 度的问题
        float currentYaw = IMU_GetYaw();
        float error = TrackingUtils::NormalizeAngle(ctx.turnTargetYaw - currentYaw);

        float angular_output = PID_Update_Positional(&angle_pid, error, 0.0f);
        ctx.SetCarStatus(30, angular_output);

        // 持续一段时间后退出
        if (m_timer >= 500)
        {
            return TRACK_STATE_TRACKING;
        }
        return TRACK_STATE_RIGHT_ANGLE;
    }

    const char *Name() const override
    {
        return "RIGHT_ANGLE";
    }

  private:
    PIDController angle_pid = {};
    uint32_t m_timer = 0;
};

// 避障状态 退出条件检测时间或者误差重新归0
class ObstacleAvoidanceState : public TrackStateBase
{
  public:
    ObstacleAvoidanceState()
    {
        PID_Init(&angle_pid, PID_MODE_POSITIONAL, 1.5f, 0.0f, 0.0f, 0.0f, 0.02f);
        PID_SetOutputLimit(&angle_pid, -100.0f, 100.0f);
    }
    void Enter(TrackContext &ctx) override
    {
        // 关闭安全检测
        ctx.safetyCheckEnabled = false;
        m_timer = 0;
        this->origin_angle = IMU_GetYaw();
        this->phase = PHASE_TURN_OUT;
    }

    void Exit(TrackContext &ctx) override
    {
        // 恢复安全检测
        ctx.safetyCheckEnabled = true;
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
            targetYaw = origin_angle + 50.0f;
            if (m_timer >= 350) // 持续时间可调
            {
                phase = PHASE_PARALLEL;
                m_timer = 0;
            }
            break;

        case PHASE_PARALLEL:
            // 第二阶段 回正平行直行 (回到初始角度)
            targetYaw = origin_angle;
            if (m_timer >= 400) // 持续时间决定避障距离
            {
                phase = PHASE_RETURN;
                m_timer = 0;
            }
            break;

        case PHASE_RETURN:
            // 第三阶段：向左切回 (例如 45 度)
            targetYaw = origin_angle - 45.0f;

            // 检测是否回到线上
            {
                auto res = MegAdcGetCalibratedResult();
                // 简单的回线判断，可根据实际情况调整
                if ((res.l + res.r) > 1.0f)
                {
                    return TRACK_STATE_TRACKING;
                }
            }
            // 超时强制停车
            if (m_timer >= 1500)
            {
                return TRACK_STATE_STOP;
            }
            break;
        }

        // 计算角度误差并归一化
        float error = TrackingUtils::NormalizeAngle(targetYaw - currentYaw);

        float angular_output = PID_Update_Positional(&angle_pid, error, 0.0f);
        ctx.SetCarStatus(Config::VEL_TRACKING, angular_output);

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
    auto adc = MegAdcGetCalibratedResult();
    float sum = adc.l + adc.r + adc.lm + adc.rm;
    if (sum < Config::OUT_OF_LINE_THRESHOLD)
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
    };

    for (auto &p : params)
    {
        ParamServer_Register(&p);
    }
}