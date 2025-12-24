#include "track.h"
#include "common.h"
#include "fsm.hpp"
#include "led.h"
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
// 出线检测
} // namespace Config

// ============================================================================
// 循迹上下文 (业务数据)
// ============================================================================
struct TrackContext
{
    // 循迹参数
    float trackA = 1.0f;
    float trackB = 1.0f;
    float trackErr = 0.0f;
    float trackOutput = 0.0f;
    float outOfLineThreshold = 0.0005;

    // 安全检测
    uint32_t safetyTimer = 0;
    bool safetyCheckEnabled = true; // 安全检测开关，默认开启

    // 环岛变量
    float enterAngle = 0.0;
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
    static void CalcTrack(TrackContext &ctx, PIDController &pid)
    {
        auto res = MegAdcGetCalibratedResult();
        float denom = ctx.trackA * (res.l + res.r) + ctx.trackB * (res.lm + res.rm);

        ctx.trackErr =
            std::isnan(denom) ? 0.0f : (ctx.trackA * (res.l - res.r) + ctx.trackB * (res.lm - res.rm)) / denom;
        // LOG_INFO("%s\n",float_to_str(ctx.trackErr));

        constexpr float A = 1.0f, B = 0.2f;
        ctx.trackErr = A * ctx.trackErr + B * pow(ctx.trackErr, 3.0); // 误差整形

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
        PID_Init(&this->pid, PID_MODE_POSITIONAL, 13.00f, 0.0f, 0.0f, 0.0f, 0.02f); // good for v under 37
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
        // // // 2. 避障检测
        // static int TofCount = 0;
        // if (TofGetDistance() < OBSTACLE_DIST_THRESHOLD)
        // {
        //     TofCount++;
        // }
        // else
        // {
        //     TofCount -= 2;
        //     if (TofCount < 0)
        //         TofCount = 0;
        // }
        // if (TofCount >= 20)
        // {
        //     return TRACK_STATE_OBSTACLE_AVOIDANCE;
        // }

        // 3. 环岛检测
        static int RingDetectCount = 0;
        auto res = MegAdcGetCalibratedResult();
        if (res.m >= 1.6 || RingDetectCount >= 5)
        {
            RingDetectCount++;
        }
        else
        {
            RingDetectCount -= 2;
            if (RingDetectCount < 0)
                RingDetectCount = 0;
        }
        if (RingDetectCount >= 20)
        {
            RingDetectCount = 0;
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
    constexpr static float OBSTACLE_DIST_THRESHOLD = 400;
};

// 预环岛状态 通过打角让小车进入环岛 退出条件是时间到达或者中路电感不再检测到环岛入口
class PreRingState : public TrackStateBase
{
  public:
    PreRingState()
    {
        PID_Init(&this->pid, PID_MODE_POSITIONAL, 14.00f, 0.0f, 0.0f, 0.0f, 0.02f); // good for v under 37
        PID_SetOutputLimit(&this->pid, -100.0f, 100.0f);
    }
    void Enter(TrackContext &ctx) override
    {
        LED_Command(4, true);

        ctx.enterAngle = IMU_GetYaw()+15;
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
        PID_Init(&this->pid, PID_MODE_POSITIONAL, 14.00f, 0.0f, 0.0f, 0.0f, 0.02f); // good for v under 37
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
        float diff = fabsf(currentAngle - ctx.enterAngle);
        if (diff > 180.0f)
        {
            diff = 360.0f - diff;
        }

        // 如果角度差小于阈值（例如 15 度），则认为已经转了一圈回到入口方向
        // 还需要加一个最小时间限制，防止刚进环岛就误判退出
        static uint32_t ringTimer = 0;
        ringTimer += dt;

        if (ringTimer > 500 && diff < 15.0f)
        {
            ringTimer = 0;
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
        float angular_output = PID_Update_Positional(&angle_pid, ctx.enterAngle, IMU_GetYaw());
        ctx.SetCarStatus(0, angular_output);
        // return ((m_timer += dt) >= 500) ? TRACK_STATE_TRACKING : TRACK_STATE_EXIT_RING;
        return TRACK_STATE_EXIT_RING;
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
        m_timer = 0;
        this->origin_angle = IMU_GetYaw();
    }

    void Exit(TrackContext &ctx) override
    {
        // 恢复安全检测
        ctx.safetyCheckEnabled = true;
    }

    TrackState Update(TrackContext &ctx, uint32_t dt) override
    {
        if (second_phase)
        {
            // 第二阶段 左打60度 直到重回线上
            float angular_output = PID_Update_Positional(&angle_pid, this->origin_angle - 60.0, IMU_GetYaw());
            ctx.SetCarStatus(0, angular_output);
            // 判断是否重新回到线上
            // if(fabsf(ctx.trackErr)>100.0)
            {
                // 退出避障状态
                // return TRACK_STATE_STOP;
            }
        }
        else
        {
            // 第一阶段 先向右打60度 走指定路程
            float angular_output = PID_Update_Positional(&angle_pid, this->origin_angle + 60.0, IMU_GetYaw());
            ctx.SetCarStatus(Config::VEL_TRACKING, angular_output);
            this->m_timer += dt;
            // 判断是否已经走了足够的时间
            //  if (this->m_timer >=(uint32_t) (35*2*50 / Car_GetTargetVelocity()))
            if (this->m_timer >= 1000)
            {
                // 进入第二阶段
                this->second_phase = true;
                this->m_timer = 0;
            }
        }
        return TRACK_STATE_OBSTACLE_AVOIDANCE;
    }

    const char *Name() const override
    {
        return "OBSTACLE_AVOIDANCE";
    }

  private:
    uint32_t m_timer = 0;
    float origin_angle = 0;
    bool second_phase = false;
    PIDController angle_pid = {};
    constexpr static float POLYLINE_LENGTH = 0.4f;
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