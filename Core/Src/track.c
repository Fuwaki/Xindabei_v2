#include "track.h"
#include "car_control.h"
#include "log.h"
#include "meg_adc.h"
#include "param_server.h"
#include "pid.h"
#include <math.h>

static float track_A = 1.0f;
static float track_B = 0.5f;
static float track_err = 0.0f;
static float track_output = 0.0f;

static float GetTrackA(void)
{
    return track_A;
}
static void SetTrackA(float v)
{
    track_A = v;
}
static float GetTrackB(void)
{
    return track_B;
}
static void SetTrackB(float v)
{
    track_B = v;
}
static float GetTrackErr(void)
{
    return track_err;
}
static float GetTrackOutput(void)
{
    return track_output;
}

static PIDController track_pid;
enum STATUS
{
    STATUS_STOP,     // 空闲状态
    STATUS_TRACKING, // 正常循迹
    STATUS_PRE_RING, // 预环岛
    STATUS_RING,     // 环岛中
    STATUS_OBSTACLE, // 避障中 以恒定速度和角速度运行 直到重新寻上线
};
enum STATUS status = STATUS_STOP;

const float BASE_VELOCITY = 1.0; // 基础前进速度
void TrackInit()
{
    LOG_INFO("TrackInit start");
    PID_Init(&track_pid, PID_MODE_POSITIONAL, 0.85, 0.0, 0.0, 0.0, 0.1);
    PID_SetOutputLimit(&track_pid, -2.0, 2.0);

    static ParamDesc track_params[] = {
        {.name = "Track_A",
         .type = PARAM_TYPE_FLOAT,
         .ops.f.get = GetTrackA,
         .ops.f.set = SetTrackA,
         .step = 0.1f,
         .read_only = 0,
         .mask = 0},
        {.name = "Track_B",
         .type = PARAM_TYPE_FLOAT,
         .ops.f.get = GetTrackB,
         .ops.f.set = SetTrackB,
         .step = 0.1f,
         .read_only = 0,
         .mask = 0},
        {.name = "Track_Err",
         .type = PARAM_TYPE_FLOAT,
         .ops.f.get = GetTrackErr,
         .read_only = 1,
         .mask = PARAM_MASK_SERIAL},
        {.name = "Track_Out",
         .type = PARAM_TYPE_FLOAT,
         .ops.f.get = GetTrackOutput,
         .read_only = 1,
         .mask = PARAM_MASK_SERIAL},
    };

    for (int i = 0; i < (int)(sizeof(track_params) / sizeof(track_params[0])); ++i)
    {
        ParamServer_Register(&track_params[i]);
    }

    LOG_INFO("TrackInit done");
}
void TrackHandler()
{
    // // TODO: 翻车停机
    // switch (status)
    // {
    // case STATUS_STOP: {
    //     SetTargetCarStatus(0, 0);
    //     break;
    // }

    // case STATUS_TRACKING: {
    adc_result result = MegAdcGetResult();
    track_err = (track_A * (result.l - result.r) + track_B * (result.lm - result.rm)) /
                (track_A * (result.l + result.r) + track_B * (result.lm + result.rm));
    
    if (isnan(track_err)) {
        track_err = 0.0f;
    }
    track_output = PID_Update_Positional(&track_pid, 0.0, track_err);

    SetTargetCarStatus(3.0, track_output);             //正为右
    //     break;
    // }
    // case STATUS_PRE_RING:

    // case STATUS_RING:
    // case STATUS_OBSTACLE:
    //     break;
    // };
}