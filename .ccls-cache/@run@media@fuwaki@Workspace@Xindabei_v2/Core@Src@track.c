#include "track.h"
#include "car_control.h"
#include "log.h"
#include "meg_adc.h"
#include "pid.h"
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
    PID_Init(&track_pid, PID_MODE_POSITIONAL, 0.1, 0.0, 0.0, 0.0, 0.1);

    LOG_INFO("TrackInit done");
}
void TrackHandler()
{
    // TODO: 翻车停机
    switch (status)
    {
    case STATUS_STOP: {
        SetTargetCarStatus(0, 0);
        break;
    }

    case STATUS_TRACKING: {
        adc_result result = MegAdcGetResult();
        const float A = 1.0, B = 1.0; // 差比和差参数
        float err = (A * (result.l - result.r) + B * (result.lm - result.rm)) /
                    (A * (result.l + result.r) + B * (result.lm + result.rm));
        float output = PID_Update_Positional(&track_pid, 0, err);
        SetTargetCarStatus(BASE_VELOCITY, output);
        break;
    }
    case STATUS_PRE_RING:

    case STATUS_RING:
    case STATUS_OBSTACLE:
        break;
    };
}