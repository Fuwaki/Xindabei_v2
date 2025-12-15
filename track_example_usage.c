// 这是一个示例文件，展示如何使用新的状态机接口
// 在实际项目中，这些代码可以放在其他需要控制小车状态的文件中

#include "track.h"
#include "log.h"
#include <string.h>

// 示例：通过外部命令控制状态机
void ExampleStartTracking(void)
{
    LOG_INFO("Starting tracking mode");
    TrackSetCommand(TRACK_CMD_START);
}

void ExampleStopCar(void)
{
    LOG_INFO("Stopping car");
    TrackSetCommand(TRACK_CMD_STOP);
}

void ExampleResetStateMachine(void)
{
    LOG_INFO("Resetting state machine");
    TrackSetCommand(TRACK_CMD_RESET);
}

// 示例：检查当前状态
void ExampleCheckCurrentState(void)
{
    TrackState current = TrackGetCurrentState();
    const char* state_name = TrackGetStateName(current);
    LOG_INFO("Current state: %s", state_name);
}

// 示例：在按键处理中使用状态机
void ExampleKeyHandler(int key)
{
    switch (key) {
        case 1: // 按键1：开始循迹
            TrackSetCommand(TRACK_CMD_START);
            break;
        case 2: // 按键2：停止
            TrackSetCommand(TRACK_CMD_STOP);
            break;
        default:
            break;
    }
}

// 示例：在UART命令处理中使用状态机
void ExampleUartCommandHandler(const char* cmd)
{
    if (strcmp(cmd, "start") == 0) {
        TrackSetCommand(TRACK_CMD_START);
    } else if (strcmp(cmd, "stop") == 0) {
        TrackSetCommand(TRACK_CMD_STOP);
    } else if (strcmp(cmd, "reset") == 0) {
        TrackSetCommand(TRACK_CMD_RESET);
    } else if (strcmp(cmd, "status") == 0) {
        TrackState current = TrackGetCurrentState();
        const char* state_name = TrackGetStateName(current);
        LOG_INFO("Car status: %s", state_name);
    }
}