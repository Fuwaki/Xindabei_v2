#pragma once
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// 状态枚举
typedef enum {
    TRACK_STATE_STOP,      // 停止状态
    TRACK_STATE_TRACKING,  // 普通循迹状态
    TRACK_STATE_PRE_RING,  // 预环岛状态
    TRACK_STATE_RING,      // 环岛中状态
    TRACK_STATE_EXIT_RING, // 出环岛状态
    TRACK_STATE_OBSTACLE_AVOIDANCE, // 避障状态
    TRACK_STATE_COUNT      // 状态数量，用于数组大小
} TrackState;

// 状态机外部控制接口
typedef enum {
    TRACK_CMD_START,        // 开始循迹
    TRACK_CMD_STOP,         // 停止
    TRACK_CMD_RESET         // 重置状态机
} TrackCommand;

// 状态机初始化
void TrackInit(void);

// 状态机主处理函数 (由TrackTask调用)
void TrackHandler(void);

// 外部控制接口
void TrackSetCommand(TrackCommand cmd);

// 获取当前状态
TrackState TrackGetCurrentState(void);

// 获取状态名称 (用于调试)
const char* TrackGetStateName(TrackState state);

// 安全检测开关功能
void TrackSetSafetyCheckEnabled(bool enabled);
bool TrackIsSafetyCheckEnabled(void);

#ifdef __cplusplus
}
#endif
