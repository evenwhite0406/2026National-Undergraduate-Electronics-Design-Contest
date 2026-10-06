#ifndef LINE_FOLLOW_H
#define LINE_FOLLOW_H

#include <stdbool.h>
#include <stdint.h>

/* 调用：Init一次，Start启动，主循环持续Update，Stop立即结束。 */
extern uint32_t LINE_FOLLOW_PERIOD_MS;
extern int16_t LINE_STRAIGHT_SPEED;
extern int16_t LINE_BASE_SPEED;
extern int16_t LINE_MIN_SPEED;
extern int16_t LINE_MAX_SPEED;
extern int16_t LINE_LOST_FORWARD_SPEED;
extern int16_t LINE_LOST_TURN_SPEED;
extern int16_t LINE_INTERSECTION_SPEED;
extern int16_t LINE_MAX_CORRECTION;
extern int32_t LINE_CURVE_SLOWDOWN_X100;
extern uint32_t LINE_LOST_COAST_MS;
extern uint32_t LINE_LOST_STOP_MS;
extern int32_t LINE_KP_X100;
extern int32_t LINE_KD_X100;
extern int16_t LINE_RAMP_NORMAL;
extern int16_t LINE_RAMP_FAST;

/* 直角弯：先用yaw闭环转指定角度，再脱离yaw闭环低速寻线。 */
extern int16_t LINE_CORNER_ANGLE_X10;
extern int16_t LINE_CORNER_YAW_TOLERANCE_X10;
extern uint16_t LINE_CORNER_YAW_STABLE_SAMPLES;
extern uint32_t LINE_CORNER_TURN_TIMEOUT_MS;
extern int16_t LINE_CORNER_FIND_SPEED;
extern uint32_t LINE_CORNER_FIND_TIMEOUT_MS;
extern uint16_t LINE_CORNER_FIND_CONFIRM_SAMPLES;
extern uint32_t LINE_CORNER_COOLDOWN_MS;

typedef enum {
    LINE_FOLLOW_STATE_STOPPED = 0,
    LINE_FOLLOW_STATE_TRACKING,
    LINE_FOLLOW_STATE_TURNING,
    LINE_FOLLOW_STATE_FINDING,
    LINE_FOLLOW_STATE_LOST,
    LINE_FOLLOW_STATE_INTERSECTION,
    LINE_FOLLOW_STATE_FAILSAFE
} LineFollowState;

typedef struct {
    LineFollowState state;
    uint8_t raw_mask;
    uint8_t filtered_mask;
    uint8_t active_count;
    int16_t raw_error_x10;
    int16_t error_x10;
    int16_t derivative_x10;
    int16_t last_valid_error_x10;
    int16_t left_speed;
    int16_t right_speed;
    int16_t yaw_x10;
    int16_t corner_target_yaw_x10;
    uint8_t arc_active;
    int8_t arc_direction;
    int16_t p_correction;
    int16_t d_correction;
    int16_t target_correction;
    int16_t current_correction;
    int16_t current_base_speed;
    int16_t target_base_speed;
} LineFollowData;

/** 初始化循线控制器，不启动电机。 */
void LineFollow_Init(void);

/** 启动非阻塞循线任务。 */
void LineFollow_Start(uint32_t now_ms);

/** 推进一次循线状态机，应在主循环中持续调用。 */
void LineFollow_Update(uint32_t now_ms);

/** 立即停止循线、停止yaw闭环并关闭电机驱动。 */
void LineFollow_Stop(void);

/** Limit the tracking base speed; the existing base-speed ramp applies it. */
void LineFollow_SetCruiseSpeedLimit(int16_t speed_limit);

/** Use a task-specific base speed; zero restores LINE_BASE_SPEED. */
void LineFollow_SetBaseSpeedOverride(int16_t base_speed);

/** Override the time-only A-line ignore window for the current run. */
void LineFollow_SetStartLineIgnoreMs(uint32_t ignore_ms);

/** Enable or disable stop-line detection and its straight-through handling. */
void LineFollow_SetStartLineDetectionEnabled(uint8_t enabled);

/** Override the base-speed ramp interval for the current run. */
void LineFollow_SetBaseRampIntervalMs(uint32_t interval_ms);

/** Begin a non-blocking straight-line deceleration to zero. */
void LineFollow_RequestSmoothStop(void);

/** Return nonzero after the requested smooth stop has switched the motor off. */
uint8_t LineFollow_IsSmoothStopComplete(void);

/** 清除A点启停线的武装状态、检测计数和锁存事件。 */
void LineFollow_StartLineReset(void);

/** 返回是否已经连续离开A点启停线并完成检测武装。 */
uint8_t LineFollow_StartLineIsArmed(void);

/** 返回并清除一次锁存的A点启停线事件；无事件时返回0。 */
uint8_t LineFollow_StartLineConsumeEvent(void);

LineFollowState LineFollow_GetState(void);
LineFollowData LineFollow_GetData(void);
const char *LineFollow_GetStatusText(void);

#endif
