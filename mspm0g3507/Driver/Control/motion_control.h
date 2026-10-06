#ifndef MOTION_CONTROL_H
#define MOTION_CONTROL_H

#include <stdbool.h>
#include <stdint.h>

/* 调用：Init -> Start；主循环Service；用SetWheelSpeeds或MoveRelativeCounts下发目标。 */
/* 旧运动控制层参数；启用本模块前还需补齐当前缺失的balance_control模块。 */
#define MOTION_COMMAND_LIMIT               (100L)
#define MOTION_POSITION_KP_X1000           (40L)
#define MOTION_POSITION_KI_X1000           (0L)
#define MOTION_POSITION_KD_X1000           (120L)
#define MOTION_POSITION_SPEED_LIMIT        (30L)
#define MOTION_POSITION_TOLERANCE_COUNTS   (5L)
#define MOTION_POSITION_STABLE_SAMPLES     (5U)

typedef struct {
    int16_t left_command;
    int16_t right_command;
    int32_t left_target_cps;
    int32_t right_target_cps;
    int32_t left_feedback_cps;
    int32_t right_feedback_cps;
    int32_t left_position_count;
    int32_t right_position_count;
    int32_t left_position_target;
    int32_t right_position_target;
    int16_t target_yaw_x10;
    int16_t feedback_yaw_x10;
    int16_t angle_correction;
    int8_t left_duty;
    int8_t right_duty;
    bool angle_loop_active;
    bool position_loop_active;
    bool position_reached;
    bool running;
} MotionControlData;

/** 初始化位置兼容层和速度、前后角度、转向三环控制器。 */
void MotionControl_Init(void);

/** 使能电机闭环，初始目标速度为零。 */
void MotionControl_Start(void);

/**
 * 设置左右轮速度指令，范围-100～100。
 * 指令不是PWM；每单位表示每个10 ms采样周期1个编码器计数，
 * 因此当前严格对应100个编码器计数/秒。
 */
void MotionControl_SetWheelSpeeds(int16_t left_speed,
                                  int16_t right_speed);

/** 设置左右编码器绝对目标计数，max_speed限制位置环输出速度。 */
void MotionControl_MoveToCounts(int32_t left_target_count,
                                int32_t right_target_count,
                                int16_t max_speed);

/** 以当前位置为基准设置左右轮相对位移，单位为编码器计数。 */
void MotionControl_MoveRelativeCounts(int32_t left_delta_count,
                                      int32_t right_delta_count,
                                      int16_t max_speed);

/** 退出位置模式并保持当前车轮为零速度。 */
void MotionControl_CancelPosition(void);

bool MotionControl_IsPositionReached(void);

/** 主循环服务函数，在编码器产生新的10ms采样后更新位置外环。 */
void MotionControl_Service(void);

/** 清除PID并立即关闭两路电机。 */
void MotionControl_Stop(void);

MotionControlData MotionControl_GetData(void);

#endif
