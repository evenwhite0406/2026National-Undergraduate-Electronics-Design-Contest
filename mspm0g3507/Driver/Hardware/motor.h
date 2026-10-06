#ifndef MOTOR_H
#define MOTOR_H

/**
 * 调用方法：
 *   Motor_Init();
 *   Motor_SetSpeeds(30, 30);                 // 双轮速度闭环
 *   Motor_DriveHeading(30, 0);               // 速度30，持续保持0.0度
 *   Motor_DriveDistanceHeading(30, 780, 0);  // 行驶780计数后停车
 *   Motor_Off();
 * 所有以 _X1000 结尾的PID参数均为实际值的1000倍。
 */

#include <stdint.h>

#define MOTOR_LEFT   (0U)
#define MOTOR_RIGHT  (1U)

/*
 * 更换接线后的实测结果：原先启用交换时，逻辑左驱动物理右轮，逻辑右驱动物理左轮。
 * 因此当前不再交换通道，使MOTOR_LEFT/MOTOR_RIGHT恢复物理左右含义。
 */
extern uint8_t MOTOR_LOGICAL_SIDE_SWAP;

/* 更换接线后正占空比表现为后退，统一反转极性，使正速度仍表示向前。 */
extern int32_t MOTOR_OUTPUT_SIGN;

/* 默认编码器参数和实车速度 PID。速度 100 对应 4000 counts/s。 */
extern uint32_t MOTOR_ENCODER_LEFT_CPR;
extern uint32_t MOTOR_ENCODER_RIGHT_CPR;

/*
 * 车轮有效直径，单位微米。65000表示65.000 mm。
 * 落地距离标定后只需微调此参数，不必改运动控制代码。
 */
extern uint32_t MOTOR_WHEEL_DIAMETER_UM;

extern int32_t MOTOR_SPEED_PID_KP_X1000;
extern int32_t MOTOR_SPEED_PID_KI_X1000;
extern int32_t MOTOR_SPEED_PID_KD_X1000;

/* 位置输出为基础速度，yaw输出为左右轮差速速度。 */
extern int32_t MOTOR_POSITION_PID_KP_X1000;
extern int32_t MOTOR_POSITION_PID_KI_X1000;
extern int32_t MOTOR_POSITION_PID_KD_X1000;
extern int32_t MOTOR_YAW_PID_KP_X1000;
extern int32_t MOTOR_YAW_PID_KI_X1000;
extern int32_t MOTOR_YAW_PID_KD_X1000;

/* 以下参数需要根据实车惯量、轮胎抓地力和编码器分辨率调整。 */
extern int32_t MOTOR_POSITION_SPEED_LIMIT;
extern int32_t MOTOR_YAW_SPEED_LIMIT;
extern int32_t MOTOR_POSITION_TOLERANCE_COUNTS;
extern int32_t MOTOR_YAW_TOLERANCE_X10;
extern uint16_t MOTOR_TARGET_STABLE_SAMPLES;
/* BMI088超过该时间未成功更新，航向/位置串级控制立即停车。 */
extern uint32_t MOTOR_IMU_TIMEOUT_MS;

/** 初始化编码器、双路速度PID并使能电机，初始目标速度为0。 */
void Motor_Init(void);
void Motor_On(void);
void Motor_Off(void);

/**
 * 设置单轮闭环目标速度，范围 -100～100；正数前进，负数后退。
 * 调用后由编码器10ms中断自动完成测速和PID调速，不需要主循环刷新。
 */
void Motor_SetSpeed(uint8_t side, int16_t speed);

/** 同时设置左右轮闭环目标速度。 */
void Motor_SetSpeeds(int16_t left_speed, int16_t right_speed);

/**
 * 以指定闭环速度持续保持yaw航向行驶，不启用位置环。
 * speed范围-100～100；target_yaw_x10单位0.1度。一直运行到其他命令打断。
 */
void Motor_DriveHeading(int16_t speed, int16_t target_yaw_x10);

/** 停止持续航向行驶；等价于设置左右目标速度为0。 */
void Motor_StopHeading(void);

/**
 * 按指定闭环速度和yaw航向行驶指定距离。
 * speed范围1～100；relative_counts正数前进、负数后退；yaw单位0.1度。
 * 调用一次后控制器会在10ms中断中自动运行，到达目标后自动停车。
 */
void Motor_DriveDistanceHeading(int16_t speed,
                                int32_t relative_counts,
                                int16_t target_yaw_x10);

/**
 * 距离与编码器平均计数换算。
 * distance_mm和counts均为有符号值：正数前进，负数后退。
 * 换算使用左右CPR平均值和MOTOR_WHEEL_DIAMETER_UM。
 */
int32_t Motor_DistanceMmToCounts(int32_t distance_mm);
int32_t Motor_DistanceCmToCounts(int32_t distance_cm);
int32_t Motor_CountsToDistanceMm(int32_t counts);

/**
 * 使用实际距离单位启动位置+yaw控制。
 * 毫米接口适合精细标定；厘米接口的distance_cm为整数厘米。
 */
void Motor_DriveDistanceMmHeading(int16_t speed,
                                  int32_t distance_mm,
                                  int16_t target_yaw_x10);
void Motor_DriveDistanceCmHeading(int16_t speed,
                                  int32_t distance_cm,
                                  int16_t target_yaw_x10);

/** 兼容接口：使用默认速度30执行位置/yaw运动。 */
void Motor_MovePositionYaw(int32_t relative_counts,
                           int16_t target_yaw_x10);

/** 取消位置/yaw控制并立即停车。 */
void Motor_CancelPositionYaw(void);

/** 非零表示位置和yaw已连续稳定到达目标。 */
uint8_t Motor_IsPositionYawReached(void);

/** 修改双轮共用的PID参数，参数为千分制定点数。 */
void Motor_SetSpeedPID(int32_t kp_x1000,
                       int32_t ki_x1000,
                       int32_t kd_x1000);

/** 在线修改yaw航向环PID并清空其历史状态。 */
void Motor_SetYawPID(int32_t kp_x1000,
                     int32_t ki_x1000,
                     int32_t kd_x1000);

/** 返回目标速度、反馈速度和PID输出，便于OLED或串口观察。 */
int16_t Motor_GetTargetSpeed(uint8_t side);
int16_t Motor_GetFeedbackSpeed(uint8_t side);
int8_t Motor_GetOutputDuty(uint8_t side);

/** 编码器10ms测速完成后的内部入口，应用层无需调用。 */
void Motor_SpeedControlUpdate(int32_t left_count,
                              int32_t right_count,
                              int32_t left_counts_per_second,
                              int32_t right_counts_per_second);

/** 底层PWM占空比接口，仅供闭环控制器使用。 */
void Motor_SetDuty(uint8_t side, int8_t duty);

#endif
