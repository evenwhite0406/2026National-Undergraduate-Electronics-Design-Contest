#ifndef VEHICLE_TUNING_H
#define VEHICLE_TUNING_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 八路数字灰度循迹参数。 */
extern uint8_t LINE_SENSOR_ACTIVE_LOW;
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
extern uint32_t START_LINE_IGNORE_MS;

extern int32_t LINE_KP_X100;
extern int32_t LINE_KD_X100;

extern int16_t LINE_RAMP_NORMAL;
extern int16_t LINE_RAMP_FAST;
extern int16_t LINE_BASE_RAMP_STEP;
extern uint32_t LINE_BASE_RAMP_INTERVAL_MS;
extern int16_t LINE_CORRECTION_RAMP_STEP;
extern int16_t LINE_CORRECTION_REVERSE_RAMP_STEP;
extern int16_t LINE_DERIVATIVE_LIMIT_X10;
extern int16_t LINE_ERROR_RECOVERY_GAP_X10;

/* Competition-track distance-based approach profile. */
extern uint32_t TRACK_LAP_DISTANCE_CM;
extern int16_t TASK4_CRUISE_SPEED;
extern uint32_t TASK4_PASS_B_DISTANCE_CM;
extern uint32_t TASK4_BASE_RAMP_INTERVAL_MS;
extern uint32_t TASK4_BRAKE_START_CM;
extern uint32_t TASK4_CRAWL_START_CM;
extern int16_t TASK2_CRUISE_SPEED;
extern uint32_t TASK2_BRAKE_START_CM;
extern int32_t TASK2_POSITION_KP_X100;
extern uint32_t TASK2_STOP_YAW_DEG;
extern uint32_t TASK56_STOP_YAW_DEG;
extern int16_t TASK56_CRUISE_SPEED;
extern uint32_t TASK56_PASS_A_DISTANCE_CM;
extern int16_t APPROACH_SPEED;
extern int16_t CRAWL_SPEED;

/* Non-blocking launch profile and K230 longitudinal feedforward. */
extern uint32_t START_S_CURVE_MS;
extern uint32_t START_BALL_S_CURVE_MS;
extern uint32_t START_FF_LEAD_MS;
extern int32_t START_FF_GAIN_X100;
extern int32_t START_FF_DAMP_X100;
extern int16_t START_FF_MAX_X10;
extern int16_t START_FF_SIGN;
extern uint8_t START_FF_LINK_ENABLE;
extern uint8_t START_IMU_FORWARD_AXIS;
extern uint8_t START_IMU_FORWARD_INVERT;
extern uint32_t START_FF_LOG_MS;

/* 误差低通滤波系数，范围1～10。 */
extern int32_t g_line_filter_alpha_normal;
extern int32_t g_line_filter_alpha_fast;


/* 直角yaw闭环与脱离闭环后的寻线参数。 */
extern int16_t LINE_CORNER_ANGLE_X10;
extern int16_t LINE_CORNER_YAW_TOLERANCE_X10;
extern uint16_t LINE_CORNER_YAW_STABLE_SAMPLES;
extern uint32_t LINE_CORNER_TURN_TIMEOUT_MS;
extern int16_t LINE_CORNER_FIND_SPEED;
extern uint32_t LINE_CORNER_FIND_TIMEOUT_MS;
extern uint16_t LINE_CORNER_FIND_CONFIRM_SAMPLES;
extern uint32_t LINE_CORNER_COOLDOWN_MS;

#ifdef __cplusplus
}
#endif

#endif
