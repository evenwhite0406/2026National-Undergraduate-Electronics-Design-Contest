/**
 * @file line_follow.c
 * @brief 八路数字灰度循迹、圆弧PD跟踪与直角yaw闭环转向。
 *
 * 直角识别：
 *   LINE_1~LINE_5为黑且LINE_7/8为白 -> 左直角；
 *   LINE_4~LINE_8为黑且LINE_1/2为白 -> 右直角。
 *
 * 直角动作：
 *   1. 读取当前yaw；
 *   2. 左转目标=当前yaw+90°，右转目标=当前yaw-90°；
 *   3. Motor_DriveHeading(0, target)执行原地yaw闭环；
 *   4. yaw连续稳定进入容差后关闭yaw闭环；
 *   5. 使用普通左右轮速度命令继续同方向低速寻线；
 *   6. 中间LINE_4/5连续检测到黑线后恢复普通PD循迹。
 */

#include "line_follow.h"
#include "vehicle_tuning.h"
#include "../Hardware/motor.h"
#include "../PeripheralTest/gyro_bmi088.h"
#include "../PeripheralTest/line_sensor.h"

#define LEFT_CORNER_REQUIRED_MASK      (0x1FU) /* LINE_1~LINE_5 */
#define LEFT_CORNER_FORBIDDEN_MASK     (0xC0U) /* LINE_7~LINE_8 */
#define RIGHT_CORNER_REQUIRED_MASK     (0xF8U) /* LINE_4~LINE_8 */
#define RIGHT_CORNER_FORBIDDEN_MASK    (0x03U) /* LINE_1~LINE_2 */
#define CENTER_SENSOR_MASK             (0x18U) /* LINE_4~LINE_5 */
#define LINE_SENSOR_ALL_MASK           (0xFFU) /* LINE_1~LINE_8 configured */
#define START_STOP_LINE_MASK           (0xFFU) /* LINE_1~LINE_8 */
#define START_LINE_DETECT_SAMPLES      (3U)
#define START_LINE_RELEASE_SAMPLES     (5U)
#define SMALL_CORRECTION_BAND          (6)
#define SMALL_CORRECTION_INTERVAL_MS   (50U)

typedef enum {
    CORNER_PHASE_NONE = 0,
    CORNER_PHASE_YAW_LEFT,
    CORNER_PHASE_YAW_RIGHT,
    CORNER_PHASE_FIND_LEFT,
    CORNER_PHASE_FIND_RIGHT
} CornerPhase;

static LineFollowData g_line;
static uint32_t g_last_update_ms;
static uint32_t g_lost_since_ms;
static int16_t g_last_error_x10;
static int16_t g_derivative_x10;
static int16_t s_current_base_speed;
static int16_t s_target_base_speed;
static int16_t s_current_correction;
static int16_t s_cruise_speed_limit;
static int16_t s_base_speed_override;
static uint32_t s_last_base_ramp_ms;
static uint32_t s_last_correction_ramp_ms;
static uint32_t s_line_follow_start_ms;
static uint32_t s_start_line_ignore_ms;
static uint32_t s_base_ramp_interval_ms;
static uint8_t s_smooth_stop_active;
static uint8_t s_smooth_stop_complete;

/*
 * A点启停线状态：事件只锁存，不在循迹层直接停车。
 */
static uint8_t s_start_line_armed;
static uint8_t s_start_line_event;
static uint8_t s_start_line_detect_count;
static uint8_t s_start_line_release_count;
static uint8_t s_start_line_detection_enabled;

static CornerPhase g_corner_phase;
static uint32_t g_corner_phase_start_ms;
static uint32_t g_corner_cooldown_until_ms;
static uint16_t g_corner_yaw_stable_count;
static uint16_t g_corner_find_confirm_count;

static int16_t Limit16(int16_t value, int16_t minimum, int16_t maximum)
{
    if (value > maximum) return maximum;
    if (value < minimum) return minimum;
    return value;
}

static int16_t GetBaseSpeedTarget(void)
{
    if (s_base_speed_override > 0) {
        return Limit16(s_base_speed_override, 0, LINE_MAX_SPEED);
    }

    return Limit16(LINE_BASE_SPEED, 0, LINE_STRAIGHT_SPEED);
}

static int16_t Abs16(int16_t value)
{
    return (value < 0) ? (int16_t) -value : value;
}

static int16_t Ramp16(int16_t current, int16_t target, int16_t step)
{
    if (step < 1) step = 1;
    if (target > (int16_t) (current + step)) return (int16_t) (current + step);
    if (target < (int16_t) (current - step)) return (int16_t) (current - step);
    return target;
}

static int16_t RampCorrection(int16_t current, int16_t target,
                              uint32_t now_ms)
{
    int16_t step = LINE_CORRECTION_RAMP_STEP;
    int16_t difference = Abs16((int16_t) (target - current));
    int16_t result;

    /*
     * A one-sample mask change must not drive the correction through zero
     * at the normal attack rate. Release the old arc first, then build the
     * correction in the new direction on a later control tick.
     */
    if (((current > 0) && (target < 0)) ||
        ((current < 0) && (target > 0))) {
        target = 0;
        step = LINE_CORRECTION_REVERSE_RAMP_STEP;
    } else if ((((current > 0) && (target > 0)) ||
                ((current < 0) && (target < 0))) &&
               (difference <= SMALL_CORRECTION_BAND)) {
        /*
         * Adjacent digital patterns on one arc commonly move the requested
         * correction by 4~5.  Apply these small same-direction changes at a
         * slower cadence so the inside wheel cannot step down and back up
         * every time channel 5 or 7 flickers.  A large error still uses the
         * original fast correction ramp.
         */
        if ((uint32_t) (now_ms - s_last_correction_ramp_ms) <
            SMALL_CORRECTION_INTERVAL_MS) {
            return current;
        }
        step = 1;
    }

    result = Ramp16(current, target, step);
    if (result != current) {
        s_last_correction_ramp_ms = now_ms;
    }
    return result;
}

static int16_t WrapYawX10(int32_t angle_x10)
{
    while (angle_x10 > 1800L) angle_x10 -= 3600L;
    while (angle_x10 < -1800L) angle_x10 += 3600L;
    return (int16_t) angle_x10;
}

static int16_t YawErrorX10(int16_t target_x10, int16_t current_x10)
{
    return WrapYawX10((int32_t) target_x10 - (int32_t) current_x10);
}

static uint8_t IsStartStopLineRaw(uint8_t active_mask)
{
    if (LineSensor_GetConfiguredMask() != LINE_SENSOR_ALL_MASK) {
        return 0U;
    }

    /*
     * Stop-line rule: all eight channels must be active in the same
     * real-time sample. The filtered mask is intentionally not used.
     */
    return (active_mask == START_STOP_LINE_MASK) ? 1U : 0U;
}

static uint8_t IsStartLineIgnoreActive(uint32_t now_ms)
{
    return ((uint32_t) (now_ms - s_line_follow_start_ms) <
            s_start_line_ignore_ms)
        ? 1U
        : 0U;
}

static void UpdateStartLineDetector(uint8_t mask, uint32_t now_ms)
{
    uint8_t start_line_active = IsStartStopLineRaw(mask);

    if (IsStartLineIgnoreActive(now_ms) != 0U) {
        s_start_line_armed = 0U;
        s_start_line_event = 0U;
        s_start_line_detect_count = 0U;
        s_start_line_release_count = 0U;
        return;
    }

    if (s_start_line_armed == 0U) {
        s_start_line_detect_count = 0U;

        if (start_line_active == 0U) {
            if (s_start_line_release_count <
                START_LINE_RELEASE_SAMPLES) {
                s_start_line_release_count++;
            }

            if (s_start_line_release_count >=
                START_LINE_RELEASE_SAMPLES) {
                s_start_line_armed = 1U;
            }
        } else {
            s_start_line_release_count = 0U;
        }
        return;
    }

    if (start_line_active != 0U) {
        if (s_start_line_detect_count <
            START_LINE_DETECT_SAMPLES) {
            s_start_line_detect_count++;
        }

        if (s_start_line_detect_count >=
            START_LINE_DETECT_SAMPLES) {
            s_start_line_event = 1U;
            s_start_line_armed = 0U;
            s_start_line_detect_count = 0U;
            s_start_line_release_count = 0U;
        }
    } else {
        s_start_line_detect_count = 0U;
    }
}

void LineFollow_StartLineReset(void)
{
    s_start_line_armed = 0U;
    s_start_line_event = 0U;
    s_start_line_detect_count = 0U;
    s_start_line_release_count = 0U;
}

uint8_t LineFollow_StartLineIsArmed(void)
{
    return s_start_line_armed;
}

uint8_t LineFollow_StartLineConsumeEvent(void)
{
    uint8_t event = s_start_line_event;

    s_start_line_event = 0U;
    return event;
}

static int16_t PositionX10(uint8_t mask, uint8_t count)
{
    /*
     * The installed sensor numbering is opposite to the logical steering
     * sign used by the existing left/right motor mixer.  Keep the motor
     * direction unchanged and map LINE_1..LINE_8 into that steering frame.
     */
    static const int16_t weight_x10[LINE_SENSOR_COUNT] = {
        350, 240, 140, 50, -50, -140, -240, -350
    };
    int16_t sum = 0;
    uint8_t i;

    if (count == 0U) return 0;

    for (i = 0U; i < LINE_SENSOR_COUNT; i++) {
        if ((mask & (uint8_t) (1U << i)) != 0U) {
            sum = (int16_t) (sum + weight_x10[i]);
        }
    }

    return (int16_t) (sum / (int16_t) count);
}

static void Output(int16_t left, int16_t right, int16_t slew)
{
    left = Limit16(left, -LINE_MAX_SPEED, LINE_MAX_SPEED);
    right = Limit16(right, -LINE_MAX_SPEED, LINE_MAX_SPEED);

    g_line.left_speed = Ramp16(g_line.left_speed, left, slew);
    g_line.right_speed = Ramp16(g_line.right_speed, right, slew);

    Motor_SetSpeeds(g_line.left_speed, g_line.right_speed);
}

static void HandleStartStopLineStraight(uint32_t now_ms)
{
    int16_t straight_target = GetBaseSpeedTarget();

    if (straight_target > s_cruise_speed_limit) {
        straight_target = s_cruise_speed_limit;
    }

    straight_target = Limit16(straight_target, 0, LINE_MAX_SPEED);
    s_target_base_speed = straight_target;

    if ((uint32_t) (now_ms - s_last_base_ramp_ms) >=
        s_base_ramp_interval_ms) {
        s_current_base_speed = Ramp16(s_current_base_speed,
                                      s_target_base_speed,
                                      LINE_BASE_RAMP_STEP);
        s_last_base_ramp_ms = now_ms;
    }

    s_current_correction = Ramp16(s_current_correction, 0,
                                  LINE_CORRECTION_RAMP_STEP);
    g_line.error_x10 = 0;
    g_derivative_x10 = 0;
    g_line.state = LINE_FOLLOW_STATE_INTERSECTION;

    Output(s_current_base_speed, s_current_base_speed, LINE_RAMP_NORMAL);
}

static void HandleSmoothStop(uint32_t now_ms)
{
    if ((uint32_t) (now_ms - s_last_base_ramp_ms) >=
        s_base_ramp_interval_ms) {
        s_current_base_speed = Ramp16(s_current_base_speed, 0,
                                      LINE_BASE_RAMP_STEP);
        s_last_base_ramp_ms = now_ms;
    }

    s_current_correction = Ramp16(s_current_correction, 0,
                                  LINE_CORRECTION_RAMP_STEP);
    g_line.error_x10 = 0;
    g_derivative_x10 = 0;

    Output(s_current_base_speed, s_current_base_speed, LINE_RAMP_NORMAL);

    if ((s_current_base_speed == 0) &&
        (g_line.left_speed == 0) &&
        (g_line.right_speed == 0)) {
        Motor_SetSpeeds(0, 0);
        Motor_Off();
        g_line.state = LINE_FOLLOW_STATE_STOPPED;
        s_smooth_stop_active = 0U;
        s_smooth_stop_complete = 1U;
    }
}

static uint8_t IsLeftCorner(uint8_t mask)
{
    return ((((mask & LEFT_CORNER_REQUIRED_MASK) ==
              LEFT_CORNER_REQUIRED_MASK) &&
             ((mask & LEFT_CORNER_FORBIDDEN_MASK) == 0U)))
           ? 1U : 0U;
}

static uint8_t IsRightCorner(uint8_t mask)
{
    return ((((mask & RIGHT_CORNER_REQUIRED_MASK) ==
              RIGHT_CORNER_REQUIRED_MASK) &&
             ((mask & RIGHT_CORNER_FORBIDDEN_MASK) == 0U)))
           ? 1U : 0U;
}

static void ResetTrackingHistory(uint8_t mask, uint8_t count)
{
    int16_t error = PositionX10(mask, count);

    g_line.error_x10 = error;
    g_last_error_x10 = error;
    g_derivative_x10 = 0;

    if (error != 0) {
        g_line.last_valid_error_x10 = error;
    }
}

static void CornerFail(void)
{
    Motor_StopHeading();
    Motor_SetSpeeds(0, 0);

    g_corner_phase = CORNER_PHASE_NONE;
    g_corner_yaw_stable_count = 0U;
    g_corner_find_confirm_count = 0U;

    g_line.left_speed = 0;
    g_line.right_speed = 0;
    g_line.state = LINE_FOLLOW_STATE_FAILSAFE;
}

static uint8_t StartCorner(uint8_t turn_left, uint32_t now_ms)
{
    int16_t yaw_x10;
    uint32_t yaw_update_ms;
    int32_t target;

    if (BMI088_Gyro_GetYawSnapshot(&yaw_x10, &yaw_update_ms) == 0U) {
        CornerFail();
        return 0U;
    }

    if ((uint32_t) (now_ms - yaw_update_ms) > MOTOR_IMU_TIMEOUT_MS) {
        CornerFail();
        return 0U;
    }

    target = (int32_t) yaw_x10 +
             ((turn_left != 0U) ? (int32_t) LINE_CORNER_ANGLE_X10
                                : -(int32_t) LINE_CORNER_ANGLE_X10);

    g_line.yaw_x10 = yaw_x10;
    g_line.corner_target_yaw_x10 = WrapYawX10(target);
    g_line.left_speed = 0;
    g_line.right_speed = 0;

    g_corner_phase = (turn_left != 0U)
        ? CORNER_PHASE_YAW_LEFT
        : CORNER_PHASE_YAW_RIGHT;
    g_corner_phase_start_ms = now_ms;
    g_corner_yaw_stable_count = 0U;
    g_corner_find_confirm_count = 0U;
    g_line.state = LINE_FOLLOW_STATE_TURNING;

    /*
     * 先立即撤销普通循迹留下的前进目标和PWM，降低进入直角时
     * 继续向前滑行的风险；随后只调用一次yaw闭环命令。
     */
    Motor_SetSpeeds(0, 0);

    /*
     * speed=0表示原地yaw闭环。左转yaw增加，故左转目标为当前yaw+90°；
     * 右转目标为当前yaw-90°。只调用一次，不能在每个周期重复调用。
     */
    Motor_DriveHeading(0, g_line.corner_target_yaw_x10);
    return 1U;
}

static void EnterCornerFind(uint32_t now_ms)
{
    uint8_t was_left =
        (g_corner_phase == CORNER_PHASE_YAW_LEFT) ? 1U : 0U;

    /*
     * 立刻退出yaw闭环。后续寻线只使用左右轮速度闭环，不保持角度。
     */
    Motor_StopHeading();

    g_line.left_speed = 0;
    g_line.right_speed = 0;
    g_corner_phase = (was_left != 0U)
        ? CORNER_PHASE_FIND_LEFT
        : CORNER_PHASE_FIND_RIGHT;
    g_corner_phase_start_ms = now_ms;
    g_corner_find_confirm_count = 0U;
    g_line.state = LINE_FOLLOW_STATE_FINDING;
}

static void FinishCorner(uint8_t mask, uint8_t count, uint32_t now_ms)
{
    Motor_SetSpeeds(0, 0);

    g_line.left_speed = 0;
    g_line.right_speed = 0;
    g_corner_phase = CORNER_PHASE_NONE;
    g_corner_yaw_stable_count = 0U;
    g_corner_find_confirm_count = 0U;
    g_corner_cooldown_until_ms = now_ms + LINE_CORNER_COOLDOWN_MS;
    g_line.state = LINE_FOLLOW_STATE_TRACKING;

    /*
     * 以重新找到的新线位置初始化PD历史，防止退出直角时产生微分冲击。
     */
    ResetTrackingHistory(mask, count);
}

static void HandleCornerYaw(uint32_t now_ms)
{
    int16_t yaw_x10;
    uint32_t yaw_update_ms;
    int16_t error_x10;

    if ((uint32_t) (now_ms - g_corner_phase_start_ms) >=
        LINE_CORNER_TURN_TIMEOUT_MS) {
        CornerFail();
        return;
    }

    if (BMI088_Gyro_GetYawSnapshot(&yaw_x10, &yaw_update_ms) == 0U) {
        CornerFail();
        return;
    }

    if ((uint32_t) (now_ms - yaw_update_ms) > MOTOR_IMU_TIMEOUT_MS) {
        CornerFail();
        return;
    }

    g_line.yaw_x10 = yaw_x10;
    g_line.left_speed = Motor_GetTargetSpeed(MOTOR_LEFT);
    g_line.right_speed = Motor_GetTargetSpeed(MOTOR_RIGHT);

    error_x10 = YawErrorX10(g_line.corner_target_yaw_x10, yaw_x10);

    if (Abs16(error_x10) <= LINE_CORNER_YAW_TOLERANCE_X10) {
        if (g_corner_yaw_stable_count <
            LINE_CORNER_YAW_STABLE_SAMPLES) {
            g_corner_yaw_stable_count++;
        }
    } else {
        g_corner_yaw_stable_count = 0U;
    }

    if (g_corner_yaw_stable_count >=
        LINE_CORNER_YAW_STABLE_SAMPLES) {
        EnterCornerFind(now_ms);
    }
}

static void HandleCornerFind(uint8_t mask, uint8_t count, uint32_t now_ms)
{
    uint8_t center_active =
        ((mask & CENTER_SENSOR_MASK) != 0U) ? 1U : 0U;

    if ((uint32_t) (now_ms - g_corner_phase_start_ms) >=
        LINE_CORNER_FIND_TIMEOUT_MS) {
        CornerFail();
        return;
    }

    /*
     * 此阶段明确不使用yaw闭环，只继续同方向低速原地寻线。
     */
    if (g_corner_phase == CORNER_PHASE_FIND_LEFT) {
        /*
         * 本车实测：左轮正、右轮负会使yaw增加，因此继续向左寻线。
         */
        Output(LINE_CORNER_FIND_SPEED,
               (int16_t) -LINE_CORNER_FIND_SPEED,
               LINE_RAMP_FAST);
    } else {
        /* 右轮正、左轮负会使yaw减小，因此继续向右寻线。 */
        Output((int16_t) -LINE_CORNER_FIND_SPEED,
               LINE_CORNER_FIND_SPEED,
               LINE_RAMP_FAST);
    }

    if (center_active != 0U) {
        if (g_corner_find_confirm_count <
            LINE_CORNER_FIND_CONFIRM_SAMPLES) {
            g_corner_find_confirm_count++;
        }
    } else {
        g_corner_find_confirm_count = 0U;
    }

    if (g_corner_find_confirm_count >=
        LINE_CORNER_FIND_CONFIRM_SAMPLES) {
        FinishCorner(mask, count, now_ms);
    }
}

static void HandleLost(uint32_t now_ms)
{
    uint32_t lost_ms;
    int16_t direction;

    if (g_line.state != LINE_FOLLOW_STATE_LOST) {
        g_lost_since_ms = now_ms;
    }

    g_line.state = LINE_FOLLOW_STATE_LOST;
    lost_ms = now_ms - g_lost_since_ms;

    if (lost_ms >= LINE_LOST_STOP_MS) {
        g_line.state = LINE_FOLLOW_STATE_FAILSAFE;
        Output(0, 0, LINE_RAMP_FAST);
        return;
    }

    direction = (g_line.last_valid_error_x10 < 0) ? -1 : 1;

    if (lost_ms < LINE_LOST_COAST_MS) {
        int16_t correction =
            (int16_t) ((direction * LINE_LOST_TURN_SPEED) / 2);

        Output((int16_t) (LINE_LOST_FORWARD_SPEED + correction),
               (int16_t) (LINE_LOST_FORWARD_SPEED - correction),
               LINE_RAMP_FAST);
    } else {
        Output((int16_t) (direction * LINE_LOST_TURN_SPEED),
               (int16_t) (-direction * LINE_LOST_TURN_SPEED),
               LINE_RAMP_FAST);
    }
}

static void Track(uint8_t mask, uint8_t count, uint32_t now_ms)
{
    int16_t raw_error;
    int16_t error_delta;
    int16_t p_correction;
    int16_t d_correction;
    int16_t target_correction;
    int16_t base_speed_target;
    int16_t abs_error;
    int16_t error_gap;
    int32_t alpha;

    raw_error = PositionX10(mask, count);
    g_line.raw_error_x10 = raw_error;
    error_gap = Abs16((int16_t) (raw_error - g_line.error_x10));

    /*
     * A 5/6 or 6/7 transition changes the digital centroid by about 45~50.
     * Treat both sides identically: small adjacent-channel chatter uses the
     * stronger smoothing coefficient, while a real large displacement uses
     * the normal faster response so recovery from an outward drift is not
     * delayed.
     */
    alpha = (error_gap < LINE_ERROR_RECOVERY_GAP_X10)
        ? g_line_filter_alpha_fast
        : g_line_filter_alpha_normal;

    if (alpha < 1L) alpha = 1L;
    if (alpha > 10L) alpha = 10L;

    /*
     * 普通区域使用较强平滑，边缘或多路压线时提高当前误差权重。
     */
    /* Lower edge/wide alpha smooths adjacent digital patterns on an arc. */
    g_line.error_x10 = (int16_t) (
        ((int32_t) g_line.error_x10 * (10L - alpha) +
         (int32_t) raw_error * alpha) / 10L);

    error_delta = (int16_t) (g_line.error_x10 - g_last_error_x10);

    g_derivative_x10 = (int16_t) (
        ((int32_t) g_derivative_x10 * 4L +
         (int32_t) error_delta) / 5L);
    g_derivative_x10 = Limit16(
        g_derivative_x10,
        (int16_t) -LINE_DERIVATIVE_LIMIT_X10,
        LINE_DERIVATIVE_LIMIT_X10);
    g_line.derivative_x10 = g_derivative_x10;

    g_last_error_x10 = g_line.error_x10;

    if (g_line.error_x10 != 0) {
        g_line.last_valid_error_x10 = g_line.error_x10;
    }

    abs_error = Abs16(g_line.error_x10);

    base_speed_target = GetBaseSpeedTarget();
    s_target_base_speed = (int16_t) (
        base_speed_target -
        ((int32_t) abs_error * LINE_CURVE_SLOWDOWN_X100) / 1000L);

    s_target_base_speed = Limit16(s_target_base_speed,
                                  LINE_MIN_SPEED,
                                  base_speed_target);
    if (s_target_base_speed > s_cruise_speed_limit) {
        s_target_base_speed = s_cruise_speed_limit;
    }

    p_correction = (int16_t) (
        ((int32_t) LINE_KP_X100 * g_line.error_x10) / 1000L);
    d_correction = (int16_t) (
        ((int32_t) LINE_KD_X100 * g_derivative_x10) / 1000L);

    /* D may damp a turn, but must not reverse it before error crosses zero. */
    if ((p_correction > 0) &&
        (d_correction < (int16_t) -p_correction)) {
        d_correction = (int16_t) -p_correction;
    } else if ((p_correction < 0) &&
               (d_correction > (int16_t) -p_correction)) {
        d_correction = (int16_t) -p_correction;
    }

    target_correction = (int16_t) (p_correction + d_correction);

    target_correction = Limit16(target_correction,
                                (int16_t) -LINE_MAX_CORRECTION,
                                LINE_MAX_CORRECTION);
    g_line.p_correction = p_correction;
    g_line.d_correction = d_correction;
    g_line.target_correction = target_correction;

    /*
     * Keep one continuous steering path for straight lines and arcs.  The
     * previous ARC latch imposed a minimum correction at entry, abruptly
     * slowing the inside wheel.  PD now always reaches the symmetric wheel
     * mixer through the same correction slew limiter.
     */
    s_current_correction = RampCorrection(s_current_correction,
                                          target_correction,
                                          now_ms);

    if ((uint32_t) (now_ms - s_last_base_ramp_ms) >=
        s_base_ramp_interval_ms) {
        s_current_base_speed = Ramp16(s_current_base_speed,
                                      s_target_base_speed,
                                      LINE_BASE_RAMP_STEP);
        s_last_base_ramp_ms = now_ms;
    }

    g_line.state = LINE_FOLLOW_STATE_TRACKING;

    Output((int16_t) (s_current_base_speed + s_current_correction),
           (int16_t) (s_current_base_speed - s_current_correction),
           LINE_RAMP_NORMAL);
}

void LineFollow_Init(void)
{
    g_line = (LineFollowData) {0};
    g_line.state = LINE_FOLLOW_STATE_STOPPED;

    g_last_update_ms = 0U;
    g_lost_since_ms = 0U;
    g_last_error_x10 = 0;
    g_derivative_x10 = 0;
    s_current_base_speed = 0;
    s_target_base_speed = LINE_BASE_SPEED;
    s_current_correction = 0;
    s_cruise_speed_limit = LINE_STRAIGHT_SPEED;
    s_base_speed_override = 0;
    s_last_base_ramp_ms = 0U;
    s_last_correction_ramp_ms = 0U;
    s_line_follow_start_ms = 0U;
    s_start_line_ignore_ms = START_LINE_IGNORE_MS;
    s_base_ramp_interval_ms = LINE_BASE_RAMP_INTERVAL_MS;
    s_smooth_stop_active = 0U;
    s_smooth_stop_complete = 0U;
    s_start_line_detection_enabled = 1U;
    LineFollow_StartLineReset();

    g_corner_phase = CORNER_PHASE_NONE;
    g_corner_phase_start_ms = 0U;
    g_corner_cooldown_until_ms = 0U;
    g_corner_yaw_stable_count = 0U;
    g_corner_find_confirm_count = 0U;

    LineSensor_FilterReset();
}

void LineFollow_Start(uint32_t now_ms)
{
    LineFollow_Init();

    g_last_update_ms = now_ms - LINE_FOLLOW_PERIOD_MS;
    s_last_base_ramp_ms = now_ms;
    s_last_correction_ramp_ms = now_ms;
    s_line_follow_start_ms = now_ms;
    s_smooth_stop_active = 0U;
    s_smooth_stop_complete = 0U;
    g_line.state = LINE_FOLLOW_STATE_TRACKING;

    Motor_StopHeading();
    Motor_On();
    Motor_SetSpeeds(0, 0);
}

void LineFollow_Update(uint32_t now_ms)
{
    uint8_t count;

    if ((g_line.state == LINE_FOLLOW_STATE_STOPPED) ||
        (g_line.state == LINE_FOLLOW_STATE_FAILSAFE) ||
        ((uint32_t) (now_ms - g_last_update_ms) <
         LINE_FOLLOW_PERIOD_MS)) {
        return;
    }

    g_last_update_ms = now_ms;
    g_line.raw_mask = LineSensor_ReadActiveMask();
    g_line.filtered_mask = LineSensor_FilterSample(g_line.raw_mask);
    count = LineSensor_CountActive(g_line.filtered_mask);
    g_line.active_count = count;

    if (s_start_line_detection_enabled != 0U) {
        UpdateStartLineDetector(g_line.raw_mask, now_ms);
    }

    if (s_smooth_stop_active != 0U) {
        HandleSmoothStop(now_ms);
        return;
    }

    /*
     * Crossing the wide A line must always cancel steering correction and
     * command equal wheel speeds.  Event generation is optional per task,
     * but straight-through handling is a line-follow safety behaviour.
     */
    if (IsStartStopLineRaw(g_line.raw_mask) != 0U) {
        HandleStartStopLineStraight(now_ms);
        return;
    }

    if (count == 0U) {
        HandleLost(now_ms);
    } else {
        if (g_line.state == LINE_FOLLOW_STATE_LOST) {
            g_derivative_x10 = 0;
            g_last_error_x10 = g_line.last_valid_error_x10;
        }

        Track(g_line.filtered_mask, count, now_ms);
    }
}

void LineFollow_Stop(void)
{
    Motor_StopHeading();
    Motor_SetSpeeds(0, 0);
    Motor_Off();

    g_corner_phase = CORNER_PHASE_NONE;
    g_corner_yaw_stable_count = 0U;
    g_corner_find_confirm_count = 0U;

    g_line.state = LINE_FOLLOW_STATE_STOPPED;
    g_line.left_speed = 0;
    g_line.right_speed = 0;
    s_current_base_speed = 0;
    s_target_base_speed = 0;
    s_current_correction = 0;
    s_base_speed_override = 0;
    s_smooth_stop_active = 0U;
    s_smooth_stop_complete = 0U;
}

void LineFollow_SetCruiseSpeedLimit(int16_t speed_limit)
{
    s_cruise_speed_limit = Limit16(speed_limit, 0, LINE_MAX_SPEED);
}

void LineFollow_SetBaseSpeedOverride(int16_t base_speed)
{
    s_base_speed_override = Limit16(base_speed, 0, LINE_MAX_SPEED);
}

void LineFollow_SetStartLineIgnoreMs(uint32_t ignore_ms)
{
    s_start_line_ignore_ms = ignore_ms;
}

void LineFollow_SetStartLineDetectionEnabled(uint8_t enabled)
{
    s_start_line_detection_enabled = (enabled != 0U) ? 1U : 0U;
    if (s_start_line_detection_enabled == 0U) {
        LineFollow_StartLineReset();
    }
}

void LineFollow_SetBaseRampIntervalMs(uint32_t interval_ms)
{
    if (interval_ms < 1U) {
        interval_ms = 1U;
    }

    s_base_ramp_interval_ms = interval_ms;
}

void LineFollow_RequestSmoothStop(void)
{
    if ((g_line.state == LINE_FOLLOW_STATE_STOPPED) ||
        (g_line.state == LINE_FOLLOW_STATE_FAILSAFE)) {
        return;
    }

    s_smooth_stop_active = 1U;
    s_smooth_stop_complete = 0U;
    s_target_base_speed = 0;
}

uint8_t LineFollow_IsSmoothStopComplete(void)
{
    return s_smooth_stop_complete;
}

LineFollowState LineFollow_GetState(void)
{
    return g_line.state;
}

LineFollowData LineFollow_GetData(void)
{
    /* Retained in LineFollowData for Task 8 telemetry compatibility. */
    g_line.arc_active = 0U;
    g_line.arc_direction = 0;
    g_line.current_correction = s_current_correction;
    g_line.current_base_speed = s_current_base_speed;
    g_line.target_base_speed = s_target_base_speed;
    return g_line;
}

const char *LineFollow_GetStatusText(void)
{
    switch (g_line.state) {
        case LINE_FOLLOW_STATE_TRACKING:
            return "TRACKING";

        case LINE_FOLLOW_STATE_TURNING:
            return (g_corner_phase == CORNER_PHASE_YAW_LEFT)
                ? "YAW LEFT +90"
                : "YAW RIGHT -90";

        case LINE_FOLLOW_STATE_FINDING:
            return (g_corner_phase == CORNER_PHASE_FIND_LEFT)
                ? "FIND LINE LEFT"
                : "FIND LINE RIGHT";

        case LINE_FOLLOW_STATE_INTERSECTION:
            return "INTERSECTION";

        case LINE_FOLLOW_STATE_LOST:
            return "SEARCH LINE";

        case LINE_FOLLOW_STATE_FAILSAFE:
            return "LINE FAILSAFE";

        default:
            return "STOPPED";
    }
}
