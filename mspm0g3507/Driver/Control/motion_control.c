/**
 * @file motion_control.c
 * @brief 对现有循迹/视觉任务提供兼容接口，并把指令交给自平衡三环控制器。
 *
 * 左右轮速度指令在这里拆分为车体平均速度和目标差速：
 *   target_speed = (left + right) / 2
 *   target_turn  = right - left
 * 真正的速度PI、前后角度PD、转向反馈和PWM混合均由balance_control.c完成。
 */

#include "motion_control.h"
#include "balance_control.h"
#include "pid.h"
#include "../Hardware/encoder.h"
#include "ti_msp_dl_config.h"

#define MOTION_SPEED_UNIT_CPS              \
    (ENCODER_SPEED_100_PERCENT_CPS / 100L)
static PIDController g_left_position_pid;
static PIDController g_right_position_pid;
static volatile int32_t g_sample_left_count;
static volatile int32_t g_sample_right_count;
static volatile uint8_t g_sample_ready;
static volatile uint8_t g_initialized;
static MotionControlData g_control;
static int16_t g_position_max_speed;
static uint8_t g_position_stable_count;

static int32_t MotionControl_Limit(int32_t value,
                                   int32_t minimum,
                                   int32_t maximum)
{
    if (value > maximum) return maximum;
    if (value < minimum) return minimum;
    return value;
}

static int32_t MotionControl_Abs32(int32_t value)
{
    return (value < 0L) ? -value : value;
}

static int16_t MotionControl_RoundFloat(float value)
{
    if (value > 32767.0f) return 32767;
    if (value < -32768.0f) return -32768;
    return (int16_t) ((value >= 0.0f) ? (value + 0.5f) : (value - 0.5f));
}

static void MotionControl_ApplyWheelCommands(void)
{
    float left_cps = (float) g_control.left_command *
                     (float) MOTION_SPEED_UNIT_CPS;
    float right_cps = (float) g_control.right_command *
                      (float) MOTION_SPEED_UNIT_CPS;

    g_control.left_target_cps = (int32_t) left_cps;
    g_control.right_target_cps = (int32_t) right_cps;
    BalanceControl_SetTargetSpeed((left_cps + right_cps) * 0.5f);
    BalanceControl_SetTargetTurn(right_cps - left_cps);
}

static void MotionControl_RefreshFeedback(void)
{
    BalanceControlState balance = BalanceControl_GetState();

    g_control.left_feedback_cps = (int32_t) balance.left_speed;
    g_control.right_feedback_cps = (int32_t) balance.right_speed;
    g_control.angle_correction =
        MotionControl_RoundFloat(balance.differential_pwm);
    g_control.left_duty = (int8_t) MotionControl_RoundFloat(balance.left_pwm);
    g_control.right_duty = (int8_t) MotionControl_RoundFloat(balance.right_pwm);
    g_control.angle_loop_active = balance.enabled;
}

void MotionControl_Init(void)
{
    PID_Init(&g_left_position_pid,
             MOTION_POSITION_KP_X1000,
             MOTION_POSITION_KI_X1000,
             MOTION_POSITION_KD_X1000,
             -MOTION_POSITION_SPEED_LIMIT,
             MOTION_POSITION_SPEED_LIMIT);
    PID_Init(&g_right_position_pid,
             MOTION_POSITION_KP_X1000,
             MOTION_POSITION_KI_X1000,
             MOTION_POSITION_KD_X1000,
             -MOTION_POSITION_SPEED_LIMIT,
             MOTION_POSITION_SPEED_LIMIT);
    PID_SetIntegralLimits(&g_left_position_pid, -10L, 10L);
    PID_SetIntegralLimits(&g_right_position_pid, -10L, 10L);

    BalanceControl_Init();
    g_control = (MotionControlData) {0};
    g_sample_left_count = 0L;
    g_sample_right_count = 0L;
    g_sample_ready = 0U;
    g_position_max_speed = (int16_t) MOTION_POSITION_SPEED_LIMIT;
    g_position_stable_count = 0U;
    g_initialized = 1U;
}

void MotionControl_Start(void)
{
    PID_Reset(&g_left_position_pid);
    PID_Reset(&g_right_position_pid);
    g_control.left_command = 0;
    g_control.right_command = 0;
    g_control.position_loop_active = false;
    g_control.position_reached = false;
    g_control.running = true;
    g_position_stable_count = 0U;
    BalanceControl_Enable();
    MotionControl_ApplyWheelCommands();
}

void MotionControl_SetWheelSpeeds(int16_t left_speed,
                                  int16_t right_speed)
{
    g_control.position_loop_active = false;
    g_control.position_reached = false;
    g_position_stable_count = 0U;
    PID_Reset(&g_left_position_pid);
    PID_Reset(&g_right_position_pid);
    g_control.left_command = (int16_t) MotionControl_Limit(
        left_speed, -MOTION_COMMAND_LIMIT, MOTION_COMMAND_LIMIT);
    g_control.right_command = (int16_t) MotionControl_Limit(
        right_speed, -MOTION_COMMAND_LIMIT, MOTION_COMMAND_LIMIT);
    MotionControl_ApplyWheelCommands();
}

void MotionControl_MoveToCounts(int32_t left_target_count,
                                int32_t right_target_count,
                                int16_t max_speed)
{
    if (!g_control.running) MotionControl_Start();

    max_speed = (int16_t) MotionControl_Abs32(max_speed);
    if (max_speed < 1) max_speed = 1;
    if (max_speed > MOTION_POSITION_SPEED_LIMIT) {
        max_speed = (int16_t) MOTION_POSITION_SPEED_LIMIT;
    }

    g_control.left_position_target = left_target_count;
    g_control.right_position_target = right_target_count;
    g_control.position_loop_active = true;
    g_control.position_reached = false;
    g_position_max_speed = max_speed;
    g_position_stable_count = 0U;
    PID_Reset(&g_left_position_pid);
    PID_Reset(&g_right_position_pid);
}

void MotionControl_MoveRelativeCounts(int32_t left_delta_count,
                                      int32_t right_delta_count,
                                      int16_t max_speed)
{
    MotionControl_MoveToCounts(
        Encoder_GetCount(ENCODER_1) + left_delta_count,
        Encoder_GetCount(ENCODER_2) + right_delta_count,
        max_speed);
}

void MotionControl_CancelPosition(void)
{
    g_control.position_loop_active = false;
    g_control.position_reached = false;
    g_control.left_command = 0;
    g_control.right_command = 0;
    g_position_stable_count = 0U;
    PID_Reset(&g_left_position_pid);
    PID_Reset(&g_right_position_pid);
    MotionControl_ApplyWheelCommands();
}

bool MotionControl_IsPositionReached(void)
{
    return g_control.position_reached;
}

void MotionControl_Service(void)
{
    uint32_t primask;
    int32_t left_count;
    int32_t right_count;
    int32_t left_error;
    int32_t right_error;

    MotionControl_RefreshFeedback();
    if (!g_control.running || (g_sample_ready == 0U)) return;

    primask = __get_PRIMASK();
    __disable_irq();
    left_count = g_sample_left_count;
    right_count = g_sample_right_count;
    g_sample_ready = 0U;
    if (primask == 0U) __enable_irq();

    g_control.left_position_count = left_count;
    g_control.right_position_count = right_count;
    if (!g_control.position_loop_active) return;

    left_error = g_control.left_position_target - left_count;
    right_error = g_control.right_position_target - right_count;

    if (MotionControl_Abs32(left_error) <=
        MOTION_POSITION_TOLERANCE_COUNTS) {
        g_control.left_command = 0;
        PID_Reset(&g_left_position_pid);
    } else {
        g_control.left_command = (int16_t) MotionControl_Limit(
            PID_Update(&g_left_position_pid,
                       g_control.left_position_target, left_count),
            -g_position_max_speed, g_position_max_speed);
    }

    if (MotionControl_Abs32(right_error) <=
        MOTION_POSITION_TOLERANCE_COUNTS) {
        g_control.right_command = 0;
        PID_Reset(&g_right_position_pid);
    } else {
        g_control.right_command = (int16_t) MotionControl_Limit(
            PID_Update(&g_right_position_pid,
                       g_control.right_position_target, right_count),
            -g_position_max_speed, g_position_max_speed);
    }

    if ((MotionControl_Abs32(left_error) <=
         MOTION_POSITION_TOLERANCE_COUNTS) &&
        (MotionControl_Abs32(right_error) <=
         MOTION_POSITION_TOLERANCE_COUNTS)) {
        if (g_position_stable_count < MOTION_POSITION_STABLE_SAMPLES) {
            g_position_stable_count++;
        }
    } else {
        g_position_stable_count = 0U;
        g_control.position_reached = false;
    }
    if (g_position_stable_count >= MOTION_POSITION_STABLE_SAMPLES) {
        g_control.position_reached = true;
    }

    MotionControl_ApplyWheelCommands();
}

void MotionControl_Stop(void)
{
    g_control.left_command = 0;
    g_control.right_command = 0;
    g_control.left_target_cps = 0L;
    g_control.right_target_cps = 0L;
    g_control.left_duty = 0;
    g_control.right_duty = 0;
    g_control.angle_loop_active = false;
    g_control.position_loop_active = false;
    g_control.position_reached = false;
    g_control.running = false;
    g_position_stable_count = 0U;
    PID_Reset(&g_left_position_pid);
    PID_Reset(&g_right_position_pid);
    BalanceControl_Disable();
}

MotionControlData MotionControl_GetData(void)
{
    MotionControl_RefreshFeedback();
    return g_control;
}

void Encoder_10msCallback(const EncoderData *encoder1,
                          const EncoderData *encoder2)
{
    if ((g_initialized == 0U) || (encoder1 == 0) || (encoder2 == 0)) {
        return;
    }
    g_sample_left_count = encoder1->count;
    g_sample_right_count = encoder2->count;
    g_sample_ready = 1U;
}
