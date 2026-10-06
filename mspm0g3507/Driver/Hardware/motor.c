/**
 * @file motor.c
 * @brief TB6612 双直流电机驱动
 *
 * 当前 SysConfig 引脚：
 *
 * 实车确认的物理对应关系：
 * 右轮硬件通道：PA26/TIMG8_CCP0，方向PB23/PB21
 * 左轮硬件通道：PB22/TIMG8_CCP1，方向PA22/PB19
 *
 * 历史命名的硬件通道0方向：
 *   L1：PB23
 *   L2：PB21
 *
 * 历史命名的硬件通道1方向：
 *   R1：PA22
 *   R2：PB19
 *
 * TB6612 STBY：
 *   PB20
 *
 * side始终使用物理含义：0=左轮，1=右轮。MOTOR_LOGICAL_SIDE_SWAP负责
 * 把物理左右映射到实际焊接的硬件通道，上层不需要交换控制公式。
 *
 * duty：
 *   -100 ~ 100
 *   正数为前进，负数为后退，0为停止
 */

#include "motor.h"
#include "encoder.h"
#include "../Control/pid.h"
#include "../PeripheralTest/gyro_bmi088.h"
#include "ti_msp_dl_config.h"

#include <stdint.h>

extern volatile uint32_t g_ms;

#define MOTOR_LEFT_SIDE          (0U)
#define MOTOR_RIGHT_SIDE         (1U)
#define MOTOR_INVALID_SIDE       (0xFFU)

/*
 * 当无法读取定时器周期时使用的备用值。
 * 32 MHz / (3199 + 1) = 10 kHz。
 */
#define MOTOR_PWM_PERIOD_DEFAULT (3199U)

/* 圆周率放大1,000,000倍，避免距离换算依赖浮点运算。 */
#define MOTOR_PI_X1000000       (3141593LL)

static PIDController g_speed_pid[2];
static PIDController g_position_pid;
static PIDController g_yaw_pid;
static volatile int16_t g_target_speed[2];
static int16_t g_previous_pid_target[2];
static volatile int16_t g_feedback_speed[2];
static volatile int8_t g_output_duty[2];
static int32_t g_filtered_cps[2];
static uint8_t g_closed_loop_initialized;
static volatile uint8_t g_closed_loop_enabled;
static volatile uint8_t g_position_yaw_active;
static volatile uint8_t g_heading_active;
static volatile uint8_t g_position_yaw_reached;
static int32_t g_position_target;
static int16_t g_yaw_target_x10;
static int16_t g_cruise_speed;
static uint8_t g_target_stable_count;

static int16_t Motor_LimitSpeed(int32_t speed)
{
    if (speed > 100L) return 100;
    if (speed < -100L) return -100;
    return (int16_t) speed;
}

static int16_t Motor_CountsPerSecondToSpeed(int32_t counts_per_second)
{
    int32_t speed = (int32_t) (((int64_t) counts_per_second * 100LL) /
                               ENCODER_SPEED_100_PERCENT_CPS);
    return Motor_LimitSpeed(speed);
}

static int32_t Motor_Abs32(int32_t value)
{
    return (value < 0L) ? -value : value;
}

/** 有符号整数除法，结果四舍五入到最近整数。 */
static int64_t Motor_DivideRoundNearest(int64_t numerator,
                                        int64_t denominator)
{
    if (denominator <= 0LL) {
        return 0LL;
    }

    if (numerator >= 0LL) {
        return (numerator + denominator / 2LL) / denominator;
    }

    return -(((-numerator) + denominator / 2LL) / denominator);
}

/** 把64位结果安全限制到int32_t范围。 */
static int32_t Motor_ClampInt64ToInt32(int64_t value)
{
    if (value > (int64_t) INT32_MAX) {
        return INT32_MAX;
    }
    if (value < (int64_t) INT32_MIN) {
        return INT32_MIN;
    }
    return (int32_t) value;
}

/** 根据有效轮径计算车轮周长，返回单位为微米。 */
static uint32_t Motor_GetWheelCircumferenceUm(void)
{
    int64_t circumference_um;

    if (MOTOR_WHEEL_DIAMETER_UM == 0U) {
        return 0U;
    }

    circumference_um =
        ((int64_t) MOTOR_WHEEL_DIAMETER_UM * MOTOR_PI_X1000000 +
         500000LL) / 1000000LL;

    if (circumference_um <= 0LL) {
        return 0U;
    }
    if (circumference_um > (int64_t) UINT32_MAX) {
        return UINT32_MAX;
    }

    return (uint32_t) circumference_um;
}

static int16_t Motor_WrapYawError(int32_t error_x10)
{
    while (error_x10 > 1800L) error_x10 -= 3600L;
    while (error_x10 < -1800L) error_x10 += 3600L;
    return (int16_t) error_x10;
}

/** 把上层物理左右轮编号转换为板上实际硬件通道。 */
static uint8_t Motor_MapLogicalSide(uint8_t side)
{
    if ((side != MOTOR_LEFT_SIDE) && (side != MOTOR_RIGHT_SIDE)) {
        return MOTOR_INVALID_SIDE;
    }

    if (MOTOR_LOGICAL_SIDE_SWAP != 0U) {
        return (side == MOTOR_LEFT_SIDE) ?
            MOTOR_RIGHT_SIDE : MOTOR_LEFT_SIDE;
    }
    return side;
}


/**
 * @brief 读取当前PWM周期
 *
 * SysConfig负责设置TIMG8的LOAD值，避免代码中的周期值
 * 与SysConfig配置不一致。
 */
static uint32_t Motor_GetPwmPeriod(void)
{
    uint32_t period;

    period = DL_TimerG_getLoadValue(Motor_INST);

    if (period == 0U) {
        period = MOTOR_PWM_PERIOD_DEFAULT;
    }

    return period;
}


/**
 * @brief 设置左电机方向引脚
 */
static void Motor_SetLeftDirection(int8_t direction)
{
    if (direction > 0) {
        /* 左轮前进：L1=0，L2=1 */
        DL_GPIO_clearPins(
            GPIO_MOTOR_PIN_L1_PORT,
            GPIO_MOTOR_PIN_L1_PIN);

        DL_GPIO_setPins(
            GPIO_MOTOR_PIN_L2_PORT,
            GPIO_MOTOR_PIN_L2_PIN);
    } else if (direction < 0) {
        /* 左轮后退：L1=1，L2=0 */
        DL_GPIO_setPins(
            GPIO_MOTOR_PIN_L1_PORT,
            GPIO_MOTOR_PIN_L1_PIN);

        DL_GPIO_clearPins(
            GPIO_MOTOR_PIN_L2_PORT,
            GPIO_MOTOR_PIN_L2_PIN);
    } else {
        /* 停止：L1=0，L2=0 */
        DL_GPIO_clearPins(
            GPIO_MOTOR_PIN_L1_PORT,
            GPIO_MOTOR_PIN_L1_PIN);

        DL_GPIO_clearPins(
            GPIO_MOTOR_PIN_L2_PORT,
            GPIO_MOTOR_PIN_L2_PIN);
    }
}


/**
 * @brief 设置右电机方向引脚
 */
static void Motor_SetRightDirection(int8_t direction)
{
    if (direction > 0) {
        /* 右轮前进：R1=1，R2=0 */
        DL_GPIO_setPins(
            GPIO_MOTOR_PIN_R1_PORT,
            GPIO_MOTOR_PIN_R1_PIN);

        DL_GPIO_clearPins(
            GPIO_MOTOR_PIN_R2_PORT,
            GPIO_MOTOR_PIN_R2_PIN);
    } else if (direction < 0) {
        /* 右轮后退：R1=0，R2=1 */
        DL_GPIO_clearPins(
            GPIO_MOTOR_PIN_R1_PORT,
            GPIO_MOTOR_PIN_R1_PIN);

        DL_GPIO_setPins(
            GPIO_MOTOR_PIN_R2_PORT,
            GPIO_MOTOR_PIN_R2_PIN);
    } else {
        /* 停止：R1=0，R2=0 */
        DL_GPIO_clearPins(
            GPIO_MOTOR_PIN_R1_PORT,
            GPIO_MOTOR_PIN_R1_PIN);

        DL_GPIO_clearPins(
            GPIO_MOTOR_PIN_R2_PORT,
            GPIO_MOTOR_PIN_R2_PIN);
    }
}


void Motor_Init(void)
{
    PID_Init(&g_speed_pid[MOTOR_LEFT_SIDE],
             MOTOR_SPEED_PID_KP_X1000,
             MOTOR_SPEED_PID_KI_X1000,
             MOTOR_SPEED_PID_KD_X1000, -100L, 100L);
    PID_Init(&g_speed_pid[MOTOR_RIGHT_SIDE],
             MOTOR_SPEED_PID_KP_X1000,
             MOTOR_SPEED_PID_KI_X1000,
             MOTOR_SPEED_PID_KD_X1000, -100L, 100L);
    PID_SetIntegralLimits(&g_speed_pid[MOTOR_LEFT_SIDE], -100L, 100L);
    PID_SetIntegralLimits(&g_speed_pid[MOTOR_RIGHT_SIDE], -100L, 100L);
    PID_Init(&g_position_pid,
             MOTOR_POSITION_PID_KP_X1000,
             MOTOR_POSITION_PID_KI_X1000,
             MOTOR_POSITION_PID_KD_X1000,
             -MOTOR_POSITION_SPEED_LIMIT, MOTOR_POSITION_SPEED_LIMIT);
    PID_Init(&g_yaw_pid,
             MOTOR_YAW_PID_KP_X1000,
             MOTOR_YAW_PID_KI_X1000,
             MOTOR_YAW_PID_KD_X1000,
             -MOTOR_YAW_SPEED_LIMIT, MOTOR_YAW_SPEED_LIMIT);

    g_target_speed[MOTOR_LEFT_SIDE] = 0;
    g_target_speed[MOTOR_RIGHT_SIDE] = 0;
    g_previous_pid_target[MOTOR_LEFT_SIDE] = 0;
    g_previous_pid_target[MOTOR_RIGHT_SIDE] = 0;
    g_feedback_speed[MOTOR_LEFT_SIDE] = 0;
    g_feedback_speed[MOTOR_RIGHT_SIDE] = 0;
    g_output_duty[MOTOR_LEFT_SIDE] = 0;
    g_output_duty[MOTOR_RIGHT_SIDE] = 0;
    g_filtered_cps[MOTOR_LEFT_SIDE] = 0L;
    g_filtered_cps[MOTOR_RIGHT_SIDE] = 0L;
    g_position_yaw_active = 0U;
    g_heading_active = 0U;
    g_position_yaw_reached = 0U;
    g_target_stable_count = 0U;

    Encoder_Init(MOTOR_ENCODER_LEFT_CPR, MOTOR_ENCODER_RIGHT_CPR);
    /*
     * Vehicle measurement: when both wheels move forward, the left encoder
     * counts negative while the right encoder counts positive.  Normalize
     * both channels here so positive always means vehicle-forward.
     */
    /* Current motor polarity test: commanded wheel direction and encoder
     * feedback were opposite on both channels. */
    Encoder_SetReversed(ENCODER_1, false);
    Encoder_SetReversed(ENCODER_2, true);
    g_closed_loop_initialized = 1U;
    g_closed_loop_enabled = 1U;
    Motor_SetDuty(MOTOR_LEFT, 0);
    Motor_SetDuty(MOTOR_RIGHT, 0);
    Motor_On();
}

/**
 * @brief 使能TB6612
 */
void Motor_On(void)
{
    /*
     * 确保TIMG8开始计数。
     * 即使SysConfig已经启动，再调用一次也不会改变PWM参数。
     */
    DL_TimerG_startCounter(Motor_INST);

    /* STBY高电平，TB6612进入工作状态 */
    DL_GPIO_setPins(
        GPIO_MOTOR_PIN_STBY_PORT,
        GPIO_MOTOR_PIN_STBY_PIN);
}


/**
 * @brief 关闭TB6612并停止两个电机
 */
void Motor_Off(void)
{
    uint32_t period = Motor_GetPwmPeriod();

    /*
     * 当前PWM采用反向比较关系：
     * compare越小，占空比越大；
     * compare等于period时，占空比为0。
     */
    DL_TimerG_setCaptureCompareValue(
        Motor_INST,
        period,
        GPIO_Motor_C0_IDX);

    DL_TimerG_setCaptureCompareValue(
        Motor_INST,
        period,
        GPIO_Motor_C1_IDX);

    Motor_SetLeftDirection(0);
    Motor_SetRightDirection(0);

    /* STBY低电平，TB6612待机 */
    DL_GPIO_clearPins(
        GPIO_MOTOR_PIN_STBY_PORT,
        GPIO_MOTOR_PIN_STBY_PIN);

    g_closed_loop_enabled = 0U;
    g_position_yaw_active = 0U;
    g_heading_active = 0U;
    g_target_speed[MOTOR_LEFT_SIDE] = 0;
    g_target_speed[MOTOR_RIGHT_SIDE] = 0;
    g_output_duty[MOTOR_LEFT_SIDE] = 0;
    g_output_duty[MOTOR_RIGHT_SIDE] = 0;
    if (g_closed_loop_initialized != 0U) {
        PID_Reset(&g_speed_pid[MOTOR_LEFT_SIDE]);
        PID_Reset(&g_speed_pid[MOTOR_RIGHT_SIDE]);
    }
}

void Motor_SetSpeed(uint8_t side, int16_t speed)
{
    if ((side != MOTOR_LEFT_SIDE) && (side != MOTOR_RIGHT_SIDE)) {
        return;
    }
    if (g_closed_loop_initialized == 0U) {
        Motor_Init();
    } else if (g_closed_loop_enabled == 0U) {
        g_closed_loop_enabled = 1U;
        Motor_On();
    }

    speed = Motor_LimitSpeed(speed);
    g_position_yaw_active = 0U;
    g_heading_active = 0U;
    g_position_yaw_reached = 0U;
    g_target_speed[side] = speed;
    if (speed == 0) {
        PID_Reset(&g_speed_pid[side]);
        g_output_duty[side] = 0;
        Motor_SetDuty(side, 0);
    }
}

void Motor_SetSpeeds(int16_t left_speed, int16_t right_speed)
{
    uint32_t primask;
    uint8_t stop_left;
    uint8_t stop_right;

    if (g_closed_loop_initialized == 0U) {
        Motor_Init();
    } else if (g_closed_loop_enabled == 0U) {
        g_closed_loop_enabled = 1U;
        Motor_On();
    }

    left_speed = Motor_LimitSpeed(left_speed);
    right_speed = Motor_LimitSpeed(right_speed);
    stop_left = (left_speed == 0) ? 1U : 0U;
    stop_right = (right_speed == 0) ? 1U : 0U;

    /*
     * PID运行在10ms定时器中断中。短暂屏蔽中断，保证它只能看到
     * 两个旧目标或两个新目标，不会看到左右轮一新一旧。
     */
    primask = __get_PRIMASK();
    __disable_irq();

    g_position_yaw_active = 0U;
    g_heading_active = 0U;
    g_position_yaw_reached = 0U;
    g_target_speed[MOTOR_LEFT_SIDE] = left_speed;
    g_target_speed[MOTOR_RIGHT_SIDE] = right_speed;

    if (stop_left != 0U) {
        PID_Reset(&g_speed_pid[MOTOR_LEFT_SIDE]);
        g_output_duty[MOTOR_LEFT_SIDE] = 0;
    }
    if (stop_right != 0U) {
        PID_Reset(&g_speed_pid[MOTOR_RIGHT_SIDE]);
        g_output_duty[MOTOR_RIGHT_SIDE] = 0;
    }

    if (primask == 0U) {
        __enable_irq();
    }

    /* 零速命令立即撤掉PWM，不必等待下一次10ms控制中断。 */
    if (stop_left != 0U) {
        Motor_SetDuty(MOTOR_LEFT, 0);
    }
    if (stop_right != 0U) {
        Motor_SetDuty(MOTOR_RIGHT, 0);
    }
}

void Motor_DriveHeading(int16_t speed, int16_t target_yaw_x10)
{
    uint32_t primask;

    if (g_closed_loop_initialized == 0U) {
        Motor_Init();
    } else if (g_closed_loop_enabled == 0U) {
        g_closed_loop_enabled = 1U;
        Motor_On();
    }

    speed = Motor_LimitSpeed(speed);
    target_yaw_x10 = Motor_WrapYawError(target_yaw_x10);

    primask = __get_PRIMASK();
    __disable_irq();
    g_cruise_speed = speed;
    g_yaw_target_x10 = target_yaw_x10;
    g_position_yaw_active = 0U;
    g_position_yaw_reached = 0U;
    PID_Reset(&g_yaw_pid);
    PID_Reset(&g_speed_pid[MOTOR_LEFT_SIDE]);
    PID_Reset(&g_speed_pid[MOTOR_RIGHT_SIDE]);
    g_heading_active = 1U;
    if (primask == 0U) __enable_irq();
}

void Motor_StopHeading(void)
{
    Motor_SetSpeeds(0, 0);
}

int32_t Motor_DistanceMmToCounts(int32_t distance_mm)
{
    uint32_t circumference_um = Motor_GetWheelCircumferenceUm();
    uint64_t cpr_sum =
        (uint64_t) MOTOR_ENCODER_LEFT_CPR +
        (uint64_t) MOTOR_ENCODER_RIGHT_CPR;
    int64_t numerator;
    int64_t denominator;
    int64_t result;

    if ((circumference_um == 0U) || (cpr_sum == 0ULL)) {
        return 0L;
    }

    /*
     * 位置环反馈为左右计数平均值：
     * average_count = distance_um × (CPR_L + CPR_R)
     *                 / (2 × circumference_um)
     */
    numerator = (int64_t) distance_mm * 1000LL * (int64_t) cpr_sum;
    denominator = 2LL * (int64_t) circumference_um;
    result = Motor_DivideRoundNearest(numerator, denominator);

    return Motor_ClampInt64ToInt32(result);
}

int32_t Motor_DistanceCmToCounts(int32_t distance_cm)
{
    int64_t distance_mm = (int64_t) distance_cm * 10LL;

    if (distance_mm > (int64_t) INT32_MAX) {
        distance_mm = (int64_t) INT32_MAX;
    } else if (distance_mm < (int64_t) INT32_MIN) {
        distance_mm = (int64_t) INT32_MIN;
    }

    return Motor_DistanceMmToCounts((int32_t) distance_mm);
}

int32_t Motor_CountsToDistanceMm(int32_t counts)
{
    uint32_t circumference_um = Motor_GetWheelCircumferenceUm();
    uint64_t cpr_sum =
        (uint64_t) MOTOR_ENCODER_LEFT_CPR +
        (uint64_t) MOTOR_ENCODER_RIGHT_CPR;
    int64_t numerator;
    int64_t denominator;
    int64_t result;

    if ((circumference_um == 0U) || (cpr_sum == 0ULL)) {
        return 0L;
    }

    numerator =
        (int64_t) counts * 2LL * (int64_t) circumference_um;
    denominator = 1000LL * (int64_t) cpr_sum;
    result = Motor_DivideRoundNearest(numerator, denominator);

    return Motor_ClampInt64ToInt32(result);
}

void Motor_DriveDistanceMmHeading(int16_t speed,
                                  int32_t distance_mm,
                                  int16_t target_yaw_x10)
{
    int32_t relative_counts = Motor_DistanceMmToCounts(distance_mm);

    /*
     * 非零距离因四舍五入得到0计数时，至少执行1个计数，避免命令无动作。
     */
    if ((distance_mm > 0L) && (relative_counts == 0L)) {
        relative_counts = 1L;
    } else if ((distance_mm < 0L) && (relative_counts == 0L)) {
        relative_counts = -1L;
    }

    Motor_DriveDistanceHeading(speed, relative_counts, target_yaw_x10);
}

void Motor_DriveDistanceCmHeading(int16_t speed,
                                  int32_t distance_cm,
                                  int16_t target_yaw_x10)
{
    int32_t relative_counts = Motor_DistanceCmToCounts(distance_cm);

    if ((distance_cm > 0L) && (relative_counts == 0L)) {
        relative_counts = 1L;
    } else if ((distance_cm < 0L) && (relative_counts == 0L)) {
        relative_counts = -1L;
    }

    Motor_DriveDistanceHeading(speed, relative_counts, target_yaw_x10);
}

void Motor_DriveDistanceHeading(int16_t speed,
                                int32_t relative_counts,
                                int16_t target_yaw_x10)
{
    int32_t current_position;
    uint32_t primask;

    if (g_closed_loop_initialized == 0U) {
        Motor_Init();
    } else if (g_closed_loop_enabled == 0U) {
        g_closed_loop_enabled = 1U;
        Motor_On();
    }

    if (speed < 0) speed = (int16_t) -speed;
    if (speed < 1) speed = 1;
    if (speed > 100) speed = 100;
    if (relative_counts < 0L) speed = (int16_t) -speed;

    current_position = (int32_t)
        (((int64_t) Encoder_GetCount(ENCODER_1) +
          (int64_t) Encoder_GetCount(ENCODER_2)) / 2LL);
    target_yaw_x10 = Motor_WrapYawError(target_yaw_x10);

    primask = __get_PRIMASK();
    __disable_irq();
    g_position_target = current_position + relative_counts;
    g_yaw_target_x10 = target_yaw_x10;
    g_cruise_speed = speed;
    g_heading_active = 0U;
    g_position_yaw_reached = 0U;
    g_target_stable_count = 0U;
    PID_Reset(&g_position_pid);
    PID_Reset(&g_yaw_pid);
    PID_Reset(&g_speed_pid[MOTOR_LEFT_SIDE]);
    PID_Reset(&g_speed_pid[MOTOR_RIGHT_SIDE]);
    g_position_yaw_active = 1U;
    if (primask == 0U) __enable_irq();
}

void Motor_MovePositionYaw(int32_t relative_counts,
                           int16_t target_yaw_x10)
{
    Motor_DriveDistanceHeading(30, relative_counts, target_yaw_x10);
}

void Motor_CancelPositionYaw(void)
{
    Motor_SetSpeeds(0, 0);
}

uint8_t Motor_IsPositionYawReached(void)
{
    return g_position_yaw_reached;
}

void Motor_SetSpeedPID(int32_t kp_x1000,
                       int32_t ki_x1000,
                       int32_t kd_x1000)
{
    if (g_closed_loop_initialized == 0U) {
        Motor_Init();
    }
    PID_SetTunings(&g_speed_pid[MOTOR_LEFT_SIDE],
                   kp_x1000, ki_x1000, kd_x1000);
    PID_SetTunings(&g_speed_pid[MOTOR_RIGHT_SIDE],
                   kp_x1000, ki_x1000, kd_x1000);
    PID_Reset(&g_speed_pid[MOTOR_LEFT_SIDE]);
    PID_Reset(&g_speed_pid[MOTOR_RIGHT_SIDE]);
}

void Motor_SetYawPID(int32_t kp_x1000,
                     int32_t ki_x1000,
                     int32_t kd_x1000)
{
    if (g_closed_loop_initialized == 0U) {
        Motor_Init();
    }
    PID_SetTunings(&g_yaw_pid, kp_x1000, ki_x1000, kd_x1000);
    PID_Reset(&g_yaw_pid);
}

int16_t Motor_GetTargetSpeed(uint8_t side)
{
    if ((side != MOTOR_LEFT_SIDE) && (side != MOTOR_RIGHT_SIDE)) return 0;
    return g_target_speed[side];
}

int16_t Motor_GetFeedbackSpeed(uint8_t side)
{
    if ((side != MOTOR_LEFT_SIDE) && (side != MOTOR_RIGHT_SIDE)) return 0;
    return g_feedback_speed[side];
}

int8_t Motor_GetOutputDuty(uint8_t side)
{
    if ((side != MOTOR_LEFT_SIDE) && (side != MOTOR_RIGHT_SIDE)) return 0;
    return g_output_duty[side];
}

void Motor_SpeedControlUpdate(int32_t left_count,
                              int32_t right_count,
                              int32_t left_counts_per_second,
                              int32_t right_counts_per_second)
{
    int32_t measured_cps[2];
    int16_t yaw_feedback = 0;
    uint32_t yaw_update_ms = 0U;
    uint8_t yaw_valid = 0U;
    uint8_t side;

    if ((g_closed_loop_initialized == 0U) ||
        (g_closed_loop_enabled == 0U)) {
        return;
    }

    measured_cps[MOTOR_LEFT_SIDE] = left_counts_per_second;
    measured_cps[MOTOR_RIGHT_SIDE] = right_counts_per_second;

    if ((g_heading_active != 0U) || (g_position_yaw_active != 0U)) {
        yaw_valid = BMI088_Gyro_GetYawSnapshot(&yaw_feedback,
                                               &yaw_update_ms);
        if ((yaw_valid != 0U) &&
            ((uint32_t) (g_ms - yaw_update_ms) > MOTOR_IMU_TIMEOUT_MS)) {
            yaw_valid = 0U;
        }
    }

    if (g_heading_active != 0U) {
        if (yaw_valid == 0U) {
            /* 没有可靠yaw反馈时自动停车。 */
            g_heading_active = 0U;
            g_target_speed[MOTOR_LEFT_SIDE] = 0;
            g_target_speed[MOTOR_RIGHT_SIDE] = 0;
            PID_Reset(&g_yaw_pid);
        } else {
            int16_t yaw_error = Motor_WrapYawError(
                (int32_t) g_yaw_target_x10 - yaw_feedback);
            int32_t turn_speed = PID_Update(&g_yaw_pid, yaw_error, 0L);

            /* 基础速度相同，yaw环只叠加幅度相反的差速修正。 */
            /* Vehicle test: left-negative/right-positive produces negative
             * yaw, so positive yaw correction is left-positive/right-negative. */
            g_target_speed[MOTOR_LEFT_SIDE] = Motor_LimitSpeed(
                (int32_t) g_cruise_speed + turn_speed);
            g_target_speed[MOTOR_RIGHT_SIDE] = Motor_LimitSpeed(
                (int32_t) g_cruise_speed - turn_speed);
        }
    }

    if (g_position_yaw_active != 0U) {
        int32_t position = (int32_t) (((int64_t) left_count +
                                      (int64_t) right_count) / 2LL);
        int32_t position_error = g_position_target - position;

        /* yaw闭环失去有效IMU反馈时立即退出运动，禁止盲目继续输出。 */
        if (yaw_valid == 0U) {
            g_position_yaw_active = 0U;
            g_position_yaw_reached = 0U;
            g_target_speed[MOTOR_LEFT_SIDE] = 0;
            g_target_speed[MOTOR_RIGHT_SIDE] = 0;
            PID_Reset(&g_position_pid);
            PID_Reset(&g_yaw_pid);
        }

        if (g_position_yaw_active != 0U) {
            int16_t yaw_error = Motor_WrapYawError(
                (int32_t) g_yaw_target_x10 - yaw_feedback);
            int32_t position_speed = PID_Update(&g_position_pid,
                                                g_position_target, position);
            int32_t base_speed = g_cruise_speed;
            int32_t turn_speed = PID_Update(&g_yaw_pid, yaw_error, 0L);

            /* 按指定速度巡航；接近目标时位置环只负责限速和停车。 */
            if (Motor_Abs32(position_speed) < Motor_Abs32(base_speed)) {
                base_speed = position_speed;
            }

            g_target_speed[MOTOR_LEFT_SIDE] = Motor_LimitSpeed(
                base_speed + turn_speed);
            g_target_speed[MOTOR_RIGHT_SIDE] = Motor_LimitSpeed(
                base_speed - turn_speed);

            if ((Motor_Abs32(position_error) <=
                 MOTOR_POSITION_TOLERANCE_COUNTS) &&
                (Motor_Abs32(yaw_error) <= MOTOR_YAW_TOLERANCE_X10)) {
                if (g_target_stable_count < MOTOR_TARGET_STABLE_SAMPLES) {
                    g_target_stable_count++;
                }
            } else {
                g_target_stable_count = 0U;
            }

            if (g_target_stable_count >= MOTOR_TARGET_STABLE_SAMPLES) {
                g_position_yaw_active = 0U;
                g_position_yaw_reached = 1U;
                g_target_speed[MOTOR_LEFT_SIDE] = 0;
                g_target_speed[MOTOR_RIGHT_SIDE] = 0;
                PID_Reset(&g_position_pid);
                PID_Reset(&g_yaw_pid);
            }
        }
    }

    for (side = MOTOR_LEFT_SIDE; side <= MOTOR_RIGHT_SIDE; side++) {
        int32_t output;

        /* 约50ms的一阶低通，减小10ms计数分辨率造成的速度跳变。 */
        g_filtered_cps[side] +=
            (measured_cps[side] - g_filtered_cps[side]) / 5L;
        g_feedback_speed[side] =
            Motor_CountsPerSecondToSpeed(g_filtered_cps[side]);

        if (g_target_speed[side] == 0) {
            PID_Reset(&g_speed_pid[side]);
            output = 0L;
        } else {
            /* A yaw correction can reverse one wheel target.  Do not carry
             * the previous direction's integral state across zero. */
            if (((g_target_speed[side] > 0) &&
                 (g_previous_pid_target[side] <= 0)) ||
                ((g_target_speed[side] < 0) &&
                 (g_previous_pid_target[side] >= 0))) {
                PID_Reset(&g_speed_pid[side]);
            }
            output = PID_Update(&g_speed_pid[side],
                                g_target_speed[side],
                                g_feedback_speed[side]);

            /*
             * A speed overshoot must reduce drive to zero, not command the
             * opposite direction.  Reversing the H-bridge every time the
             * measured speed crosses the target makes the vehicle jerk and
             * can leave it travelling opposite to a positive command.
             */
            if ((g_target_speed[side] > 0) && (output < 0L)) {
                output = 0L;
            } else if ((g_target_speed[side] < 0) && (output > 0L)) {
                output = 0L;
            }
        }
        g_previous_pid_target[side] = g_target_speed[side];

        g_output_duty[side] = (int8_t) output;
        Motor_SetDuty(side, (int8_t) output);
    }
}


/**
 * @brief 设置一个电机的PWM占空比和方向
 *
 * @param side  0为左轮，1为右轮
 * @param duty  -100~100
 */
void Motor_SetDuty(uint8_t side, int8_t duty)
{
    int16_t signed_duty;
    uint32_t magnitude;
    uint32_t period;
    uint32_t compare_value;

    side = Motor_MapLogicalSide(side);
    if (side == MOTOR_INVALID_SIDE) {
        return;
    }

    /* 上层始终使用“正数向前”，接线方向差异只在驱动层处理。 */
    signed_duty = (int16_t) duty * MOTOR_OUTPUT_SIGN;

    /* 限制占空比范围 */
    if (signed_duty > 100) {
        signed_duty = 100;
    } else if (signed_duty < -100) {
        signed_duty = -100;
    }

    period = Motor_GetPwmPeriod();

    /*
     * 先让对应PWM变为0%，防止切换方向时电机瞬间冲击。
     */
    if (side == MOTOR_LEFT_SIDE) {
        DL_TimerG_setCaptureCompareValue(
            Motor_INST,
            period,
            GPIO_Motor_C0_IDX);
    } else if (side == MOTOR_RIGHT_SIDE) {
        DL_TimerG_setCaptureCompareValue(
            Motor_INST,
            period,
            GPIO_Motor_C1_IDX);
    } else {
        /* 非法电机编号 */
        return;
    }

    if (signed_duty == 0) {
        if (side == MOTOR_LEFT_SIDE) {
            Motor_SetLeftDirection(0);
        } else {
            Motor_SetRightDirection(0);
        }

        return;
    }

    /* 先设置方向 */
    if (side == MOTOR_LEFT_SIDE) {
        if (signed_duty > 0) {
            Motor_SetLeftDirection(1);
        } else {
            Motor_SetLeftDirection(-1);
        }
    } else {
        if (signed_duty > 0) {
            Motor_SetRightDirection(1);
        } else {
            Motor_SetRightDirection(-1);
        }
    }

    if (signed_duty > 0) {
        magnitude = (uint32_t) signed_duty;
    } else {
        magnitude = (uint32_t) (-signed_duty);
    }

    /*
     * 当前PWM极性：
     *
     * duty = 0：
     *   compare = period
     *
     * duty = 50：
     *   compare约为period的一半
     *
     * duty = 100：
     *   compare = 0
     */
    compare_value =
        period - ((period * magnitude) / 100U);

    /* 最后输出PWM，避免方向切换时产生突跳 */
    if (side == MOTOR_LEFT_SIDE) {
        DL_TimerG_setCaptureCompareValue(
            Motor_INST,
            compare_value,
            GPIO_Motor_C0_IDX);
    } else {
        DL_TimerG_setCaptureCompareValue(
            Motor_INST,
            compare_value,
            GPIO_Motor_C1_IDX);
    }
}
