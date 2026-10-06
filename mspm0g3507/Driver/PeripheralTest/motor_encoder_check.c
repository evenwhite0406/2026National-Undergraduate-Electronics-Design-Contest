/**
 * @file motor_encoder_check.c
 * @brief 电机与编码器对应关系非阻塞自检。
 *
 * 本测试用于闭环方向尚未确认的阶段，因此有意绕过三环控制器，以低PWM依次
 * 驱动左右电机。测试时必须架空车轮，并观察“左电机”阶段是否确实只有左轮转。
 */

#include "motor_encoder_check.h"
#include "../Hardware/encoder.h"
#include "../Hardware/motor.h"

static MotorEncoderCheckState g_check_state;
static MotorEncoderCheckResult g_check_result;
static uint32_t g_deadline_ms;
static int32_t g_encoder1_start;
static int32_t g_encoder2_start;

static int32_t MotorEncoderCheck_Abs(int32_t value)
{
    return (value < 0L) ? -value : value;
}

static void MotorEncoderCheck_ClearResult(void)
{
    g_check_result.left_test_encoder1_delta = 0L;
    g_check_result.left_test_encoder2_delta = 0L;
    g_check_result.right_test_encoder1_delta = 0L;
    g_check_result.right_test_encoder2_delta = 0L;
    g_check_result.left_motor_encoder = 0U;
    g_check_result.right_motor_encoder = 0U;
    g_check_result.left_encoder_sign = 0;
    g_check_result.right_encoder_sign = 0;
}

static uint8_t MotorEncoderCheck_SelectEncoder(int32_t delta1,
                                                int32_t delta2,
                                                int8_t *sign)
{
    int32_t magnitude1 = MotorEncoderCheck_Abs(delta1);
    int32_t magnitude2 = MotorEncoderCheck_Abs(delta2);
    uint8_t encoder = 0U;
    int32_t selected_delta = 0L;

    if ((magnitude1 >= CHECK_MIN_COUNTS) &&
        (magnitude1 >= magnitude2 * CHECK_DOMINANCE_MULTIPLIER)) {
        encoder = 1U;
        selected_delta = delta1;
    } else if ((magnitude2 >= CHECK_MIN_COUNTS) &&
               (magnitude2 >= magnitude1 * CHECK_DOMINANCE_MULTIPLIER)) {
        encoder = 2U;
        selected_delta = delta2;
    }

    if (sign != 0) {
        *sign = (selected_delta > 0L) ? 1 :
                ((selected_delta < 0L) ? -1 : 0);
    }
    return encoder;
}

void MotorEncoderCheck_Init(void)
{
    g_check_state = MOTOR_ENCODER_CHECK_IDLE;
    MotorEncoderCheck_ClearResult();
    Motor_SetDuty(MOTOR_LEFT, 0);
    Motor_SetDuty(MOTOR_RIGHT, 0);
}

void MotorEncoderCheck_Start(uint32_t now_ms)
{
    MotorEncoderCheck_Stop();
    MotorEncoderCheck_ClearResult();
    g_encoder1_start = Encoder_GetCount(ENCODER_1);
    g_encoder2_start = Encoder_GetCount(ENCODER_2);
    Motor_SetDuty(MOTOR_RIGHT, 0);
    Motor_SetDuty(MOTOR_LEFT, CHECK_PWM_DUTY);
    Motor_On();
    g_deadline_ms = now_ms + CHECK_DRIVE_MS;
    g_check_state = MOTOR_ENCODER_CHECK_LEFT_RUNNING;
}

MotorEncoderCheckState MotorEncoderCheck_Update(uint32_t now_ms)
{
    if ((g_check_state == MOTOR_ENCODER_CHECK_IDLE) ||
        (g_check_state == MOTOR_ENCODER_CHECK_RESULT) ||
        ((int32_t) (now_ms - g_deadline_ms) < 0)) {
        return g_check_state;
    }

    if (g_check_state == MOTOR_ENCODER_CHECK_LEFT_RUNNING) {
        g_check_result.left_test_encoder1_delta =
            Encoder_GetCount(ENCODER_1) - g_encoder1_start;
        g_check_result.left_test_encoder2_delta =
            Encoder_GetCount(ENCODER_2) - g_encoder2_start;
        Motor_SetDuty(MOTOR_LEFT, 0);
        Motor_SetDuty(MOTOR_RIGHT, 0);
        Motor_Off();
        g_deadline_ms = now_ms + CHECK_PAUSE_MS;
        g_check_state = MOTOR_ENCODER_CHECK_PAUSE;
    } else if (g_check_state == MOTOR_ENCODER_CHECK_PAUSE) {
        g_encoder1_start = Encoder_GetCount(ENCODER_1);
        g_encoder2_start = Encoder_GetCount(ENCODER_2);
        Motor_SetDuty(MOTOR_LEFT, 0);
        Motor_SetDuty(MOTOR_RIGHT, CHECK_PWM_DUTY);
        Motor_On();
        g_deadline_ms = now_ms + CHECK_DRIVE_MS;
        g_check_state = MOTOR_ENCODER_CHECK_RIGHT_RUNNING;
    } else if (g_check_state == MOTOR_ENCODER_CHECK_RIGHT_RUNNING) {
        g_check_result.right_test_encoder1_delta =
            Encoder_GetCount(ENCODER_1) - g_encoder1_start;
        g_check_result.right_test_encoder2_delta =
            Encoder_GetCount(ENCODER_2) - g_encoder2_start;
        Motor_SetDuty(MOTOR_LEFT, 0);
        Motor_SetDuty(MOTOR_RIGHT, 0);
        Motor_Off();
        g_check_result.left_motor_encoder = MotorEncoderCheck_SelectEncoder(
            g_check_result.left_test_encoder1_delta,
            g_check_result.left_test_encoder2_delta,
            &g_check_result.left_encoder_sign);
        g_check_result.right_motor_encoder = MotorEncoderCheck_SelectEncoder(
            g_check_result.right_test_encoder1_delta,
            g_check_result.right_test_encoder2_delta,
            &g_check_result.right_encoder_sign);
        g_check_state = MOTOR_ENCODER_CHECK_RESULT;
    }
    return g_check_state;
}

void MotorEncoderCheck_Stop(void)
{
    Motor_SetDuty(MOTOR_LEFT, 0);
    Motor_SetDuty(MOTOR_RIGHT, 0);
    Motor_Off();
    g_check_state = MOTOR_ENCODER_CHECK_IDLE;
}

MotorEncoderCheckState MotorEncoderCheck_GetState(void)
{
    return g_check_state;
}

MotorEncoderCheckResult MotorEncoderCheck_GetResult(void)
{
    return g_check_result;
}
