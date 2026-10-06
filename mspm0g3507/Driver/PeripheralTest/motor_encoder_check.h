/**
 * @file motor_encoder_check.h
 * @brief 低PWM验证左右电机与两路编码器对应关系。
 */

#ifndef MOTOR_ENCODER_CHECK_H
#define MOTOR_ENCODER_CHECK_H

#include <stdint.h>

/* 调用：Init -> Start(now_ms)，主循环Update(now_ms)，最后GetResult。测试时架空车轮。 */
#define CHECK_PWM_DUTY             (18)   /* 测试PWM，能转动且尽量低。 */
#define CHECK_DRIVE_MS             (600U) /* 单轮测试时间。 */
#define CHECK_PAUSE_MS             (300U)
#define CHECK_MIN_COUNTS           (2L)   /* 判定编码器有效的最少计数。 */
#define CHECK_DOMINANCE_MULTIPLIER (2L)   /* 主通道至少为另一通道的倍数。 */

typedef enum {
    MOTOR_ENCODER_CHECK_IDLE = 0,
    MOTOR_ENCODER_CHECK_LEFT_RUNNING,
    MOTOR_ENCODER_CHECK_PAUSE,
    MOTOR_ENCODER_CHECK_RIGHT_RUNNING,
    MOTOR_ENCODER_CHECK_RESULT
} MotorEncoderCheckState;

typedef struct {
    int32_t left_test_encoder1_delta;
    int32_t left_test_encoder2_delta;
    int32_t right_test_encoder1_delta;
    int32_t right_test_encoder2_delta;
    uint8_t left_motor_encoder;   /* 0=无法判断，1=ENCODER_1，2=ENCODER_2 */
    uint8_t right_motor_encoder;
    int8_t left_encoder_sign;     /* -1/0/+1 */
    int8_t right_encoder_sign;
} MotorEncoderCheckResult;

void MotorEncoderCheck_Init(void);
void MotorEncoderCheck_Start(uint32_t now_ms);
MotorEncoderCheckState MotorEncoderCheck_Update(uint32_t now_ms);
void MotorEncoderCheck_Stop(void);
MotorEncoderCheckState MotorEncoderCheck_GetState(void);
MotorEncoderCheckResult MotorEncoderCheck_GetResult(void);

#endif
