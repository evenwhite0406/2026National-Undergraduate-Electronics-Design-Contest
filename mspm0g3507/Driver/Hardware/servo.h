/**
 * @file servo.h
 * @brief 双路舵机 PWM 驱动，角度自动限幅并按微秒脉宽换算比较值。
 *
 * SysConfig 配置要求：
 *   1. 新建 PWM 实例，名称设为 Servo；
 *   2. 当前工程使用TIMA0，PA8=C0（舵机1），PA9=C1（舵机2）；
 *   3. 32MHz时钟8分频、周期40000 tick，得到100Hz PWM。
 *
 * 使用方法：
 * @code
 * Servo_Init();
 * Servo_SetAngle(90.0f);                 // 兼容接口：控制舵机1
 * Servo_SetChannelAngle(SERVO_2, 45.0f); // 控制舵机2
 * @endcode
 */

#ifndef HARDWARE_SERVO_H
#define HARDWARE_SERVO_H

#include <stdbool.h>
#include <stdint.h>

/* ======================== 实车可调参数 ======================== */
/* 机械角度范围；不要盲目扩大，否则舵机可能堵转。 */
extern float SERVO_MIN_ANGLE_DEG;
extern float SERVO_MAX_ANGLE_DEG;
extern float SERVO_DEFAULT_ANGLE_DEG;

/* 对应机械端点的控制脉宽，须依据实际舵机逐步校准。 */
extern uint16_t SERVO_MIN_PULSE_US;
extern uint16_t SERVO_CENTER_PULSE_US;
extern uint16_t SERVO_MAX_PULSE_US;

/* 当前TIMA0受16位计数限制，配置为10000us（100Hz）。 */
#define SERVO_PWM_PERIOD_US       (10000U)

/* 1：角度越大输出脉宽越大；0：反向安装，角度越大脉宽越小。 */
extern uint8_t SERVO_DIRECTION_NORMAL;

/*
 * 0：比较值等于高电平脉宽tick；
 * 1：PWM极性相反，比较值等于周期tick减去脉宽tick。
 */
extern uint8_t SERVO_COMPARE_INVERTED;

typedef enum {
    SERVO_1 = 1U, /* PA8 / TIMA0_CCP0 */
    SERVO_2 = 2U  /* PA9 / TIMA0_CCP1 */
} Servo_Channel;

/** 初始化舵机并转到 SERVO_DEFAULT_ANGLE_DEG。 */
void Servo_Init(void);

/** 设置机械角度，超出范围时自动限幅。 */
void Servo_SetAngle(float angle_deg);

/** 设置指定舵机角度；通道无效时不执行操作。 */
void Servo_SetChannelAngle(Servo_Channel channel, float angle_deg);

/** 同时更新两路舵机目标。 */
void Servo_SetAngles(float servo1_deg, float servo2_deg);

/** 直接设置脉宽，超出端点范围时自动限幅。 */
void Servo_SetPulseUs(uint16_t pulse_us);
void Servo_SetChannelPulseUs(Servo_Channel channel, uint16_t pulse_us);

/** 返回最近一次设置的角度和脉宽。 */
float Servo_GetAngle(void);
uint16_t Servo_GetPulseUs(void);
float Servo_GetChannelAngle(Servo_Channel channel);
uint16_t Servo_GetChannelPulseUs(Servo_Channel channel);

/** 返回 SysConfig 是否已经生成舵机 PWM 配置。 */
bool Servo_IsConfigured(void);

#endif
