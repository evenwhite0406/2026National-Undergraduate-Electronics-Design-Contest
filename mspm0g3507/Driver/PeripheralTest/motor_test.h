/**
 * @file motor_test.h
 * @brief 工程双路电机驱动的安全测试指令封装。
 *
 * 典型用法：
 * @code
 * MotorTest_Init();
 * MotorTest_Drive(MOTOR_TEST_BOTH_FORWARD, 20); // 双电机以 20% 输出前进。
 * delay_cycles(16000000U);                      // 32 MHz 下约为 0.5 秒。
 * MotorTest_Stop();
 * @endcode
 *
 * 执行电机测试前必须架空小车，确保车轮可以自由转动。
 */

#ifndef MOTOR_TEST_H
#define MOTOR_TEST_H

#include <stdint.h>

typedef enum {
    MOTOR_TEST_STOP = 0,
    MOTOR_TEST_LEFT_FORWARD,
    MOTOR_TEST_LEFT_BACKWARD,
    MOTOR_TEST_RIGHT_FORWARD,
    MOTOR_TEST_RIGHT_BACKWARD,
    MOTOR_TEST_BOTH_FORWARD,
    MOTOR_TEST_BOTH_BACKWARD
} MotorTestCommand;

/** 初始化并使两路电机处于停止/待机状态。 */
void MotorTest_Init(void);

/** 将两路 PWM 清零并关闭电机驱动。 */
void MotorTest_Stop(void);

/** 执行一条指令；speed为闭环速度指令，按绝对值限制在100以内。 */
void MotorTest_Drive(MotorTestCommand command, int8_t speed);

/** 返回适合 OLED 显示的简短指令名称。 */
const char *MotorTest_GetName(MotorTestCommand command);

#endif
