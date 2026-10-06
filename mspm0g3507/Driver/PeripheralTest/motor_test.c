/**
 * @file motor_test.c
 * @brief 基于 Driver/Hardware/motor.c 的电机联调序列。
 *
 * 本模块不负责配置 GPIO 或 PWM，必须先执行 SYSCFG_DL_init()，并链接
 * 速度指令由闭环运动控制器转换成PWM，不直接操作底层占空比。
 */

#include "motor_test.h"
#include "../Control/motion_control.h"

#ifndef MOTOR_LEFT
#define MOTOR_LEFT   (0U)
#endif

#ifndef MOTOR_RIGHT
#define MOTOR_RIGHT  (1U)
#endif

static int8_t MotorTest_AbsSpeed(int8_t speed)
{
    if (speed < 0) {
        speed = (int8_t) -speed;
    }

    if (speed > 100) {
        speed = 100;
    }

    return speed;
}

void MotorTest_Init(void)
{
    MotorTest_Stop();
}

void MotorTest_Stop(void)
{
    MotionControl_Stop();
}

void MotorTest_Drive(MotorTestCommand command, int8_t speed_command)
{
    int8_t speed = MotorTest_AbsSpeed(speed_command);

    if ((command == MOTOR_TEST_STOP) || (speed == 0)) {
        MotorTest_Stop();
        return;
    }

    MotionControl_Start();

    switch (command) {
    case MOTOR_TEST_LEFT_FORWARD:
        MotionControl_SetWheelSpeeds(speed, 0);
        break;
    case MOTOR_TEST_LEFT_BACKWARD:
        MotionControl_SetWheelSpeeds((int16_t) -speed, 0);
        break;
    case MOTOR_TEST_RIGHT_FORWARD:
        MotionControl_SetWheelSpeeds(0, speed);
        break;
    case MOTOR_TEST_RIGHT_BACKWARD:
        MotionControl_SetWheelSpeeds(0, (int16_t) -speed);
        break;
    case MOTOR_TEST_BOTH_FORWARD:
        MotionControl_SetWheelSpeeds(speed, speed);
        break;
    case MOTOR_TEST_BOTH_BACKWARD:
        MotionControl_SetWheelSpeeds((int16_t) -speed,
                                     (int16_t) -speed);
        break;
    default:
        MotorTest_Stop();
        break;
    }
}

const char *MotorTest_GetName(MotorTestCommand command)
{
    switch (command) {
    case MOTOR_TEST_LEFT_FORWARD:
        return "LEFT +";
    case MOTOR_TEST_LEFT_BACKWARD:
        return "LEFT -";
    case MOTOR_TEST_RIGHT_FORWARD:
        return "RIGHT+";
    case MOTOR_TEST_RIGHT_BACKWARD:
        return "RIGHT-";
    case MOTOR_TEST_BOTH_FORWARD:
        return "BOTH +";
    case MOTOR_TEST_BOTH_BACKWARD:
        return "BOTH -";
    default:
        return "STOP";
    }
}
