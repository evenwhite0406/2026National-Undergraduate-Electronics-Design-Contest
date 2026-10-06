/**
 * @file angle_hold_test.c
 * @brief 不依赖串口的原地角度保持测试。
 */

#include "angle_hold_test.h"
#include "../Hardware/motor.h"

#define ANGLE_HOLD_DEFAULT_TARGET_X10   (300)

static int16_t g_angle_hold_target_x10 = ANGLE_HOLD_DEFAULT_TARGET_X10;
static uint8_t g_angle_hold_running = 0U;

static int16_t AngleHoldTest_WrapTarget(int32_t target_x10)
{
    while (target_x10 > 1800L) {
        target_x10 -= 3600L;
    }
    while (target_x10 < -1800L) {
        target_x10 += 3600L;
    }
    return (int16_t) target_x10;
}

void AngleHoldTest_Init(void)
{
    g_angle_hold_target_x10 = ANGLE_HOLD_DEFAULT_TARGET_X10;
    g_angle_hold_running = 0U;
}

void AngleHoldTest_SetTargetX10(int16_t target_yaw_x10)
{
    g_angle_hold_target_x10 =
        AngleHoldTest_WrapTarget((int32_t) target_yaw_x10);

    if (g_angle_hold_running != 0U) {
        Motor_DriveHeading(0, g_angle_hold_target_x10);
    }
}

int16_t AngleHoldTest_GetTargetX10(void)
{
    return g_angle_hold_target_x10;
}

void AngleHoldTest_Start(void)
{
    Motor_DriveHeading(0, g_angle_hold_target_x10);
    g_angle_hold_running = 1U;
}

void AngleHoldTest_Stop(void)
{
    Motor_StopHeading();
    g_angle_hold_running = 0U;
}

uint8_t AngleHoldTest_IsRunning(void)
{
    return g_angle_hold_running;
}
