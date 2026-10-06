#ifndef ANGLE_HOLD_TEST_H
#define ANGLE_HOLD_TEST_H

#include <stdint.h>

/*
 * 角度单位为0.1度：
 *   300  = +30.0°
 *  -900  = -90.0°
 *     0  = 保持BMI088当前零点方向
 */

void AngleHoldTest_Init(void);
void AngleHoldTest_SetTargetX10(int16_t target_yaw_x10);
int16_t AngleHoldTest_GetTargetX10(void);
void AngleHoldTest_Start(void);
void AngleHoldTest_Stop(void);
uint8_t AngleHoldTest_IsRunning(void);

#endif
