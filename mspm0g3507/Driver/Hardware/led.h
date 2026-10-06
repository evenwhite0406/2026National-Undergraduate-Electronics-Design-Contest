/**
 * @file led.h
 * @brief 三色状态 LED 封装：LED1=红，LED2=黄，LED3=蓝。
 *
 * 使用方法：
 * @code
 * LED_Init();                 // DL_MSPM0G3507_init() 之后调用一次
 * LED_On(LED_RED);            // 点亮红灯
 * LED_Off(LED_RED);           // 熄灭红灯
 * LED_Set(LED_YELLOW, true);  // 设置黄灯状态
 * LED_Toggle(LED_BLUE);       // 翻转蓝灯
 * LED_AllOff();               // 熄灭全部 LED
 * @endcode
 */

#ifndef HARDWARE_LED_H
#define HARDWARE_LED_H

#include <stdbool.h>
#include <stdint.h>

/* ======================== 实车可调参数 ======================== */
/* 1：GPIO 输出高电平时点亮；0：GPIO 输出低电平时点亮。 */
extern uint8_t LED_ACTIVE_HIGH;

/* LED 编号与颜色固定对应，值也可直接使用 1、2、3。 */
typedef enum {
    LED_RED    = 1U, /* LED1：红灯 */
    LED_YELLOW = 2U, /* LED2：黄灯 */
    LED_BLUE   = 3U  /* LED3：蓝灯 */
} LED_Color;

/** 初始化 LED 状态。须在 DL_MSPM0G3507_init() 之后调用。 */
void LED_Init(void);

/** 控制指定 LED；编号无效时不执行操作。 */
void LED_Set(LED_Color color, bool on);
void LED_On(LED_Color color);
void LED_Off(LED_Color color);
void LED_Toggle(LED_Color color);

/** 一次性设置三路 LED，参数顺序为红、黄、蓝。 */
void LED_SetAll(bool red_on, bool yellow_on, bool blue_on);
void LED_AllOn(void);
void LED_AllOff(void);

#endif
