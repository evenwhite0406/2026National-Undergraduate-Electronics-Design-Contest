/**
 * @file buzzer.h
 * @brief 有源蜂鸣器 GPIO 驱动，支持常开关控制和非阻塞定时鸣叫。
 *
 * SysConfig 配置要求：
 *   GPIO 组名：GPIO_BUZZER
 *   输出引脚名：BUZZER
 *
 * 使用方法：
 * @code
 * Buzzer_Init();
 * Buzzer_On();
 * Buzzer_Off();
 *
 * Buzzer_Beep(100U, g_ms); // 响100ms，立即返回
 * Buzzer_Service(g_ms);    // 主循环中持续调用，时间到后自动停止
 * @endcode
 */

#ifndef HARDWARE_BUZZER_H
#define HARDWARE_BUZZER_H

#include <stdbool.h>
#include <stdint.h>

/* ======================== 实车可调参数 ======================== */
/* 1：GPIO 输出高电平时鸣响；0：GPIO 输出低电平时鸣响。 */
extern uint8_t BUZZER_ACTIVE_HIGH;

/* 常用提示音持续时间，可直接传给 Buzzer_Beep()。 */
extern uint32_t BUZZER_SHORT_BEEP_MS;
extern uint32_t BUZZER_LONG_BEEP_MS;

/** 初始化并关闭蜂鸣器。须在 DL_MSPM0G3507_init() 之后调用。 */
void Buzzer_Init(void);

/** 直接控制蜂鸣器。 */
void Buzzer_On(void);
void Buzzer_Off(void);
void Buzzer_Toggle(void);
void Buzzer_Set(bool on);

/**
 * 启动一次非阻塞鸣叫。
 * @param duration_ms 鸣叫时间；传0等同于关闭。
 * @param now_ms 当前系统毫秒计数，例如 g_ms。
 */
void Buzzer_Beep(uint32_t duration_ms, uint32_t now_ms);

/** 主循环持续调用，负责在定时时间到达后关闭蜂鸣器。 */
void Buzzer_Service(uint32_t now_ms);

/** 取消定时鸣叫并立即关闭。 */
void Buzzer_Cancel(void);

/** 返回 SysConfig 中是否存在 GPIO_BUZZER/BUZZER 引脚。 */
bool Buzzer_IsConfigured(void);

/** 返回当前软件记录的鸣响状态。 */
bool Buzzer_IsOn(void);

#endif
