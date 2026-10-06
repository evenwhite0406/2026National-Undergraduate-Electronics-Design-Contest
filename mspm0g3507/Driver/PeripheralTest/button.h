/**
 * @file button.h
 * @brief 外设联调工程使用的四路低电平有效 GPIO 按键读取模块。
 *
 * 典型用法：
 * @code
 * PT_Button_Update(g_ms); // 主循环持续调用
 * if ((PT_Button_TakePressedEdges() & BUTTON_1_MASK) != 0U) {
 *     // 按键1经过消抖后刚刚按下。
 * }
 * @endcode
 *
 * ReadPressedMask 返回值中位值为 1 表示按下。PT_ 前缀用于避免与
 * 工程中其他按键模块发生命名冲突。
 */

#ifndef BUTTON_H
#define BUTTON_H

#include <stdbool.h>
#include <stdint.h>

#define BUTTON_COUNT      (4U)
#define BUTTON_1_MASK     (0x01U)
#define BUTTON_2_MASK     (0x02U)
#define BUTTON_3_MASK     (0x04U)
#define BUTTON_4_MASK     (0x08U)

/* 实际按键可调参数。 */
extern uint32_t BUTTON_DEBOUNCE_MS;
extern uint32_t BUTTON_LONG_PRESS_MS;

/** 主循环持续调用，内部完成逐键消抖、边沿和长按检测。 */
void PT_Button_Update(uint32_t now_ms);

/** 返回消抖后的当前按下状态。 */
uint8_t PT_Button_GetPressedMask(void);

/** 取走本次按下沿/释放沿/长按事件；每个事件只返回一次。 */
uint8_t PT_Button_TakePressedEdges(void);
uint8_t PT_Button_TakeReleasedEdges(void);
uint8_t PT_Button_TakeLongPressEdges(void);

/** 返回指定按键持续按下时间；未按下或编号非法时返回0。 */
uint32_t PT_Button_GetHeldMs(uint8_t button_index, uint32_t now_ms);

/** 返回 SysConfig 中已配置按键的位掩码。 */
uint8_t PT_Button_GetConfiguredMask(void);

/** 返回 GPIO 原始电平：位 0～3 分别对应按键 1～4。 */
uint8_t PT_Button_ReadRawMask(void);

/** 返回低电平有效的按下状态；置位表示对应按键已按下。 */
uint8_t PT_Button_ReadPressedMask(void);

/** 使用便于阅读的编号（1～4）读取消抖后的状态。 */
bool PT_Button_IsPressed(uint8_t button_index);

#endif
