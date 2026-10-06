/**
 * @file line_sensor.h
 * @brief 八路数字灰度/循迹输入模块。
 *
 * 典型用法：
 * @code
 * uint8_t active = LineSensor_ReadActiveMask();
 * char text[LINE_SENSOR_COUNT + 1U];
 * LineSensor_FormatBits(active, text);  // 示例："00111000"
 * @endcode
 *
 * 位 0 表示 LINE_1，位 7 表示 LINE_8。如果传感器检测到黑线时输出
 * 高电平，请修改 LINE_SENSOR_ACTIVE_LOW。
 */

#ifndef LINE_SENSOR_H
#define LINE_SENSOR_H

#include <stdint.h>

#define LINE_SENSOR_COUNT       (8U)
/* 当前按黑线时IO输出低电平配置；若实测为高电平必须改成0。 */
extern uint8_t LINE_SENSOR_ACTIVE_LOW;

/** 返回 SysConfig 中已经配置的循迹通道位掩码。 */
uint8_t LineSensor_GetConfiguredMask(void);

/** 读取八路 GPIO 的物理电平，不做极性转换。 */
uint8_t LineSensor_ReadRawMask(void);

/** 读取逻辑检测结果；置位表示对应通道检测到黑线。 */
uint8_t LineSensor_ReadActiveMask(void);

/** Reset the per-channel saturating debounce filter. */
void LineSensor_FilterReset(void);

/**
 * Read a debounced active mask. Each channel uses a four-level saturating
 * integrator with hysteresis, which rejects single-sample comparator chatter.
 */
uint8_t LineSensor_ReadFilteredMask(void);

/** Feed an already sampled active mask into the debounce filter. */
uint8_t LineSensor_FilterSample(uint8_t active_mask);

/** Return the number of asserted channels in a mask. */
uint8_t LineSensor_CountActive(uint8_t mask);

/** 将掩码格式化为八个 ASCII 0/1 字符，并在末尾添加空字符。 */
void LineSensor_FormatBits(uint8_t mask, char *out);

#endif
