/**
 * @file line_sensor.c
 * @brief 最多八路 SysConfig GPIO 循迹输入的轮询驱动。
 *
 * 所有读取接口均为非阻塞快照。配置掩码可帮助 OLED 联调页面区分
 * “真实的零电平”和“SysConfig 中没有配置该通道”两种情况。
 */

#include "line_sensor.h"
#include "ti_msp_dl_config.h"

#define LINE_SENSOR_FILTER_MAX       (3U)
#define LINE_SENSOR_FILTER_SET_LEVEL (3U)
#define LINE_SENSOR_FILTER_CLR_LEVEL (0U)

static uint8_t g_filter_level[LINE_SENSOR_COUNT];
static uint8_t g_filtered_mask;
static uint8_t g_filter_seeded;

uint8_t LineSensor_GetConfiguredMask(void)
{
    uint8_t mask = 0U;

#ifdef GPIO_LINE_LINE_1_PIN
    mask |= 0x01U;
#endif
#ifdef GPIO_LINE_LINE_2_PIN
    mask |= 0x02U;
#endif
#ifdef GPIO_LINE_LINE_3_PIN
    mask |= 0x04U;
#endif
#ifdef GPIO_LINE_LINE_4_PIN
    mask |= 0x08U;
#endif
#ifdef GPIO_LINE_LINE_5_PIN
    mask |= 0x10U;
#endif
#ifdef GPIO_LINE_LINE_6_PIN
    mask |= 0x20U;
#endif
#ifdef GPIO_LINE_LINE_7_PIN
    mask |= 0x40U;
#endif
#ifdef GPIO_LINE_LINE_8_PIN
    mask |= 0x80U;
#endif

    return mask;
}

uint8_t LineSensor_ReadRawMask(void)
{
    uint8_t mask = 0U;

#ifdef GPIO_LINE_LINE_1_PIN
    if (DL_GPIO_readPins(GPIO_LINE_LINE_1_PORT, GPIO_LINE_LINE_1_PIN) != 0U) {
        mask |= 0x01U;
    }
#endif
#ifdef GPIO_LINE_LINE_2_PIN
    if (DL_GPIO_readPins(GPIO_LINE_LINE_2_PORT, GPIO_LINE_LINE_2_PIN) != 0U) {
        mask |= 0x02U;
    }
#endif
#ifdef GPIO_LINE_LINE_3_PIN
    if (DL_GPIO_readPins(GPIO_LINE_LINE_3_PORT, GPIO_LINE_LINE_3_PIN) != 0U) {
        mask |= 0x04U;
    }
#endif
#ifdef GPIO_LINE_LINE_4_PIN
    if (DL_GPIO_readPins(GPIO_LINE_LINE_4_PORT, GPIO_LINE_LINE_4_PIN) != 0U) {
        mask |= 0x08U;
    }
#endif
#ifdef GPIO_LINE_LINE_5_PIN
    if (DL_GPIO_readPins(GPIO_LINE_LINE_5_PORT, GPIO_LINE_LINE_5_PIN) != 0U) {
        mask |= 0x10U;
    }
#endif
#ifdef GPIO_LINE_LINE_6_PIN
    if (DL_GPIO_readPins(GPIO_LINE_LINE_6_PORT, GPIO_LINE_LINE_6_PIN) != 0U) {
        mask |= 0x20U;
    }
#endif
#ifdef GPIO_LINE_LINE_7_PIN
    if (DL_GPIO_readPins(GPIO_LINE_LINE_7_PORT, GPIO_LINE_LINE_7_PIN) != 0U) {
        mask |= 0x40U;
    }
#endif
#ifdef GPIO_LINE_LINE_8_PIN
    if (DL_GPIO_readPins(GPIO_LINE_LINE_8_PORT, GPIO_LINE_LINE_8_PIN) != 0U) {
        mask |= 0x80U;
    }
#endif

    return mask;
}

uint8_t LineSensor_ReadActiveMask(void)
{
    uint8_t raw = LineSensor_ReadRawMask();
    uint8_t configured = LineSensor_GetConfiguredMask();

    if (LINE_SENSOR_ACTIVE_LOW != 0U) {
        /* 大多数比较器模块检测到黑线时会把数字输出拉低。 */
        return (uint8_t) ((~raw) & configured);
    }
    return (uint8_t) (raw & configured);
}

void LineSensor_FilterReset(void)
{
    uint8_t i;

    for (i = 0U; i < LINE_SENSOR_COUNT; i++) {
        g_filter_level[i] = 0U;
    }
    g_filtered_mask = 0U;
    g_filter_seeded = 0U;
}

uint8_t LineSensor_ReadFilteredMask(void)
{
    return LineSensor_FilterSample(LineSensor_ReadActiveMask());
}

uint8_t LineSensor_FilterSample(uint8_t sample)
{
    uint8_t bit;

    if (g_filter_seeded == 0U) {
        for (bit = 0U; bit < LINE_SENSOR_COUNT; bit++) {
            uint8_t pin_mask = (uint8_t) (1U << bit);
            g_filter_level[bit] = ((sample & pin_mask) != 0U)
                                      ? LINE_SENSOR_FILTER_MAX : 0U;
        }
        g_filtered_mask = sample;
        g_filter_seeded = 1U;
        return sample;
    }

    for (bit = 0U; bit < LINE_SENSOR_COUNT; bit++) {
        uint8_t pin_mask = (uint8_t) (1U << bit);

        if ((sample & pin_mask) != 0U) {
            if (g_filter_level[bit] < LINE_SENSOR_FILTER_MAX) {
                g_filter_level[bit]++;
            }
        } else if (g_filter_level[bit] > 0U) {
            g_filter_level[bit]--;
        }

        if (g_filter_level[bit] >= LINE_SENSOR_FILTER_SET_LEVEL) {
            g_filtered_mask |= pin_mask;
        } else if (g_filter_level[bit] <= LINE_SENSOR_FILTER_CLR_LEVEL) {
            g_filtered_mask &= (uint8_t) ~pin_mask;
        }
    }

    return g_filtered_mask;
}

uint8_t LineSensor_CountActive(uint8_t mask)
{
    uint8_t count = 0U;

    while (mask != 0U) {
        count = (uint8_t) (count + (mask & 1U));
        mask >>= 1U;
    }
    return count;
}

void LineSensor_FormatBits(uint8_t mask, char *out)
{
    uint8_t i;

    if (out == 0) {
        return;
    }

    /* 输出顺序从 LINE_1 开始，与 OLED 页面上的标识保持一致。 */
    for (i = 0U; i < LINE_SENSOR_COUNT; i++) {
        out[i] = ((mask & (uint8_t) (1U << i)) != 0U) ? '1' : '0';
    }
    out[LINE_SENSOR_COUNT] = '\0';
}
