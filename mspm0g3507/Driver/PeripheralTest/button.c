/**
 * @file button.c
 * @brief 支持 SysConfig 配置的四路低电平有效按键轮询实现。
 *
 * 每一路 GPIO 访问都采用条件编译，因此配置不足四个按键时工程仍可
 * 正常编译，未配置按键对应的位始终为零。
 */

#include "button.h"
#include "ti_msp_dl_config.h"

#ifdef GPIO_BUTTON_BUTTON_1_PIN
#ifndef GPIO_BUTTON_BUTTON_1_PORT
#define GPIO_BUTTON_BUTTON_1_PORT GPIO_BUTTON_PORT
#endif
#endif

static uint8_t g_button_initialized;
static uint8_t g_candidate_mask;
static uint8_t g_stable_mask;
static uint8_t g_pressed_edges;
static uint8_t g_released_edges;
static uint8_t g_long_press_edges;
static uint8_t g_long_reported_mask;
static uint32_t g_candidate_change_ms[BUTTON_COUNT];
static uint32_t g_press_start_ms[BUTTON_COUNT];

#ifdef GPIO_BUTTON_BUTTON_2_PIN
#ifndef GPIO_BUTTON_BUTTON_2_PORT
#define GPIO_BUTTON_BUTTON_2_PORT GPIO_BUTTON_PORT
#endif
#endif

#ifdef GPIO_BUTTON_BUTTON_3_PIN
#ifndef GPIO_BUTTON_BUTTON_3_PORT
#define GPIO_BUTTON_BUTTON_3_PORT GPIO_BUTTON_PORT
#endif
#endif

#ifdef GPIO_BUTTON_BUTTON_4_PIN
#ifndef GPIO_BUTTON_BUTTON_4_PORT
#define GPIO_BUTTON_BUTTON_4_PORT GPIO_BUTTON_PORT
#endif
#endif

uint8_t PT_Button_GetConfiguredMask(void)
{
    uint8_t mask = 0U;

#ifdef GPIO_BUTTON_BUTTON_1_PIN
    mask |= BUTTON_1_MASK;
#endif
#ifdef GPIO_BUTTON_BUTTON_2_PIN
    mask |= BUTTON_2_MASK;
#endif
#ifdef GPIO_BUTTON_BUTTON_3_PIN
    mask |= BUTTON_3_MASK;
#endif
#ifdef GPIO_BUTTON_BUTTON_4_PIN
    mask |= BUTTON_4_MASK;
#endif

    return mask;
}

uint8_t PT_Button_ReadRawMask(void)
{
    uint8_t mask = 0U;

#ifdef GPIO_BUTTON_BUTTON_1_PIN
    if (DL_GPIO_readPins(GPIO_BUTTON_BUTTON_1_PORT,
                         GPIO_BUTTON_BUTTON_1_PIN) != 0U) {
        mask |= BUTTON_1_MASK;
    }
#endif
#ifdef GPIO_BUTTON_BUTTON_2_PIN
    if (DL_GPIO_readPins(GPIO_BUTTON_BUTTON_2_PORT,
                         GPIO_BUTTON_BUTTON_2_PIN) != 0U) {
        mask |= BUTTON_2_MASK;
    }
#endif
#ifdef GPIO_BUTTON_BUTTON_3_PIN
    if (DL_GPIO_readPins(GPIO_BUTTON_BUTTON_3_PORT,
                         GPIO_BUTTON_BUTTON_3_PIN) != 0U) {
        mask |= BUTTON_3_MASK;
    }
#endif
#ifdef GPIO_BUTTON_BUTTON_4_PIN
    if (DL_GPIO_readPins(GPIO_BUTTON_BUTTON_4_PORT,
                         GPIO_BUTTON_BUTTON_4_PIN) != 0U) {
        mask |= BUTTON_4_MASK;
    }
#endif

    return mask;
}

uint8_t PT_Button_ReadPressedMask(void)
{
    uint8_t configured = PT_Button_GetConfiguredMask();

    /* 上拉输入按下时读取为 0，因此只反转已经配置的按键位。 */
    return (uint8_t) ((~PT_Button_ReadRawMask()) & configured);
}

void PT_Button_Update(uint32_t now_ms)
{
    uint8_t raw = PT_Button_ReadPressedMask();
    uint8_t index;

    if (g_button_initialized == 0U) {
        g_candidate_mask = raw;
        g_stable_mask = 0U;
        g_pressed_edges = 0U;
        g_released_edges = 0U;
        g_long_press_edges = 0U;
        g_long_reported_mask = 0U;
        for (index = 0U; index < BUTTON_COUNT; index++) {
            g_candidate_change_ms[index] = now_ms;
            g_press_start_ms[index] = now_ms;
        }
        g_button_initialized = 1U;
    }

    for (index = 0U; index < BUTTON_COUNT; index++) {
        uint8_t bit = (uint8_t) (1U << index);
        uint8_t raw_bit = raw & bit;
        uint8_t candidate_bit = g_candidate_mask & bit;
        uint8_t stable_bit = g_stable_mask & bit;

        if (raw_bit != candidate_bit) {
            g_candidate_mask ^= bit;
            g_candidate_change_ms[index] = now_ms;
            candidate_bit = raw_bit;
        }

        if ((candidate_bit != stable_bit) &&
            ((now_ms - g_candidate_change_ms[index]) >=
             BUTTON_DEBOUNCE_MS)) {
            if (candidate_bit != 0U) {
                g_stable_mask |= bit;
                g_pressed_edges |= bit;
                g_press_start_ms[index] = now_ms;
                g_long_reported_mask &= (uint8_t) ~bit;
            } else {
                g_stable_mask &= (uint8_t) ~bit;
                g_released_edges |= bit;
                g_long_reported_mask &= (uint8_t) ~bit;
            }
        }

        if (((g_stable_mask & bit) != 0U) &&
            ((g_long_reported_mask & bit) == 0U) &&
            ((now_ms - g_press_start_ms[index]) >=
             BUTTON_LONG_PRESS_MS)) {
            g_long_press_edges |= bit;
            g_long_reported_mask |= bit;
        }
    }
}

uint8_t PT_Button_GetPressedMask(void)
{
    return g_stable_mask;
}

uint8_t PT_Button_TakePressedEdges(void)
{
    uint8_t edges = g_pressed_edges;
    g_pressed_edges = 0U;
    return edges;
}

uint8_t PT_Button_TakeReleasedEdges(void)
{
    uint8_t edges = g_released_edges;
    g_released_edges = 0U;
    return edges;
}

uint8_t PT_Button_TakeLongPressEdges(void)
{
    uint8_t edges = g_long_press_edges;
    g_long_press_edges = 0U;
    return edges;
}

uint32_t PT_Button_GetHeldMs(uint8_t button_index, uint32_t now_ms)
{
    uint8_t bit;

    if ((button_index == 0U) || (button_index > BUTTON_COUNT)) return 0U;
    bit = (uint8_t) (1U << (button_index - 1U));
    if ((g_stable_mask & bit) == 0U) return 0U;
    return now_ms - g_press_start_ms[button_index - 1U];
}

bool PT_Button_IsPressed(uint8_t button_index)
{
    if ((button_index == 0U) || (button_index > BUTTON_COUNT)) {
        return false;
    }

    return (g_stable_mask & (uint8_t) (1U << (button_index - 1U))) != 0U;
}
