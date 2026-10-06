/**
 * @file buzzer.c
 * @brief GPIO 有源蜂鸣器实现。
 */

#include "buzzer.h"
#include "ti_msp_dl_config.h"

static bool g_buzzer_on;
static bool g_buzzer_timed;
static uint32_t g_buzzer_stop_ms;

bool Buzzer_IsConfigured(void)
{
#if defined(GPIO_BUZZER_BUZZER_PORT) && defined(GPIO_BUZZER_BUZZER_PIN)
    return true;
#else
    return false;
#endif
}

void Buzzer_Set(bool on)
{
#if defined(GPIO_BUZZER_BUZZER_PORT) && defined(GPIO_BUZZER_BUZZER_PIN)
    if ((on && (BUZZER_ACTIVE_HIGH != 0U)) ||
        (!on && (BUZZER_ACTIVE_HIGH == 0U))) {
        DL_GPIO_setPins(GPIO_BUZZER_BUZZER_PORT, GPIO_BUZZER_BUZZER_PIN);
    } else {
        DL_GPIO_clearPins(GPIO_BUZZER_BUZZER_PORT, GPIO_BUZZER_BUZZER_PIN);
    }
    g_buzzer_on = on;
#else
    (void) on;
    g_buzzer_on = false;
#endif
}

void Buzzer_Init(void)
{
    g_buzzer_timed = false;
    g_buzzer_stop_ms = 0U;
    Buzzer_Set(false);
}

void Buzzer_On(void)
{
    g_buzzer_timed = false;
    Buzzer_Set(true);
}

void Buzzer_Off(void)
{
    g_buzzer_timed = false;
    Buzzer_Set(false);
}

void Buzzer_Toggle(void)
{
    g_buzzer_timed = false;
    Buzzer_Set(!g_buzzer_on);
}

void Buzzer_Beep(uint32_t duration_ms, uint32_t now_ms)
{
    if (duration_ms == 0U) {
        Buzzer_Off();
        return;
    }

    Buzzer_Set(true);
    g_buzzer_stop_ms = now_ms + duration_ms;
    g_buzzer_timed = true;
}

void Buzzer_Service(uint32_t now_ms)
{
    /* 有符号差值写法可正确处理32位毫秒计数回绕。 */
    if (g_buzzer_timed && ((int32_t) (now_ms - g_buzzer_stop_ms) >= 0)) {
        g_buzzer_timed = false;
        Buzzer_Set(false);
    }
}

void Buzzer_Cancel(void)
{
    Buzzer_Off();
}

bool Buzzer_IsOn(void)
{
    return g_buzzer_on;
}
