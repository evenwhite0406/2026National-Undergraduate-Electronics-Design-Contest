/**
 * @file led.c
 * @brief SysConfig GPIO_LED 三路状态灯驱动。
 */

#include "led.h"
#include "ti_msp_dl_config.h"

static void LED_Write(GPIO_Regs *port, uint32_t pin, bool on)
{
    if ((on && (LED_ACTIVE_HIGH != 0U)) ||
        (!on && (LED_ACTIVE_HIGH == 0U))) {
        DL_GPIO_setPins(port, pin);
    } else {
        DL_GPIO_clearPins(port, pin);
    }
}

static bool LED_GetPin(LED_Color color, GPIO_Regs **port, uint32_t *pin)
{
    switch (color) {
        case LED_RED:
            *port = GPIO_LED_RED_PORT;
            *pin  = GPIO_LED_RED_PIN;
            return true;

        case LED_YELLOW:
            *port = GPIO_LED_YELLOW_PORT;
            *pin  = GPIO_LED_YELLOW_PIN;
            return true;

        case LED_BLUE:
            *port = GPIO_LED_BLUE_PORT;
            *pin  = GPIO_LED_BLUE_PIN;
            return true;

        default:
            return false;
    }
}

void LED_Init(void)
{
    LED_AllOff();
}

void LED_Set(LED_Color color, bool on)
{
    GPIO_Regs *port;
    uint32_t pin;

    if (LED_GetPin(color, &port, &pin)) {
        LED_Write(port, pin, on);
    }
}

void LED_On(LED_Color color)
{
    LED_Set(color, true);
}

void LED_Off(LED_Color color)
{
    LED_Set(color, false);
}

void LED_Toggle(LED_Color color)
{
    GPIO_Regs *port;
    uint32_t pin;

    if (LED_GetPin(color, &port, &pin)) {
        DL_GPIO_togglePins(port, pin);
    }
}

void LED_SetAll(bool red_on, bool yellow_on, bool blue_on)
{
    LED_Set(LED_RED, red_on);
    LED_Set(LED_YELLOW, yellow_on);
    LED_Set(LED_BLUE, blue_on);
}

void LED_AllOn(void)
{
    LED_SetAll(true, true, true);
}

void LED_AllOff(void)
{
    LED_SetAll(false, false, false);
}
