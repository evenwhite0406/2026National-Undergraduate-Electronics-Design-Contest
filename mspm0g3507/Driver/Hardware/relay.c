#include "relay.h"
#include "ti_msp_dl_config.h"

static bool g_relay_on;

void Relay_Init(void)
{
    Relay_Off();
}

void Relay_Set(bool on)
{
#if RELAY_ACTIVE_HIGH
    if (on) {
        DL_GPIO_setPins(GPIO_RELAY_PORT, GPIO_RELAY_PIN_0_PIN);
    } else {
        DL_GPIO_clearPins(GPIO_RELAY_PORT, GPIO_RELAY_PIN_0_PIN);
    }
#else
    if (on) {
        DL_GPIO_clearPins(GPIO_RELAY_PORT, GPIO_RELAY_PIN_0_PIN);
    } else {
        DL_GPIO_setPins(GPIO_RELAY_PORT, GPIO_RELAY_PIN_0_PIN);
    }
#endif
    g_relay_on = on;
}

void Relay_On(void)
{
    Relay_Set(true);
}

void Relay_Off(void)
{
    Relay_Set(false);
}

void Relay_Toggle(void)
{
    Relay_Set(!g_relay_on);
}

bool Relay_IsOn(void)
{
    return g_relay_on;
}
