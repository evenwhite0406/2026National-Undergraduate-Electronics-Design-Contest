#include "photoelectric_sensor.h"
#include "ti_msp_dl_config.h"

void PhotoelectricSensor_Init(void)
{
    /* Pinmux, input direction, and pull-ups are initialized by SysConfig. */
}

bool PhotoelectricSensor_IsActive(PhotoelectricSensorId sensor)
{
    uint32_t pin;
    bool level_high;

    if (sensor == PHOTOELECTRIC_SENSOR_1) {
        pin = GPIO_PS_PIN_1_PIN;
    } else if (sensor == PHOTOELECTRIC_SENSOR_2) {
        pin = GPIO_PS_PIN_2_PIN;
    } else {
        return false;
    }

    level_high = (DL_GPIO_readPins(GPIO_PS_PORT, pin) & pin) != 0U;
#if PHOTOELECTRIC_SENSOR_ACTIVE_LOW
    return !level_high;
#else
    return level_high;
#endif
}

uint8_t PhotoelectricSensor_ReadMask(void)
{
    uint8_t mask = 0U;

    if (PhotoelectricSensor_IsActive(PHOTOELECTRIC_SENSOR_1)) {
        mask |= (1U << 0);
    }
    if (PhotoelectricSensor_IsActive(PHOTOELECTRIC_SENSOR_2)) {
        mask |= (1U << 1);
    }

    return mask;
}
