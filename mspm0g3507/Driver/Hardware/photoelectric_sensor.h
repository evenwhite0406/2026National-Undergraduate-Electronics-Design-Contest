#ifndef PHOTOELECTRIC_SENSOR_H
#define PHOTOELECTRIC_SENSOR_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    PHOTOELECTRIC_SENSOR_1 = 0,
    PHOTOELECTRIC_SENSOR_2
} PhotoelectricSensorId;

/* Most open-collector photoelectric modules pull the signal low when active. */
#define PHOTOELECTRIC_SENSOR_ACTIVE_LOW (1U)

void PhotoelectricSensor_Init(void);
bool PhotoelectricSensor_IsActive(PhotoelectricSensorId sensor);
uint8_t PhotoelectricSensor_ReadMask(void);

#endif
