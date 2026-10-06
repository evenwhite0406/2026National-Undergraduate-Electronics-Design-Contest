#ifndef LINE_SENSOR_BT_TEST_H
#define LINE_SENSOR_BT_TEST_H

#include <stdint.h>
#include "../Control/task_manager.h"

void LineSensorBtTest_Init(void);

void LineSensorBtTest_Start(uint32_t now_ms);

TaskManagerResult LineSensorBtTest_Update(uint32_t now_ms);

void LineSensorBtTest_Cancel(void);

#endif