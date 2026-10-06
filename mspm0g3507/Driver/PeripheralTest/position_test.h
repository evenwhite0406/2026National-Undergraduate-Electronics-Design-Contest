#ifndef POSITION_TEST_H
#define POSITION_TEST_H

#include <stdint.h>
#include "../Control/task_manager.h"

void PositionTest_Init(void);

void PositionTest_Start(uint32_t now_ms);

TaskManagerResult PositionTest_Update(uint32_t now_ms);

void PositionTest_Cancel(void);

#endif