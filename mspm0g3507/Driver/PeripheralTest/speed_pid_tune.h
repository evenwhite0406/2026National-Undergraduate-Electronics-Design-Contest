#ifndef SPEED_PID_TUNE_H
#define SPEED_PID_TUNE_H

#include "../Control/task_manager.h"

void SpeedPidTune_Init(void);
void SpeedPidTune_Start(uint32_t now_ms);
TaskManagerResult SpeedPidTune_Update(uint32_t now_ms);
void SpeedPidTune_Cancel(void);

#endif
