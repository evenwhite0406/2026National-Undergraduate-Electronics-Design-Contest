#ifndef YAW_PID_TUNE_H
#define YAW_PID_TUNE_H

#include "../Control/task_manager.h"

void YawPidTune_Init(void);
void YawPidTune_Start(uint32_t now_ms);
TaskManagerResult YawPidTune_Update(uint32_t now_ms);
void YawPidTune_Cancel(void);

#endif
