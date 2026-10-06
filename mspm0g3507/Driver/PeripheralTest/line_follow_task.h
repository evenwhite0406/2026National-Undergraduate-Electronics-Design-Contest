#ifndef LINE_FOLLOW_TASK_H
#define LINE_FOLLOW_TASK_H

#include <stdint.h>
#include "../Control/task_manager.h"

void LineFollowTask_Init(void);
void LineFollowTask_Start(uint32_t now_ms);
TaskManagerResult LineFollowTask_Update(uint32_t now_ms);
void LineFollowTask_Cancel(void);

#endif
