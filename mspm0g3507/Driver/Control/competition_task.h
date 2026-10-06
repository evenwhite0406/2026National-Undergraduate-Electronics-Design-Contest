#ifndef COMPETITION_TASK_H
#define COMPETITION_TASK_H

#include <stdint.h>

#include "task_manager.h"

void CompetitionTask_Init(void);

void CompetitionTask1_Start(uint32_t now_ms);
void CompetitionTask2_Start(uint32_t now_ms);
void CompetitionTask3_Start(uint32_t now_ms);
void CompetitionTask4_Start(uint32_t now_ms);
void CompetitionTask5_Start(uint32_t now_ms);
void CompetitionTask6_Start(uint32_t now_ms);
void CompetitionTask7_Start(uint32_t now_ms);
void CompetitionTask8_Start(uint32_t now_ms);

TaskManagerResult CompetitionTask_Update(uint32_t now_ms);
void CompetitionTask_Cancel(void);

#endif /* COMPETITION_TASK_H */
