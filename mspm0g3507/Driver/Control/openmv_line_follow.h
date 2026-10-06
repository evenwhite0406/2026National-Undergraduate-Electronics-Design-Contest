/**
 * @file openmv_line_follow.h
 * @brief OpenMV视觉循迹配置和接口。
 */

#ifndef OPENMV_LINE_FOLLOW_CONTROL_H
#define OPENMV_LINE_FOLLOW_CONTROL_H

#include "vision_line_follow.h"

/* 调用：OpenMVLineFollow_Init/Start，主循环Update，结束Stop。 */

/* OpenMV参数独立于K230，后续可分别标定。 */
extern int16_t VISION_LINE_CENTER_X;
extern int16_t VISION_LINE_BASE_SPEED;
extern int16_t VISION_LINE_MIN_SPEED;
extern int16_t VISION_LINE_MAX_SPEED;
extern int32_t VISION_LINE_KP_X1000;
extern uint16_t VISION_LINE_MAX_CENTER_X;
extern uint32_t VISION_LINE_FRAME_TIMEOUT_MS;
#define OPENMV_LINE_CENTER_X         VISION_LINE_CENTER_X
#define OPENMV_LINE_BASE_SPEED       VISION_LINE_BASE_SPEED
#define OPENMV_LINE_MIN_SPEED        VISION_LINE_MIN_SPEED
#define OPENMV_LINE_MAX_SPEED        VISION_LINE_MAX_SPEED
#define OPENMV_LINE_KP_X1000         VISION_LINE_KP_X1000
#define OPENMV_LINE_MAX_CENTER_X     VISION_LINE_MAX_CENTER_X
#define OPENMV_LINE_FRAME_TIMEOUT_MS VISION_LINE_FRAME_TIMEOUT_MS

void OpenMVLineFollow_Init(void);
void OpenMVLineFollow_Start(uint32_t now_ms);
VisionLineState OpenMVLineFollow_Update(uint32_t now_ms);
void OpenMVLineFollow_Stop(void);
VisionLineState OpenMVLineFollow_GetState(void);
const char *OpenMVLineFollow_GetStatusText(void);
uint16_t OpenMVLineFollow_GetCenterX(void);

#endif
