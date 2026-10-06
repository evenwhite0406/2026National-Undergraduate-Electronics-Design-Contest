/**
 * @file k230_line_follow.h
 * @brief K230视觉循迹配置和接口。
 */

#ifndef K230_LINE_FOLLOW_H
#define K230_LINE_FOLLOW_H

#include "vision_line_follow.h"

/* 调用：K230LineFollow_Init/Start，主循环Update，结束Stop。 */

/* 以下参数均为待实车调节值，统一放在这里便于修改。 */
extern int16_t VISION_LINE_CENTER_X;
extern int16_t VISION_LINE_BASE_SPEED;
extern int16_t VISION_LINE_MIN_SPEED;
extern int16_t VISION_LINE_MAX_SPEED;
extern int32_t VISION_LINE_KP_X1000;
extern uint16_t VISION_LINE_MAX_CENTER_X;
extern uint32_t VISION_LINE_FRAME_TIMEOUT_MS;
#define K230_LINE_CENTER_X         VISION_LINE_CENTER_X
#define K230_LINE_BASE_SPEED       VISION_LINE_BASE_SPEED
#define K230_LINE_MIN_SPEED        VISION_LINE_MIN_SPEED
#define K230_LINE_MAX_SPEED        VISION_LINE_MAX_SPEED
#define K230_LINE_KP_X1000         VISION_LINE_KP_X1000
#define K230_LINE_MAX_CENTER_X     VISION_LINE_MAX_CENTER_X
#define K230_LINE_FRAME_TIMEOUT_MS VISION_LINE_FRAME_TIMEOUT_MS

void K230LineFollow_Init(void);
void K230LineFollow_Start(uint32_t now_ms);
VisionLineState K230LineFollow_Update(uint32_t now_ms);
void K230LineFollow_Stop(void);
VisionLineState K230LineFollow_GetState(void);
const char *K230LineFollow_GetStatusText(void);
uint16_t K230LineFollow_GetCenterX(void);

#endif
