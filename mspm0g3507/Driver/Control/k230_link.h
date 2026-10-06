#ifndef K230_LINK_H
#define K230_LINK_H

#include <stdint.h>

typedef enum {
    K230_LINK_STATE_OFFLINE = 0,
    K230_LINK_STATE_IDLE,
    K230_LINK_STATE_PREPARING,
    K230_LINK_STATE_CENTERING_TASK6,
    K230_LINK_STATE_SETTING_TASK6,
    K230_LINK_STATE_READY,
    K230_LINK_STATE_RUNNING,
    K230_LINK_STATE_RETURNING,
    K230_LINK_STATE_ERROR
} K230LinkState;

void K230Link_Init(uint32_t now_ms);

/*
 * Main-loop communication state machine. selected_task is one based.
 * It parses K230 frames and retries explicit task-transition commands.
 * Menu arguments are retained for the existing call site, but browsing the
 * menu never starts K230 ball control.
 */
void K230Link_Update(uint32_t now_ms,
                     uint8_t menu_idle,
                     uint8_t selected_task);

void K230Link_RequestPrepare(uint8_t task, uint32_t now_ms);
void K230Link_RequestTask6Center(uint32_t now_ms);

/* StartTask only grants the start when the matching READY was received. */
uint8_t K230Link_StartTask(uint8_t task, uint32_t now_ms);
void K230Link_Stop(uint32_t now_ms);

/*
 * Optional launch-feedforward side channel.  These non-retried frames never
 * replace P/G/C6/S task control, so older K230 firmware can ignore them.
 * Acceleration units are car speed-percent/s; ff_x10 is 0.1 rod degree.
 */
uint8_t K230Link_SendFeedforwardConfig(uint8_t sequence,
                                       uint32_t lead_ms,
                                       uint32_t ramp_ms,
                                       int32_t gain_x100,
                                       int32_t damp_x100,
                                       int16_t max_x10,
                                       int16_t sign);
uint8_t K230Link_SendFeedforwardSample(uint8_t sequence,
                                       int16_t ff_x10,
                                       int16_t reference_accel,
                                       int16_t measured_accel);
uint8_t K230Link_SendFeedforwardEnd(uint8_t sequence);

uint8_t K230Link_IsReady(uint8_t task, uint32_t now_ms);
uint8_t K230Link_IsOnline(uint32_t now_ms);
uint8_t K230Link_ConsumeDone(uint8_t task);

K230LinkState K230Link_GetState(void);
uint8_t K230Link_GetReadyTask(void);
uint8_t K230Link_IsTask6CenterReady(void);
uint8_t K230Link_HasBallPosition(void);
int16_t K230Link_GetBallPositionX10Mm(void);
int16_t K230Link_GetTask6TargetX10Mm(void);
const char *K230Link_GetStatusText(uint32_t now_ms);

#endif /* K230_LINK_H */
