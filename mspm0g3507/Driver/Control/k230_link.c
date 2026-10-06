#include "k230_link.h"

#include "../Hardware/UART3_OPENMV/OPENMV.h"

#include <stdio.h>
#include <string.h>

#define K230_FRAME_PAYLOAD_MAX       (15U)
#define K230_COMMAND_RETRY_MS        (200U)
#define K230_ONLINE_TIMEOUT_MS       (500U)

typedef enum {
    K230_PENDING_NONE = 0,
    K230_PENDING_PREPARE,
    K230_PENDING_CENTER_TASK6,
    K230_PENDING_START,
    K230_PENDING_STOP
} K230PendingType;

static K230LinkState s_state;
static K230PendingType s_pending_type;
static uint8_t s_pending_task;
static char s_pending_command[8];
static uint32_t s_last_command_ms;

static uint8_t s_seen_rx;
static uint32_t s_last_rx_ms;
static uint8_t s_ready_task;
static uint8_t s_running_task;
static uint8_t s_done_flags;
static uint8_t s_task6_center_ready;
static uint8_t s_task6_prepare_after_center;
static int16_t s_task6_target_x10_mm;
static int16_t s_ball_position_x10_mm;
static uint8_t s_ball_position_valid;

static char s_rx_payload[K230_FRAME_PAYLOAD_MAX + 1U];
static uint8_t s_rx_length;
static uint8_t s_rx_in_frame;

static uint8_t K230Link_IsTaskControlled(uint8_t task)
{
    return ((task >= 3U) && (task <= 6U)) ? 1U : 0U;
}

uint8_t K230Link_SendFeedforwardConfig(uint8_t sequence,
                                       uint32_t lead_ms,
                                       uint32_t ramp_ms,
                                       int32_t gain_x100,
                                       int32_t damp_x100,
                                       int16_t max_x10,
                                       int16_t sign)
{
    char frame[48];

    (void) snprintf(frame, sizeof(frame),
        "[FC,%u,%lu,%lu,%ld,%ld,%d,%d]",
        (unsigned int) sequence,
        (unsigned long) lead_ms,
        (unsigned long) ramp_ms,
        (long) gain_x100,
        (long) damp_x100,
        (int) max_x10,
        (sign < 0) ? -1 : 1);
    return OPENMV_WriteString(frame) ? 1U : 0U;
}

uint8_t K230Link_SendFeedforwardSample(uint8_t sequence,
                                       int16_t ff_x10,
                                       int16_t reference_accel,
                                       int16_t measured_accel)
{
    char frame[40];

    (void) snprintf(frame, sizeof(frame), "[FA,%u,%d,%d,%d]",
        (unsigned int) sequence, (int) ff_x10,
        (int) reference_accel, (int) measured_accel);
    return OPENMV_WriteString(frame) ? 1U : 0U;
}

uint8_t K230Link_SendFeedforwardEnd(uint8_t sequence)
{
    char frame[12];

    (void) snprintf(frame, sizeof(frame), "[FE,%u]",
        (unsigned int) sequence);
    return OPENMV_WriteString(frame) ? 1U : 0U;
}

static uint8_t K230Link_ParseDigit(char ch, uint8_t *digit)
{
    if ((ch < '0') || (ch > '9') || (digit == 0)) {
        return 0U;
    }

    *digit = (uint8_t) (ch - '0');
    return 1U;
}

static uint8_t K230Link_ParseSignedX10(const char *text,
                                      uint8_t digits,
                                      int16_t *value)
{
    int32_t magnitude = 0;
    uint8_t i;
    uint8_t digit;

    if ((text == 0) || (value == 0) ||
        ((text[0] != '+') && (text[0] != '-'))) {
        return 0U;
    }

    for (i = 0U; i < digits; i++) {
        if (K230Link_ParseDigit(text[i + 1U], &digit) == 0U) {
            return 0U;
        }
        magnitude = (magnitude * 10) + digit;
    }

    if (magnitude > 32767L) {
        return 0U;
    }

    *value = (text[0] == '-')
        ? (int16_t) (-magnitude)
        : (int16_t) magnitude;
    return 1U;
}

static void K230Link_ClearPending(void)
{
    s_pending_type = K230_PENDING_NONE;
    s_pending_task = 0U;
    s_pending_command[0] = '\0';
}

static uint8_t K230Link_QueueCommand(const char *command,
                                    K230PendingType type,
                                    uint8_t task,
                                    uint32_t now_ms)
{
    uint8_t length;

    if (command == 0) {
        return 0U;
    }

    length = (uint8_t) strlen(command);
    if ((length == 0U) || (length >= sizeof(s_pending_command))) {
        return 0U;
    }

    (void) memcpy(s_pending_command, command, (size_t) length + 1U);
    s_pending_type = type;
    s_pending_task = task;

    if (OPENMV_WriteString(s_pending_command)) {
        s_last_command_ms = now_ms;
        return 1U;
    }

    /* Keep the pending command; Update retries after the TX buffer drains. */
    s_last_command_ms = now_ms - K230_COMMAND_RETRY_MS;
    return 0U;
}

static void K230Link_QueueTaskCommand(char prefix,
                                     uint8_t task,
                                     K230PendingType type,
                                     uint32_t now_ms)
{
    char command[5];

    if (K230Link_IsTaskControlled(task) == 0U) {
        return;
    }

    command[0] = '[';
    command[1] = prefix;
    command[2] = (char) ('0' + task);
    command[3] = ']';
    command[4] = '\0';
    (void) K230Link_QueueCommand(command, type, task, now_ms);
}

static void K230Link_SetReady(uint8_t task)
{
    if (K230Link_IsTaskControlled(task) == 0U) {
        return;
    }

    s_ready_task = task;
    s_running_task = 0U;
    s_state = K230_LINK_STATE_READY;

    if ((s_pending_type == K230_PENDING_PREPARE) &&
        (s_pending_task == task)) {
        K230Link_ClearPending();
    }
}

static void K230Link_HandleNumericPayload(const char *payload,
                                          uint8_t length)
{
    int16_t value;

    /* K230 legacy position frame: [+0123*], in 0.1 mm units. */
    if ((length != 6U) || (payload[5] != '*') ||
        (K230Link_ParseSignedX10(payload, 4U, &value) == 0U)) {
        return;
    }

    if (value == 9999) {
        s_ball_position_valid = 0U;
    } else if (value == 9998) {
        s_done_flags |= (uint8_t) (1U << 3U);
    } else {
        s_ball_position_x10_mm = value;
        s_ball_position_valid = 1U;
    }
}

static void K230Link_HandlePayload(const char *payload, uint8_t length)
{
    uint8_t task;
    int16_t target;

    if ((payload == 0) || (length == 0U)) {
        return;
    }

    s_seen_rx = 1U;

    if (((payload[0] == '+') || (payload[0] == '-')) &&
        (length == 6U)) {
        K230Link_HandleNumericPayload(payload, length);
        return;
    }

    if ((length == 2U) && (payload[0] == 'R') &&
        (payload[1] >= '3') && (payload[1] <= '6')) {
        task = (uint8_t) (payload[1] - '0');
        K230Link_SetReady(task);
        return;
    }

    if ((length == 8U) && (payload[0] == 'R') &&
        (payload[1] == '6') && (payload[2] == ',') &&
        (K230Link_ParseSignedX10(&payload[3], 4U, &target) != 0U)) {
        s_task6_target_x10_mm = target;
        K230Link_SetReady(6U);
        return;
    }

    if ((length == 4U) && (memcmp(payload, "C6OK", 4U) == 0)) {
        s_task6_center_ready = 1U;
        s_ready_task = 0U;
        s_running_task = 0U;
        s_state = K230_LINK_STATE_IDLE;
        if (s_pending_type == K230_PENDING_CENTER_TASK6) {
            K230Link_ClearPending();
        }
        return;
    }

    /*
     * K230 acknowledges a P3/P4/P5 command immediately with PREPn.
     * READY is still reported separately as Rn only after the ball has
     * remained stable, so this acknowledgement never grants car start.
     */
    if ((length == 5U) && (memcmp(payload, "PREP", 4U) == 0) &&
        (payload[4] >= '3') && (payload[4] <= '5')) {
        task = (uint8_t) (payload[4] - '0');
        if ((s_pending_type == K230_PENDING_PREPARE) &&
            (s_pending_task == task)) {
            s_state = K230_LINK_STATE_PREPARING;
        }
        return;
    }

    /* C6 was received and K230 is actively returning the ball to center. */
    if ((length == 7U) && (memcmp(payload, "CENTER6", 7U) == 0)) {
        if (s_pending_type == K230_PENDING_CENTER_TASK6) {
            s_state = K230_LINK_STATE_CENTERING_TASK6;
        }
        return;
    }

    if ((length == 4U) && (memcmp(payload, "SET6", 4U) == 0)) {
        s_state = K230_LINK_STATE_SETTING_TASK6;
        return;
    }

    if ((length == 4U) && (memcmp(payload, "RUN", 3U) == 0) &&
        (payload[3] >= '3') && (payload[3] <= '6')) {
        task = (uint8_t) (payload[3] - '0');
        s_running_task = task;
        s_ready_task = 0U;
        s_state = K230_LINK_STATE_RUNNING;
        if ((s_pending_type == K230_PENDING_START) &&
            (s_pending_task == task)) {
            K230Link_ClearPending();
        }
        return;
    }

    if ((length == 2U) && (payload[0] == 'D') &&
        (payload[1] >= '3') && (payload[1] <= '6')) {
        task = (uint8_t) (payload[1] - '0');
        s_done_flags |= (uint8_t) (1U << task);
        return;
    }

    if ((length == 6U) && (memcmp(payload, "RETURN", 6U) == 0)) {
        s_state = K230_LINK_STATE_RETURNING;
        return;
    }

    if ((length == 4U) && (memcmp(payload, "IDLE", 4U) == 0)) {
        s_state = K230_LINK_STATE_IDLE;
        s_ready_task = 0U;
        s_running_task = 0U;
        if (s_pending_type == K230_PENDING_STOP) {
            K230Link_ClearPending();
        }
        return;
    }

    if ((length >= 3U) && (memcmp(payload, "ERR", 3U) == 0)) {
        s_state = K230_LINK_STATE_ERROR;
        K230Link_ClearPending();
    }
}

static void K230Link_ProcessRx(uint32_t now_ms)
{
    uint8_t data;

    while (OPENMV_ReadByte(&data)) {
        if (data == (uint8_t) '[') {
            s_rx_in_frame = 1U;
            s_rx_length = 0U;
            continue;
        }

        if (s_rx_in_frame == 0U) {
            continue;
        }

        if (data == (uint8_t) ']') {
            s_rx_payload[s_rx_length] = '\0';
            s_last_rx_ms = now_ms;
            K230Link_HandlePayload(s_rx_payload, s_rx_length);
            s_rx_in_frame = 0U;
            s_rx_length = 0U;
            continue;
        }

        if (s_rx_length < K230_FRAME_PAYLOAD_MAX) {
            s_rx_payload[s_rx_length] = (char) data;
            s_rx_length++;
        } else {
            s_rx_in_frame = 0U;
            s_rx_length = 0U;
        }
    }
}

static void K230Link_RetryPending(uint32_t now_ms)
{
    if ((s_pending_type != K230_PENDING_NONE) &&
        ((uint32_t) (now_ms - s_last_command_ms) >=
         K230_COMMAND_RETRY_MS)) {
        if (OPENMV_WriteString(s_pending_command)) {
            s_last_command_ms = now_ms;
        }
    }
}

void K230Link_Init(uint32_t now_ms)
{
    s_state = K230_LINK_STATE_OFFLINE;
    s_pending_type = K230_PENDING_NONE;
    s_pending_task = 0U;
    s_pending_command[0] = '\0';
    s_last_command_ms = now_ms;

    s_seen_rx = 0U;
    s_last_rx_ms = now_ms;
    s_ready_task = 0U;
    s_running_task = 0U;
    s_done_flags = 0U;
    s_task6_center_ready = 0U;
    s_task6_prepare_after_center = 0U;
    s_task6_target_x10_mm = 0;
    s_ball_position_x10_mm = 0;
    s_ball_position_valid = 0U;

    s_rx_payload[0] = '\0';
    s_rx_length = 0U;
    s_rx_in_frame = 0U;

}

void K230Link_Update(uint32_t now_ms,
                     uint8_t menu_idle,
                     uint8_t selected_task)
{
    (void) menu_idle;
    (void) selected_task;

    K230Link_ProcessRx(now_ms);
    K230Link_RetryPending(now_ms);

    /*
     * Menu browsing must never start ball control.  Task 5 completion (or
     * an explicit task-6 entry) requests C6; after C6OK, continue to P6
     * here so target selection is complete before the final K3 launch.
     */
    if ((s_task6_prepare_after_center != 0U) &&
        (s_task6_center_ready != 0U)) {
        s_task6_prepare_after_center = 0U;
        K230Link_RequestPrepare(6U, now_ms);
    }

    if ((K230Link_IsOnline(now_ms) == 0U) &&
        (s_state != K230_LINK_STATE_PREPARING) &&
        (s_state != K230_LINK_STATE_CENTERING_TASK6) &&
        (s_state != K230_LINK_STATE_SETTING_TASK6)) {
        s_ready_task = 0U;
        s_state = K230_LINK_STATE_OFFLINE;
    }
}

void K230Link_RequestPrepare(uint8_t task, uint32_t now_ms)
{
    if (K230Link_IsTaskControlled(task) == 0U) {
        return;
    }

    if ((s_ready_task == task) &&
        (K230Link_IsOnline(now_ms) != 0U)) {
        return;
    }

    if ((s_pending_type == K230_PENDING_PREPARE) &&
        (s_pending_task == task)) {
        return;
    }

    s_ready_task = 0U;
    s_running_task = 0U;
    if (task == 6U) {
        s_task6_center_ready = 0U;
        s_state = K230_LINK_STATE_SETTING_TASK6;
    } else {
        s_state = K230_LINK_STATE_PREPARING;
    }
    K230Link_QueueTaskCommand('P', task, K230_PENDING_PREPARE, now_ms);
}

void K230Link_RequestTask6Center(uint32_t now_ms)
{
    s_task6_prepare_after_center = 1U;

    if (s_pending_type == K230_PENDING_CENTER_TASK6) {
        return;
    }

    s_ready_task = 0U;
    s_running_task = 0U;
    s_task6_center_ready = 0U;
    s_state = K230_LINK_STATE_CENTERING_TASK6;
    (void) K230Link_QueueCommand("[C6]",
                                 K230_PENDING_CENTER_TASK6,
                                 6U,
                                 now_ms);
}

uint8_t K230Link_StartTask(uint8_t task, uint32_t now_ms)
{
    if ((K230Link_IsTaskControlled(task) == 0U) ||
        (K230Link_IsReady(task, now_ms) == 0U)) {
        return 0U;
    }

    s_done_flags &= (uint8_t) ~(1U << task);
    s_ready_task = 0U;
    s_running_task = task;
    s_state = K230_LINK_STATE_RUNNING;
    K230Link_QueueTaskCommand('G', task, K230_PENDING_START, now_ms);
    return 1U;
}

void K230Link_Stop(uint32_t now_ms)
{
    s_ready_task = 0U;
    s_running_task = 0U;
    s_task6_center_ready = 0U;
    s_task6_prepare_after_center = 0U;
    s_state = K230_LINK_STATE_RETURNING;
    (void) K230Link_QueueCommand("[S]", K230_PENDING_STOP, 0U, now_ms);
}

uint8_t K230Link_IsReady(uint8_t task, uint32_t now_ms)
{
    return ((s_ready_task == task) &&
            (K230Link_IsOnline(now_ms) != 0U)) ? 1U : 0U;
}

uint8_t K230Link_IsOnline(uint32_t now_ms)
{
    if (s_seen_rx == 0U) {
        return 0U;
    }

    return ((uint32_t) (now_ms - s_last_rx_ms) <=
            K230_ONLINE_TIMEOUT_MS) ? 1U : 0U;
}

uint8_t K230Link_ConsumeDone(uint8_t task)
{
    uint8_t mask;

    if (task > 7U) {
        return 0U;
    }

    mask = (uint8_t) (1U << task);
    if ((s_done_flags & mask) == 0U) {
        return 0U;
    }

    s_done_flags &= (uint8_t) ~mask;
    return 1U;
}

K230LinkState K230Link_GetState(void)
{
    return s_state;
}

uint8_t K230Link_GetReadyTask(void)
{
    return s_ready_task;
}

uint8_t K230Link_IsTask6CenterReady(void)
{
    return s_task6_center_ready;
}

uint8_t K230Link_HasBallPosition(void)
{
    return s_ball_position_valid;
}

int16_t K230Link_GetBallPositionX10Mm(void)
{
    return s_ball_position_x10_mm;
}

int16_t K230Link_GetTask6TargetX10Mm(void)
{
    return s_task6_target_x10_mm;
}

const char *K230Link_GetStatusText(uint32_t now_ms)
{
    if (K230Link_IsOnline(now_ms) == 0U) {
        return "K230:OFFLINE";
    }

    switch (s_state) {
        case K230_LINK_STATE_IDLE:
            return "K230:IDLE";
        case K230_LINK_STATE_PREPARING:
            return "K230:PREPARING";
        case K230_LINK_STATE_CENTERING_TASK6:
            return "K230:CENTERING";
        case K230_LINK_STATE_SETTING_TASK6:
            return "K230:SET TARGET";
        case K230_LINK_STATE_READY:
            switch (s_ready_task) {
                case 3U: return "K230:T3 READY";
                case 4U: return "K230:T4 READY";
                case 5U: return "K230:T5 READY";
                case 6U: return "K230:T6 READY";
                default: return "K230:READY";
            }
        case K230_LINK_STATE_RUNNING:
            return "K230:RUNNING";
        case K230_LINK_STATE_RETURNING:
            return "K230:RETURNING";
        case K230_LINK_STATE_ERROR:
            return "K230:ERROR";
        case K230_LINK_STATE_OFFLINE:
        default:
            return "K230:OFFLINE";
    }
}
