#include "speed_pid_tune.h"

#include "../Hardware/UART3_OPENMV/OPENMV.h"
#include "../Hardware/encoder.h"
#include "../Hardware/motor.h"

#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define SPEED_TUNE_DEFAULT_TARGET       (15L)
#define SPEED_TUNE_MAX_TARGET           (35L)
#define SPEED_TUNE_CONTROL_PERIOD_MS    (10U)
#define SPEED_TUNE_REPORT_PERIOD_MS     (100U)

static int32_t s_kp_x1000 = 2800L;
static int32_t s_ki_x1000 = 1000L;
static int32_t s_kd_x1000 = 0L;
static int32_t s_target;
static float s_left_filtered;
static float s_right_filtered;
static float s_left_integral;
static float s_right_integral;
static float s_left_last_error;
static float s_right_last_error;
static int32_t s_left_pwm;
static int32_t s_right_pwm;
static uint32_t s_control_ms;
static uint32_t s_report_ms;
static uint8_t s_filter_valid;
static uint8_t s_stop_requested;
static char s_rx[64];
static uint8_t s_rx_index;
static char s_status[22];

static int32_t LimitI32(int32_t value, int32_t low, int32_t high)
{
    if (value < low) return low;
    if (value > high) return high;
    return value;
}

static void ResetController(void)
{
    s_left_integral = 0.0f;
    s_right_integral = 0.0f;
    s_left_last_error = 0.0f;
    s_right_last_error = 0.0f;
    s_filter_valid = 0U;
}

static void ProcessCommand(char *command)
{
    long kp;
    long ki;
    long kd;
    long target;
    char response[64];

    if (sscanf(command, "PID,%ld,%ld,%ld", &kp, &ki, &kd) == 3) {
        s_kp_x1000 = LimitI32((int32_t) kp, 0L, 5000L);
        s_ki_x1000 = LimitI32((int32_t) ki, 0L, 1000L);
        s_kd_x1000 = LimitI32((int32_t) kd, 0L, 5000L);
        ResetController();
        OPENMV_WriteString("OK,PID\r\n");
    } else if (sscanf(command, "TARGET,%ld", &target) == 1) {
        s_target = LimitI32((int32_t) target, 0L, SPEED_TUNE_MAX_TARGET);
        ResetController();
        OPENMV_WriteString("OK,TARGET\r\n");
    } else if (strcmp(command, "GET") == 0) {
        (void) snprintf(response, sizeof(response),
            "PARAM,%ld,%ld,%ld,%ld\r\n", (long) s_kp_x1000,
            (long) s_ki_x1000, (long) s_kd_x1000, (long) s_target);
        OPENMV_WriteString(response);
    } else if (strcmp(command, "STOP") == 0) {
        s_stop_requested = 1U;
        OPENMV_WriteString("OK,STOP\r\n");
    } else {
        OPENMV_WriteString("ERR,COMMAND\r\n");
    }
}

static void ServiceRx(void)
{
    uint8_t byte;

    while (OPENMV_ReadByte(&byte)) {
        if ((byte == '\r') || (byte == '\n')) {
            if (s_rx_index != 0U) {
                s_rx[s_rx_index] = '\0';
                ProcessCommand(s_rx);
                s_rx_index = 0U;
            }
        } else if (s_rx_index < (sizeof(s_rx) - 1U)) {
            s_rx[s_rx_index++] = (char) byte;
        } else {
            s_rx_index = 0U;
        }
    }
}

void SpeedPidTune_Init(void)
{
    s_target = 0L;
    s_stop_requested = 0U;
    s_rx_index = 0U;
    ResetController();
}

void SpeedPidTune_Start(uint32_t now_ms)
{
    s_target = 0L;
    s_left_pwm = 0L;
    s_right_pwm = 0L;
    s_control_ms = now_ms;
    s_report_ms = now_ms;
    s_stop_requested = 0U;
    s_rx_index = 0U;
    ResetController();
    Motor_SetDuty(MOTOR_LEFT, 0);
    Motor_SetDuty(MOTOR_RIGHT, 0);
    Motor_On();
    TaskManager_SetStatus("UART3 WAIT / K4 STOP");
    OPENMV_WriteString("READY,SPEED_PID_TUNE,9600\r\n");
}

TaskManagerResult SpeedPidTune_Update(uint32_t now_ms)
{
    EncoderData left = Encoder_GetData(ENCODER_1);
    EncoderData right = Encoder_GetData(ENCODER_2);

    ServiceRx();
    if (s_stop_requested != 0U) return TASK_MANAGER_RESULT_COMPLETE;

    if ((now_ms - s_control_ms) >= SPEED_TUNE_CONTROL_PERIOD_MS) {
        float raw_left = (float) left.counts_per_second * 100.0f /
                         (float) ENCODER_SPEED_100_PERCENT_CPS;
        float raw_right = (float) right.counts_per_second * 100.0f /
                          (float) ENCODER_SPEED_100_PERCENT_CPS;
        float left_error;
        float right_error;
        float left_output;
        float right_output;
        float kp = (float) s_kp_x1000 / 1000.0f;
        float ki = (float) s_ki_x1000 / 1000.0f;
        float kd = (float) s_kd_x1000 / 1000.0f;

        if (raw_left < 0.0f) raw_left = -raw_left;
        if (raw_right < 0.0f) raw_right = -raw_right;
        s_control_ms = now_ms;
        if (s_filter_valid == 0U) {
            s_left_filtered = raw_left;
            s_right_filtered = raw_right;
            s_filter_valid = 1U;
        } else {
            s_left_filtered += 0.20f * (raw_left - s_left_filtered);
            s_right_filtered += 0.20f * (raw_right - s_right_filtered);
        }

        left_error = (float) s_target - s_left_filtered;
        right_error = (float) s_target - s_right_filtered;
        s_left_integral += left_error * 0.01f;
        s_right_integral += right_error * 0.01f;
        if (s_left_integral < 0.0f) s_left_integral = 0.0f;
        if (s_right_integral < 0.0f) s_right_integral = 0.0f;
        if (s_left_integral > 100.0f) s_left_integral = 100.0f;
        if (s_right_integral > 100.0f) s_right_integral = 100.0f;

        left_output = kp * left_error + ki * s_left_integral +
            kd * (left_error - s_left_last_error) / 0.01f;
        right_output = kp * right_error + ki * s_right_integral +
            kd * (right_error - s_right_last_error) / 0.01f;
        s_left_last_error = left_error;
        s_right_last_error = right_error;
        if (left_output < 0.0f) left_output = 0.0f;
        if (right_output < 0.0f) right_output = 0.0f;
        if (left_output > 100.0f) left_output = 100.0f;
        if (right_output > 100.0f) right_output = 100.0f;
        s_left_pwm = (int32_t) (left_output + 0.5f);
        s_right_pwm = (int32_t) (right_output + 0.5f);
        Motor_SetDuty(MOTOR_LEFT, (int8_t) s_left_pwm);
        Motor_SetDuty(MOTOR_RIGHT, (int8_t) s_right_pwm);
    }

    if ((now_ms - s_report_ms) >= SPEED_TUNE_REPORT_PERIOD_MS) {
        char frame[80];
        int32_t left_value = (int32_t) (s_left_filtered + 0.5f);
        int32_t right_value = (int32_t) (s_right_filtered + 0.5f);

        s_report_ms = now_ms;
        (void) snprintf(frame, sizeof(frame), "DATA,%ld,%ld,%ld,%ld,%ld\r\n",
            (long) s_target, (long) left_value, (long) right_value,
            (long) s_left_pwm, (long) s_right_pwm);
        OPENMV_WriteString(frame);
        (void) snprintf(s_status, sizeof(s_status), "T:%ld L:%ld R:%ld",
            (long) s_target, (long) left_value, (long) right_value);
        TaskManager_SetStatus(s_status);
    }

    return TASK_MANAGER_RESULT_RUNNING;
}

void SpeedPidTune_Cancel(void)
{
    s_target = 0L;
    s_stop_requested = 0U;
    Motor_SetDuty(MOTOR_LEFT, 0);
    Motor_SetDuty(MOTOR_RIGHT, 0);
    Motor_Off();
    ResetController();
}
