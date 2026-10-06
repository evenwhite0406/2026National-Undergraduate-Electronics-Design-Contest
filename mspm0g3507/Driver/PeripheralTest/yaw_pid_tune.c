#include "yaw_pid_tune.h"

#include "gyro_bmi088.h"
#include "../Hardware/UART3_OPENMV/OPENMV.h"
#include "../Hardware/motor.h"

#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define YAW_TUNE_REPORT_MS       (100U)
#define YAW_TUNE_CAL_SAMPLES     (300U)

typedef enum {
    YAW_TUNE_INIT = 0,
    YAW_TUNE_CALIBRATING,
    YAW_TUNE_READY,
    YAW_TUNE_ERROR
} YawTuneState;

static YawTuneState s_state;
static int32_t s_kp = 120L;
static int32_t s_ki = 0L;
static int32_t s_kd = 300L;
static int16_t s_target_x10;
static int16_t s_speed;
static uint8_t s_running;
static uint32_t s_report_ms;
static char s_rx[64];
static uint8_t s_rx_index;
static char s_line[22];

static int32_t Limit32(int32_t value, int32_t low, int32_t high)
{
    if (value < low) return low;
    if (value > high) return high;
    return value;
}

static void SendParams(void)
{
    char frame[96];
    (void) snprintf(frame, sizeof(frame),
        "PARAM,%ld,%ld,%ld,%d,%d,%u\r\n",
        (long) s_kp, (long) s_ki, (long) s_kd,
        (int) s_target_x10, (int) s_speed, (unsigned int) s_running);
    OPENMV_WriteString(frame);
}

static void ApplyHeading(void)
{
    Motor_SetYawPID(s_kp, s_ki, s_kd);
    if (s_running != 0U) {
        Motor_DriveHeading(s_speed, s_target_x10);
    }
}

static void ProcessCommand(char *command)
{
    long a;
    long b;
    long c;

    if (sscanf(command, "PID,%ld,%ld,%ld", &a, &b, &c) == 3) {
        s_kp = Limit32((int32_t) a, 0L, 5000L);
        s_ki = Limit32((int32_t) b, 0L, 1000L);
        s_kd = Limit32((int32_t) c, 0L, 5000L);
        ApplyHeading();
        OPENMV_WriteString("OK,PID\r\n");
    } else if (sscanf(command, "TARGET,%ld", &a) == 1) {
        s_target_x10 = (int16_t) Limit32((int32_t) a, -1800L, 1800L);
        if (s_running != 0U) Motor_DriveHeading(s_speed, s_target_x10);
        OPENMV_WriteString("OK,TARGET\r\n");
    } else if (sscanf(command, "SPEED,%ld", &a) == 1) {
        s_speed = (int16_t) Limit32((int32_t) a, -35L, 35L);
        if (s_running != 0U) Motor_DriveHeading(s_speed, s_target_x10);
        OPENMV_WriteString("OK,SPEED\r\n");
    } else if (strcmp(command, "RUN") == 0) {
        if (s_state == YAW_TUNE_READY) {
            s_running = 1U;
            ApplyHeading();
            OPENMV_WriteString("OK,RUN\r\n");
        } else {
            OPENMV_WriteString("ERR,NOT_READY\r\n");
        }
    } else if (strcmp(command, "STOP") == 0) {
        s_running = 0U;
        Motor_StopHeading();
        OPENMV_WriteString("OK,STOP\r\n");
    } else if (strcmp(command, "ZERO") == 0) {
        s_running = 0U;
        Motor_StopHeading();
        BMI088_Gyro_ResetAngles();
        s_target_x10 = 0;
        OPENMV_WriteString("OK,ZERO\r\n");
    } else if (strcmp(command, "GET") == 0) {
        SendParams();
    } else {
        OPENMV_WriteString("ERR,COMMAND\r\n");
    }
}

static void ServiceUart3Rx(void)
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

void YawPidTune_Init(void)
{
    s_state = YAW_TUNE_INIT;
    s_running = 0U;
    s_rx_index = 0U;
}

void YawPidTune_Start(uint32_t now_ms)
{
    int result;

    Motor_StopHeading();
    /* Isolate the yaw loop: a changing near-zero wheel-speed target must not
     * accumulate speed-loop integral while tuning in-place rotation. */
    Motor_SetSpeedPID(MOTOR_SPEED_PID_KP_X1000, 0L, 0L);
    s_running = 0U;
    s_speed = 0;
    s_target_x10 = 0;
    s_report_ms = now_ms;
    s_rx_index = 0U;
    result = BMI088_Gyro_Init();
    if (result == BMI088_GYRO_OK) {
        result = BMI088_Gyro_CalibrationStart(YAW_TUNE_CAL_SAMPLES, now_ms);
    }
    if ((result == BMI088_GYRO_OK) ||
        (result == BMI088_GYRO_CALIBRATING)) {
        s_state = YAW_TUNE_CALIBRATING;
        TaskManager_SetStatus("KEEP STILL: CALIBRATE");
        OPENMV_WriteString("CALIBRATING,YAW\r\n");
    } else {
        s_state = YAW_TUNE_ERROR;
        TaskManager_SetStatus("BMI088 INIT ERROR");
        OPENMV_WriteString("ERR,BMI088\r\n");
    }
}

TaskManagerResult YawPidTune_Update(uint32_t now_ms)
{
    const BMI088_GyroData *imu;

    ServiceUart3Rx();
    if (s_state == YAW_TUNE_CALIBRATING) {
        int result = BMI088_Gyro_CalibrationService(now_ms);
        if (result == BMI088_GYRO_OK) {
            BMI088_Gyro_ResetAngles();
            Motor_SetYawPID(s_kp, s_ki, s_kd);
            s_state = YAW_TUNE_READY;
            TaskManager_SetStatus("BT READY / SPEED=0");
            OPENMV_WriteString("READY,YAW_PID_TUNE,9600\r\n");
            SendParams();
        } else if (result < 0) {
            s_state = YAW_TUNE_ERROR;
            TaskManager_SetStatus("BMI088 CAL ERROR");
            OPENMV_WriteString("ERR,CALIBRATION\r\n");
        }
        return TASK_MANAGER_RESULT_RUNNING;
    }

    if (s_state != YAW_TUNE_READY) return TASK_MANAGER_RESULT_RUNNING;
    (void) BMI088_Gyro_Service(now_ms);
    imu = BMI088_Gyro_GetData();
    if ((now_ms - s_report_ms) >= YAW_TUNE_REPORT_MS) {
        char frame[112];
        int16_t yaw = (imu != 0) ? imu->yaw_x10 : 0;
        int16_t rate = (imu != 0) ? imu->yaw_rate_x10 : 0;

        s_report_ms = now_ms;
        (void) snprintf(frame, sizeof(frame),
            "DATA,%d,%d,%d,%d,%d,%d,%d,%d,%d\r\n",
            (int) s_target_x10, (int) yaw, (int) rate,
            (int) Motor_GetTargetSpeed(MOTOR_LEFT),
            (int) Motor_GetTargetSpeed(MOTOR_RIGHT),
            (int) Motor_GetFeedbackSpeed(MOTOR_LEFT),
            (int) Motor_GetFeedbackSpeed(MOTOR_RIGHT),
            (int) Motor_GetOutputDuty(MOTOR_LEFT),
            (int) Motor_GetOutputDuty(MOTOR_RIGHT));
        OPENMV_WriteString(frame);
        (void) snprintf(s_line, sizeof(s_line), "T:%d Y:%d R:%d",
            (int) s_target_x10, (int) yaw, (int) rate);
        TaskManager_SetStatus(s_line);
    }
    return TASK_MANAGER_RESULT_RUNNING;
}

void YawPidTune_Cancel(void)
{
    s_running = 0U;
    Motor_StopHeading();
    Motor_Off();
    Motor_SetSpeedPID(MOTOR_SPEED_PID_KP_X1000,
                      MOTOR_SPEED_PID_KI_X1000,
                      MOTOR_SPEED_PID_KD_X1000);
}
