/**
 * @file position_test.c
 * @brief 位置环 + yaw环 + 左右速度环联合测试。
 */

#include "position_test.h"
#include "gyro_bmi088.h"
#include "../Hardware/motor.h"
#include "../Hardware/encoder.h"
#include "../Hardware/led.h"
#include "../Hardware/buzzer.h"
#include "button.h"
#include <stdio.h>
/* 第一次实车测试参数。 */
#define POSITION_TEST_DISTANCE_CM     (50)
#define POSITION_TEST_SPEED           (15)
#define POSITION_TEST_TARGET_YAW_X10  (0)
#define POSITION_TEST_CAL_SAMPLES     (300U)

typedef enum {
    POSITION_TEST_IDLE = 0,
    POSITION_TEST_CALIBRATING,
    POSITION_TEST_PRIMING,
    POSITION_TEST_READY,
    POSITION_TEST_RUNNING,
    POSITION_TEST_DONE,
    POSITION_TEST_ERROR
} PositionTestState;

static PositionTestState s_state;

static int s_error;

static int32_t s_left_start;
static int32_t s_right_start;

static int16_t s_yaw_x10;
static uint32_t s_yaw_update_ms;
static uint32_t s_last_display_ms;
static uint32_t s_cal_start_ms;
static uint32_t s_cal_service_calls;

static void PositionTest_ShowDisplay(uint32_t now_ms)
{
    char line[32];
    int32_t left_delta;
    int32_t right_delta;
    int32_t average_delta;
    int32_t distance_mm;
    int16_t yaw_abs;

    /* 每100ms刷新一次，避免OLED刷新过快。 */
    if ((uint32_t) (now_ms - s_last_display_ms) < 100U) {
        return;
    }

    s_last_display_ms = now_ms;

    left_delta = Encoder_GetCount(ENCODER_1) - s_left_start;
    right_delta = Encoder_GetCount(ENCODER_2) - s_right_start;
    average_delta = (left_delta + right_delta) / 2L;

    distance_mm = Motor_CountsToDistanceMm(average_delta);

    snprintf(line, sizeof(line),
             "TARGET:500mm V15");
    TaskManager_ShowLine(2U, line);

    snprintf(line, sizeof(line),
             "L:%ld R:%ld",
             (long) left_delta,
             (long) right_delta);
    TaskManager_ShowLine(3U, line);

    snprintf(line, sizeof(line),
             "DIST:%ld mm",
             (long) distance_mm);
    TaskManager_ShowLine(4U, line);

    yaw_abs = (s_yaw_x10 < 0) ?
              (int16_t) (-s_yaw_x10) :
              s_yaw_x10;

    snprintf(line, sizeof(line),
             "YAW:%s%d.%d deg",
             (s_yaw_x10 < 0) ? "-" : "",
             yaw_abs / 10,
             yaw_abs % 10);
    TaskManager_ShowLine(5U, line);

    switch (s_state) {
        case POSITION_TEST_CALIBRATING:
                snprintf(line, sizeof(line),
                "CAL:%lu s",
                (unsigned long)
                ((now_ms - s_cal_start_ms) / 1000U));
        TaskManager_ShowLine(6U, line);

        snprintf(line, sizeof(line),
                "CALL:%lu",
                (unsigned long) s_cal_service_calls);
        TaskManager_ShowLine(7U, line);
            break;

        case POSITION_TEST_PRIMING:
            TaskManager_ShowLine(6U, "WAIT YAW DATA");
            TaskManager_ShowLine(7U, "KEEP CAR STILL");
            break;

        case POSITION_TEST_READY:
            TaskManager_ShowLine(6U, "READY");
            TaskManager_ShowLine(7U, "K3 START");
            break;

        case POSITION_TEST_RUNNING:
            TaskManager_ShowLine(6U, "3 LOOPS RUNNING");
            TaskManager_ShowLine(7U, "K4 EMERGENCY STOP");
            break;

        case POSITION_TEST_DONE:
            TaskManager_ShowLine(6U, "TARGET REACHED");
            TaskManager_ShowLine(7U, "K4 RETURN");
            break;

        case POSITION_TEST_ERROR:
            snprintf(line, sizeof(line),
                     "ERROR:%d", s_error);
            TaskManager_ShowLine(6U, line);
            TaskManager_ShowLine(7U, "K4 RETURN");
            break;

        default:
            TaskManager_ShowLine(6U, "IDLE");
            TaskManager_ShowLine(7U, "K4 RETURN");
            break;
    }
}

void PositionTest_Init(void)
{
    s_state = POSITION_TEST_IDLE;
    s_error = BMI088_GYRO_OK;

    s_left_start = 0L;
    s_right_start = 0L;

    s_yaw_x10 = 0;
    s_yaw_update_ms = 0U;

    s_last_display_ms = 0U;
}

void PositionTest_Start(uint32_t now_ms)
{
    s_last_display_ms = 0U;
    s_cal_start_ms = now_ms;
    s_cal_service_calls = 0U;
    int status;

    /* 进入任务时先确保电机停止。 */
    Motor_Off();

    s_state = POSITION_TEST_IDLE;
    s_error = BMI088_GYRO_OK;

    s_left_start = 0L;
    s_right_start = 0L;
    s_yaw_x10 = 0;
    s_yaw_update_ms = 0U;

    /* 初始化BMI088。 */
    status = BMI088_Gyro_Init();

    if (status != BMI088_GYRO_OK) {
        s_error = status;
        s_state = POSITION_TEST_ERROR;
        LED_SetAll(true, false, false);
        return;
    }

    /*
     * 启动非阻塞零偏校准。
     * 从这里开始，小车必须保持完全静止。
     */
    status = BMI088_Gyro_CalibrationStart(
        POSITION_TEST_CAL_SAMPLES,
        now_ms);

    if (status == BMI088_GYRO_CALIBRATING) {
        s_state = POSITION_TEST_CALIBRATING;

        /* 蓝灯：正在校准。 */
        LED_SetAll(false, false, true);
    } else if (status == BMI088_GYRO_OK) {
        /*
         * 通常不会直接进入这里，但保留兼容处理。
         * 校准完成后先进入PRIMING，等待有效yaw快照。
         */
        BMI088_Gyro_ResetAngles();
        BMI088_Gyro_Service(now_ms);
        s_state = POSITION_TEST_PRIMING;
    } else {
        s_error = status;
        s_state = POSITION_TEST_ERROR;
        LED_SetAll(true, false, false);
    }
}

TaskManagerResult PositionTest_Update(uint32_t now_ms)
{
    uint8_t edges = TaskManager_GetPressedEdges();
    int status;

        /* ===================== BMI088 非阻塞校准 ===================== */
        if (s_state == POSITION_TEST_CALIBRATING) {
            s_cal_service_calls++;

            status = BMI088_Gyro_CalibrationService(now_ms);

            if (status == BMI088_GYRO_OK) {
        /*
        * 校准完成后，把当前位置定义为0°。
        * 先启动姿态服务，READY期间还会持续更新yaw。
        */
        BMI088_Gyro_ResetAngles();
        (void) BMI088_Gyro_Service(now_ms);

        s_state = POSITION_TEST_READY;

        /* 强制下一次循环立即刷新OLED。 */
        s_last_display_ms = now_ms - 100U;

        /* 立即显示，便于确认状态确实已经切换。 */
        TaskManager_ShowLine(6U, "READY");
        TaskManager_ShowLine(7U, "K3 START");

        Buzzer_Beep(BUZZER_SHORT_BEEP_MS, now_ms);
    }
        else if (status != BMI088_GYRO_CALIBRATING) {
            s_error = status;
            s_state = POSITION_TEST_ERROR;

            Motor_Off();
            LED_SetAll(true, false, false);
        }
    }

    /* ===================== 等待第一帧yaw快照 ===================== */
    else if (s_state == POSITION_TEST_PRIMING) {
        status = BMI088_Gyro_Service(now_ms);

        if (status != BMI088_GYRO_OK) {
            s_error = status;
            s_state = POSITION_TEST_ERROR;

            Motor_Off();
            LED_SetAll(true, false, false);
        } else if (BMI088_Gyro_GetYawSnapshot(
                       &s_yaw_x10,
                       &s_yaw_update_ms) != 0U) {
            /*
             * yaw快照已经发布，三环控制现在可以安全启动。
             */
            s_state = POSITION_TEST_READY;

            /* 绿灯：等待K3启动。 */
            LED_SetAll(false, true, false);
        }
    }

    /* ========================= 等待K3 =========================== */
    else if (s_state == POSITION_TEST_READY) {
        status = BMI088_Gyro_Service(now_ms);

        if (status != BMI088_GYRO_OK) {
            s_error = status;
            s_state = POSITION_TEST_ERROR;

            Motor_Off();
            LED_SetAll(true, false, false);
        } else {
            /*
             * READY期间持续发布新yaw快照，避免等待时间过长后
             * 电机一启动就因快照过期而触发保护。
             */
            (void) BMI088_Gyro_GetYawSnapshot(
                &s_yaw_x10,
                &s_yaw_update_ms);

            if ((edges & BUTTON_3_MASK) != 0U) {
                s_left_start = Encoder_GetCount(ENCODER_1);
                s_right_start = Encoder_GetCount(ENCODER_2);

                /*
                 * 三环联合测试：
                 * 位置环 -> 基础速度
                 * yaw环  -> 左右差速
                 * 速度环 -> PWM
                 */
                Motor_DriveDistanceCmHeading(
                    POSITION_TEST_SPEED,
                    POSITION_TEST_DISTANCE_CM,
                    POSITION_TEST_TARGET_YAW_X10);

                s_state = POSITION_TEST_RUNNING;
            }
        }
    }

    /* ========================= 三环运行 ========================== */
    else if (s_state == POSITION_TEST_RUNNING) {
        status = BMI088_Gyro_Service(now_ms);

        if (status != BMI088_GYRO_OK) {
            s_error = status;
            s_state = POSITION_TEST_ERROR;

            Motor_CancelPositionYaw();
            Motor_Off();
            LED_SetAll(true, false, false);
        } else {
            (void) BMI088_Gyro_GetYawSnapshot(
                &s_yaw_x10,
                &s_yaw_update_ms);

            if (Motor_IsPositionYawReached() != 0U) {
                /*
                 * motor.c已经将左右目标速度清零，
                 * 这里进一步关闭驱动，确保测试结束后不再输出。
                 */
                Motor_Off();

                s_state = POSITION_TEST_DONE;
                LED_SetAll(false, false, true);
                Buzzer_Beep(BUZZER_LONG_BEEP_MS, now_ms);
            }
        }
    }
    PositionTest_ShowDisplay(now_ms);
    return TASK_MANAGER_RESULT_RUNNING;
}
 
void PositionTest_Cancel(void)
{
    /*
     * 无论当前处于校准、等待、运行还是完成状态，
     * 退出任务时都立即取消位置/yaw控制并关闭电机。
     */
    Motor_CancelPositionYaw();
    Motor_Off();

    s_state = POSITION_TEST_IDLE;
    s_error = BMI088_GYRO_OK;

    s_left_start = 0L;
    s_right_start = 0L;

    s_yaw_x10 = 0;
    s_yaw_update_ms = 0U;

    LED_SetAll(true, false, false);
}