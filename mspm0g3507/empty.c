/**
 * @file empty.c
 * @brief Eight-event vehicle competition application entry.
 *
 * 任务表见下方 g_tasks[]，共 8 项：图传、绕场停车、摆球、A 到 B、
 * 绕圈保持中心、绕圈到指定点、其他测试、循迹调参。
 */

#include "ti_msp_dl_config.h"
#include "competition_task.h"
#include "Driver/Control/bluetooth_hmi.h"
#include "Driver/Control/k230_link.h"
#include "Driver/Control/line_follow.h"
#include "Driver/Control/task_manager.h"
#include "Driver/Hardware/bluetooth.h"
#include "Driver/Hardware/buzzer.h"
#include "Driver/Hardware/led.h"
#include "Driver/Hardware/motor.h"
#include "Driver/Hardware/oled.h"
#include "Driver/Hardware/photoelectric_sensor.h"
#include "Driver/Hardware/relay.h"
#include "Driver/Hardware/UART3_OPENMV/OPENMV.h"
#include "Driver/vehicle_tuning.h"

#include <stdint.h>

volatile uint32_t g_ms = 0U;

static void App_RegisterLineFollowSliders(void)
{
    Param_Init();

    (void) Param_RegisterSlider("KP", &LINE_KP_X100,
        PARAM_TYPE_INT32, 0L, 400L);
    (void) Param_RegisterSlider("KD", &LINE_KD_X100,
        PARAM_TYPE_INT32, 0L, 500L);
    (void) Param_RegisterSlider("FILT_N", &g_line_filter_alpha_normal,
        PARAM_TYPE_INT32, 1L, 10L);
    (void) Param_RegisterSlider("FILT_F", &g_line_filter_alpha_fast,
        PARAM_TYPE_INT32, 1L, 10L);

    (void) Param_RegisterSlider("BASE_SPD", &LINE_BASE_SPEED,
        PARAM_TYPE_INT16, 6L, 45L);
    (void) Param_RegisterSlider("STR_SPD", &LINE_STRAIGHT_SPEED,
        PARAM_TYPE_INT16, 6L, 45L);
    (void) Param_RegisterSlider("MIN_SPD", &LINE_MIN_SPEED,
        PARAM_TYPE_INT16, 0L, 20L);
    (void) Param_RegisterSlider("MAX_SPD", &LINE_MAX_SPEED,
        PARAM_TYPE_INT16, 20L, 60L);
    (void) Param_RegisterSlider("INT_SPD", &LINE_INTERSECTION_SPEED,
        PARAM_TYPE_INT16, 6L, 35L);

    (void) Param_RegisterSlider("MAX_CORR", &LINE_MAX_CORRECTION,
        PARAM_TYPE_INT16, 0L, 45L);
    (void) Param_RegisterSlider("CURVE_SLOW", &LINE_CURVE_SLOWDOWN_X100,
        PARAM_TYPE_INT32, 0L, 200L);
    (void) Param_RegisterSlider("RAMP_N", &LINE_RAMP_NORMAL,
        PARAM_TYPE_INT16, 1L, 5L);
    (void) Param_RegisterSlider("RAMP_F", &LINE_RAMP_FAST,
        PARAM_TYPE_INT16, 1L, 8L);
    (void) Param_RegisterSlider("BASE_STEP", &LINE_BASE_RAMP_STEP,
        PARAM_TYPE_INT16, 1L, 5L);
    (void) Param_RegisterSlider("BASE_MS",
        &LINE_BASE_RAMP_INTERVAL_MS, PARAM_TYPE_UINT32, 5L, 30L);
    (void) Param_RegisterSlider("CORR_STEP",
        &LINE_CORRECTION_RAMP_STEP, PARAM_TYPE_INT16, 1L, 8L);
    (void) Param_RegisterSlider("REV_STEP",
        &LINE_CORRECTION_REVERSE_RAMP_STEP, PARAM_TYPE_INT16, 1L, 4L);
    (void) Param_RegisterSlider("D_LIM", &LINE_DERIVATIVE_LIMIT_X10,
        PARAM_TYPE_INT16, 10L, 200L);
    (void) Param_RegisterSlider("REC_GAP", &LINE_ERROR_RECOVERY_GAP_X10,
        PARAM_TYPE_INT16, 40L, 200L);

    (void) Param_RegisterSlider("T4_BRK_CM",
        &TASK4_BRAKE_START_CM, PARAM_TYPE_UINT32, 80L, 150L);
    (void) Param_RegisterSlider("T4_CRAWL_CM",
        &TASK4_CRAWL_START_CM, PARAM_TYPE_UINT32, 100L, 154L);
    (void) Param_RegisterSlider("T4_SPD",
        &TASK4_CRUISE_SPEED, PARAM_TYPE_INT16, 20L, 36L);
    (void) Param_RegisterSlider("T4_RAMP_MS",
        &TASK4_BASE_RAMP_INTERVAL_MS, PARAM_TYPE_UINT32, 10L, 40L);
    (void) Param_RegisterSlider("T4_DIST_CM",
        &TASK4_PASS_B_DISTANCE_CM, PARAM_TYPE_UINT32, 130L, 180L);
    (void) Param_RegisterSlider("T2_BRK_CM",
        &TASK2_BRAKE_START_CM, PARAM_TYPE_UINT32, 540L, 610L);
    (void) Param_RegisterSlider("T2_SPD",
        &TASK2_CRUISE_SPEED, PARAM_TYPE_INT16, 25L, 45L);
    (void) Param_RegisterSlider("T2_POS_KP",
        &TASK2_POSITION_KP_X100, PARAM_TYPE_INT32, 50L, 250L);
    (void) Param_RegisterSlider("T2_YAW",
        &TASK2_STOP_YAW_DEG, PARAM_TYPE_UINT32, 280L, 380L);
    (void) Param_RegisterSlider("T56_YAW",
        &TASK56_STOP_YAW_DEG, PARAM_TYPE_UINT32, 320L, 420L);
    (void) Param_RegisterSlider("T56_SPD",
        &TASK56_CRUISE_SPEED, PARAM_TYPE_INT16, 15L, 36L);
    (void) Param_RegisterSlider("T56_PASS",
        &TASK56_PASS_A_DISTANCE_CM, PARAM_TYPE_UINT32, 0L, 20L);
    (void) Param_RegisterSlider("APPR_SPD", &APPROACH_SPEED,
        PARAM_TYPE_INT16, 8L, 28L);
    (void) Param_RegisterSlider("CRAWL_SPD", &CRAWL_SPEED,
        PARAM_TYPE_INT16, 6L, 18L);

    (void) Param_RegisterSlider("S_MS", &START_S_CURVE_MS,
        PARAM_TYPE_UINT32, 200L, 2000L);
    (void) Param_RegisterSlider("BALL_S_MS", &START_BALL_S_CURVE_MS,
        PARAM_TYPE_UINT32, 500L, 2000L);
    (void) Param_RegisterSlider("FF_LEAD", &START_FF_LEAD_MS,
        PARAM_TYPE_UINT32, 0L, 200L);
    (void) Param_RegisterSlider("FF_KA", &START_FF_GAIN_X100,
        PARAM_TYPE_INT32, 0L, 20L);
    (void) Param_RegisterSlider("FF_KD", &START_FF_DAMP_X100,
        PARAM_TYPE_INT32, 0L, 20L);
    (void) Param_RegisterSlider("FF_MAX", &START_FF_MAX_X10,
        PARAM_TYPE_INT16, 0L, 100L);
    (void) Param_RegisterSlider("FF_SGN", &START_FF_SIGN,
        PARAM_TYPE_INT16, -1L, 1L);
    (void) Param_RegisterSlider("FF_EN", &START_FF_LINK_ENABLE,
        PARAM_TYPE_UINT8, 0L, 1L);
    (void) Param_RegisterSlider("IMU_AX", &START_IMU_FORWARD_AXIS,
        PARAM_TYPE_UINT8, 0L, 1L);
    (void) Param_RegisterSlider("IMU_INV", &START_IMU_FORWARD_INVERT,
        PARAM_TYPE_UINT8, 0L, 1L);
    (void) Param_RegisterSlider("FF_LOG", &START_FF_LOG_MS,
        PARAM_TYPE_UINT32, 0L, 3000L);

    (void) Param_RegisterSlider("LOST_FWD",
        &LINE_LOST_FORWARD_SPEED, PARAM_TYPE_INT16, 0L, 20L);
    (void) Param_RegisterSlider("LOST_TURN", &LINE_LOST_TURN_SPEED,
        PARAM_TYPE_INT16, 4L, 30L);
    (void) Param_RegisterSlider("LOST_COAST", &LINE_LOST_COAST_MS,
        PARAM_TYPE_UINT32, 0L, 500L);
    (void) Param_RegisterSlider("LOST_STOP", &LINE_LOST_STOP_MS,
        PARAM_TYPE_UINT32, 500L, 5000L);
}

static const TaskManagerTask g_tasks[] = {
    {"1 K230 VIDEO", CompetitionTask1_Start,
     CompetitionTask_Update, CompetitionTask_Cancel},
    {"2 LAP STOP A", CompetitionTask2_Start,
     CompetitionTask_Update, CompetitionTask_Cancel},
    {"3 BALL +5 TO -5", CompetitionTask3_Start,
     CompetitionTask_Update, CompetitionTask_Cancel},
    {"4 A TO B", CompetitionTask4_Start,
     CompetitionTask_Update, CompetitionTask_Cancel},
    {"5 LAP CENTER", CompetitionTask5_Start,
     CompetitionTask_Update, CompetitionTask_Cancel},
    {"6 LAP TARGET", CompetitionTask6_Start,
     CompetitionTask_Update, CompetitionTask_Cancel},
    {"7 OTHER TEST", CompetitionTask7_Start,
     CompetitionTask_Update, CompetitionTask_Cancel},
    {"8 LINE TUNE", CompetitionTask8_Start,
     CompetitionTask_Update, CompetitionTask_Cancel},
};

int main(void)
{
    __disable_irq();
    SYSCFG_DL_init();  

    LED_Init();
    Buzzer_Init();
    Relay_Init();
    PhotoelectricSensor_Init();
    OLED_Init();
    Bluetooth_Init();
    App_RegisterLineFollowSliders();
    OPENMV_Init();
    K230Link_Init(0U);
    Motor_Init();
    LineFollow_Init();
    CompetitionTask_Init();

    Motor_Off();
    LED_SetAll(true, false, false);

    if (SysTick_Config(CPUCLK_FREQ / 1000U) != 0U) {
        while (1) {
            LED_Toggle(LED_RED);
            delay_cycles(CPUCLK_FREQ / 4U);
        }
    }

    TaskManager_Init(g_tasks,
        (uint8_t) (sizeof(g_tasks) / sizeof(g_tasks[0])));
    __enable_irq();

    while (1) {
        uint32_t now_ms = g_ms;
        static uint32_t last_k230_menu_ms;

        OPENMV_Service();
        K230Link_Update(
            now_ms,
            TaskManager_IsIdle() ? 1U : 0U,
            (uint8_t) (TaskManager_GetSelectedIndex() + 1U));
        TaskManager_Update(now_ms);
        if (TaskManager_IsIdle() &&
            ((uint32_t) (now_ms - last_k230_menu_ms) >= 100U)) {
            last_k230_menu_ms = now_ms;
            TaskManager_ShowLine(4U, K230Link_GetStatusText(now_ms));
        }
        Param_Process();
        Buzzer_Service(now_ms);
        OLED_Service();
        Bluetooth_Service();
        __WFI();
    }
}
