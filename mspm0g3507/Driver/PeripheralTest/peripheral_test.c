/**
 * @file peripheral_test.c
 * @brief 顶层外设联调流程和 OLED 反馈页面。
 *
 * OLED 页面布局：
 *   1：标题
 *   2：四路按键状态和已配置按键掩码
 *   3：八路循迹 GPIO 原始电平
 *   4：经过极性修正的循迹检测结果
 *   5：BMI088 陀螺仪/加速度计 ID 或错误码
 *   6：融合后的横滚角和俯仰角
 *   7：积分偏航角和 Z 轴角速度
 *   8：编码器 1/2 有符号位置计数（测试时显示电机状态）
 */

#include "peripheral_test.h"
#include "button.h"
#include "gyro_bmi088.h"
#include "line_sensor.h"
#include "motor_test.h"
#include "motor_encoder_check.h"
#include "../Control/balance_control.h"
#include "../Control/bluetooth_hmi.h"
#include "../Control/k230_line_follow.h"
#include "../Control/line_follow.h"
#include "../Control/motion_control.h"
#include "../Control/openmv_line_follow.h"
#include "../Control/task_manager.h"
#include "../Hardware/encoder.h"
#include "../Hardware/bluetooth.h"
#include "../Hardware/mmc5983ma.h"
#include "../Hardware/motor.h"
#include "../Hardware/UART3_OPENMV/OPENMV.h"
#include "../Hardware/oled.h"
#include "../Hardware/UART0_openmv/openmv.h"
#include "ti_msp_dl_config.h"
#include <stdio.h>
#include <string.h>

static int g_gyro_init_status = BMI088_GYRO_ERR_I2C;
static int g_mag_init_status = MMC5983MA_ERR_I2C;
static volatile uint32_t g_millis;
static uint32_t g_last_bluetooth_ms;
static uint8_t g_speed_tune_active;

/** 1 ms 时基，用于向姿态滤波器传递真实采样间隔。 */
void PeripheralTest_1msTick(void)
{
    g_millis++;
}

/* 使用小型定宽格式化函数，避免 sprintf() 增大单片机代码体积。 */
static void LineClear(char *line)
{
    uint8_t i;

    for (i = 0U; i < OLED_TEXT_COLS; i++) {
        line[i] = ' ';
    }
    line[OLED_TEXT_COLS] = '\0';
}

static void LineAppendChar(char *line, uint8_t *idx, char ch)
{
    if (*idx < OLED_TEXT_COLS) {
        line[*idx] = ch;
        *idx = (uint8_t) (*idx + 1U);
    }
}

static void LineAppendStr(char *line, uint8_t *idx, const char *text)
{
    while ((text != 0) && (*text != '\0') && (*idx < OLED_TEXT_COLS)) {
        LineAppendChar(line, idx, *text);
        text++;
    }
}

static void LineAppendInt(char *line, uint8_t *idx, int32_t value)
{
    char tmp[11];
    uint8_t count = 0U;
    uint32_t number;

    if (value < 0) {
        LineAppendChar(line, idx, '-');
        number = (uint32_t) (-value);
    } else {
        number = (uint32_t) value;
    }

    do {
        tmp[count] = (char) ('0' + (number % 10U));
        count++;
        number /= 10U;
    } while ((number != 0U) && (count < sizeof(tmp)));

    while (count != 0U) {
        count--;
        LineAppendChar(line, idx, tmp[count]);
    }
}

static void LineAppendFixed1(char *line, uint8_t *idx, int32_t value_x10)
{
    int32_t value = value_x10;
    uint32_t number;

    if (value < 0) {
        LineAppendChar(line, idx, '-');
        number = (uint32_t) (-value);
    } else {
        LineAppendChar(line, idx, '+');
        number = (uint32_t) value;
    }

    LineAppendInt(line, idx, (int32_t) (number / 10U));
    LineAppendChar(line, idx, '.');
    LineAppendChar(line, idx, (char) ('0' + (number % 10U)));
}

static char HexNibble(uint8_t value)
{
    value &= 0x0FU;
    if (value < 10U) {
        return (char) ('0' + value);
    }

    return (char) ('A' + (value - 10U));
}

static void LineAppendHexByte(char *line, uint8_t *idx, uint8_t value)
{
    LineAppendChar(line, idx, HexNibble((uint8_t) (value >> 4)));
    LineAppendChar(line, idx, HexNibble(value));
}

static void FormatKeyBits(uint8_t pressed, uint8_t configured, char *out)
{
    uint8_t i;

    if (out == 0) {
        return;
    }

    for (i = 0U; i < BUTTON_COUNT; i++) {
        uint8_t bit = (uint8_t) (1U << i);

        if ((configured & bit) == 0U) {
            out[i] = '-';
        } else {
            out[i] = ((pressed & bit) != 0U) ? '1' : '0';
        }
    }
    out[BUTTON_COUNT] = '\0';
}

static void ShowLine(uint8_t line, const char *text)
{
    char buf[OLED_TEXT_COLS + 1U];
    uint8_t i = 0U;

    LineClear(buf);
    LineAppendStr(buf, &i, text);
    OLED_ShowString(line, 1U, buf, 1U);
}

static void ShowPage(const BMI088_GyroData *gyro, uint8_t key_pressed,
                     uint8_t key_configured, uint8_t line_raw,
                     uint8_t line_active)
{
    EncoderData encoder1 = Encoder_GetData(ENCODER_1);
    EncoderData encoder2 = Encoder_GetData(ENCODER_2);
    const MMC5983MA_Data *mag = MMC5983MA_GetData();
    char line[OLED_TEXT_COLS + 1U];
    char bits[LINE_SENSOR_COUNT + 1U];
    char key_bits[BUTTON_COUNT + 1U];
    uint8_t i;

    ShowLine(1U, "PERIPHERAL TEST");

    LineClear(line);
    i = 0U;
    FormatKeyBits(key_pressed, key_configured, key_bits);
    LineAppendStr(line, &i, "KEY:");
    LineAppendStr(line, &i, key_bits);
    LineAppendStr(line, &i, " CFG:");
    FormatKeyBits(key_configured, key_configured, key_bits);
    LineAppendStr(line, &i, key_bits);
    OLED_ShowString(2U, 1U, line, 1U);

    LineSensor_FormatBits(line_raw, bits);
    LineClear(line);
    i = 0U;
    LineAppendStr(line, &i, "RAW:");
    LineAppendStr(line, &i, bits);
    OLED_ShowString(3U, 1U, line, 1U);

    LineSensor_FormatBits(line_active, bits);
    LineClear(line);
    i = 0U;
    LineAppendStr(line, &i, "TRK:");
    LineAppendStr(line, &i, bits);
    OLED_ShowString(4U, 1U, line, 1U);

    LineClear(line);
    i = 0U;
    if ((gyro != 0) && (gyro->status == BMI088_GYRO_OK)) {
        LineAppendStr(line, &i, "G:");
        LineAppendHexByte(line, &i, gyro->chip_id);
        LineAppendStr(line, &i, " A:");
        LineAppendHexByte(line, &i, gyro->accel_chip_id);
        LineAppendStr(line, &i, " M:");
        if ((mag != 0) && (mag->status == MMC5983MA_OK)) {
            LineAppendHexByte(line, &i, mag->product_id);
            LineAppendChar(line, &i, mag->calibrated ? 'C' : '-');
        } else {
            LineAppendStr(line, &i, "--");
        }
    } else {
        LineAppendStr(line, &i, "ERR:");
        if (gyro != 0) {
            LineAppendInt(line, &i, gyro->status);
            LineAppendStr(line, &i, " G:");
            LineAppendHexByte(line, &i, gyro->chip_id);
            LineAppendStr(line, &i, " A:");
            LineAppendHexByte(line, &i, gyro->accel_chip_id);
        } else {
            LineAppendInt(line, &i, g_gyro_init_status);
        }
    }
    OLED_ShowString(5U, 1U, line, 1U);

    LineClear(line);
    i = 0U;
    LineAppendStr(line, &i, "R:");
    if (gyro != 0) {
        LineAppendFixed1(line, &i, gyro->roll_x10);
    }
    LineAppendStr(line, &i, " P:");
    if (gyro != 0) {
        LineAppendFixed1(line, &i, gyro->pitch_x10);
    }
    OLED_ShowString(6U, 1U, line, 1U);

    LineClear(line);
    i = 0U;
    LineAppendStr(line, &i, "Y:");
    if (gyro != 0) {
        LineAppendFixed1(line, &i, gyro->yaw_x10);
    }
    LineAppendStr(line, &i, " WZ:");
    if (gyro != 0) {
        LineAppendFixed1(line, &i, gyro->dps_z10);
    }
    OLED_ShowString(7U, 1U, line, 1U);

    LineClear(line);
    i = 0U;
    LineAppendStr(line, &i, "E1:");
    LineAppendInt(line, &i, encoder1.count);
    LineAppendStr(line, &i, " E2:");
    LineAppendInt(line, &i, encoder2.count);
    OLED_ShowString(8U, 1U, line, 1U);
}

/* 每2 ms更新一次IMU；轮询I2C留在主循环，不放入500 Hz定时中断。 */
static void ServiceImu(BMI088_GyroData *gyro, uint32_t *last_update_ms)
{
    uint32_t now = g_millis;
    uint32_t elapsed = now - *last_update_ms;
    int result;

    if ((g_gyro_init_status != BMI088_GYRO_OK) ||
        (elapsed < IMU_UPDATE_MS)) {
        return;
    }

    if (elapsed > IMU_MAX_UPDATE_MS) {
        elapsed = IMU_MAX_UPDATE_MS;
    }

    /* 防止500 Hz控制中断在g_gyro字段更新到一半时读取不一致快照。 */
    NVIC_DisableIRQ(TIMER_0_INST_INT_IRQN);
    result = BMI088_Gyro_Update(gyro, (uint16_t) elapsed);
    BalanceControl_NotifyImuSample(result == BMI088_GYRO_OK);
    NVIC_EnableIRQ(TIMER_0_INST_INT_IRQN);

    if (result != BMI088_GYRO_OK) {
        g_gyro_init_status = gyro->status;
    }
    *last_update_ms = now;

    if ((g_speed_tune_active == 0U) &&
        (g_gyro_init_status == BMI088_GYRO_OK) &&
        ((now - g_last_bluetooth_ms) >= BLUETOOTH_WAVEFORM_MS)) {
        g_last_bluetooth_ms = now;
        Param_SendAngles(gyro->roll_x10, gyro->pitch_x10, gyro->yaw_x10);
    }
}

/* 在原本会阻塞的电机等待期间继续进行姿态采样。 */
/* ======================== 任务实现 ======================== */

static const MotorTestCommand g_motor_sequence[] = {
    MOTOR_TEST_LEFT_FORWARD,
    MOTOR_TEST_LEFT_BACKWARD,
    MOTOR_TEST_RIGHT_FORWARD,
    MOTOR_TEST_RIGHT_BACKWARD,
    MOTOR_TEST_BOTH_FORWARD,
    MOTOR_TEST_BOTH_BACKWARD
};

static uint8_t g_motor_task_step;
static uint8_t g_motor_task_pausing;
static char g_balance_status[OLED_TEXT_COLS + 1U];
static uint32_t g_balance_status_ms;
static uint32_t g_balance_finish_ms;
static uint32_t g_balance_uart_ms;
static char g_balance_uart_rx[64];
static uint8_t g_balance_uart_rx_index;
static uint8_t g_balance_stop_requested;
static uint8_t g_balance_armed;
static uint8_t g_balance_uart_turn_frame;
static char g_angle_dir_status[OLED_TEXT_COLS + 1U];
static uint32_t g_angle_dir_status_ms;
static char g_angle_tune_status[OLED_TEXT_COLS + 1U];
static uint32_t g_angle_tune_status_ms;
static uint32_t g_angle_tune_uart_ms;
static uint32_t g_angle_tune_saturation_ms;
static char g_angle_tune_uart_rx[64];
static uint8_t g_angle_tune_uart_rx_index;
static uint8_t g_angle_tune_stop_requested;
static uint8_t g_angle_tune_armed;
static char g_speed_tune_status[OLED_TEXT_COLS + 1U];
static uint32_t g_speed_tune_status_ms;
static uint32_t g_speed_tune_wave_ms;
/* UART3自动调参实测采用值：Kp=2.8，Ki=1.0，Kd=0.02。 */
static int32_t g_speed_tune_kp_x1000 = 2800L;
static int32_t g_speed_tune_ki_x1000 = 1000L;
static int32_t g_speed_tune_kd_x1000 = 0L;
static int32_t g_speed_tune_target = SPEED_TUNE_TARGET_PERCENT;
static uint32_t g_speed_tune_control_ms;
static float g_speed_tune_left_filtered;
static float g_speed_tune_right_filtered;
static float g_speed_tune_left_integral;
static float g_speed_tune_right_integral;
static float g_speed_tune_left_last_error;
static float g_speed_tune_right_last_error;
static uint8_t g_speed_tune_filter_valid;
static int32_t g_speed_tune_left_pwm;
static int32_t g_speed_tune_right_pwm;
static char g_speed_tune_uart_rx[64];
static uint8_t g_speed_tune_uart_rx_index;
static uint8_t g_speed_tune_stop_requested;
static MotorEncoderCheckState g_encoder_check_last_state;
static uint32_t g_encoder_check_finish_ms;
static uint8_t g_encoder_check_reported;
static char g_encoder_check_status[OLED_TEXT_COLS + 1U];
static char g_encoder_cpr_status[OLED_TEXT_COLS + 1U];
static uint32_t g_encoder_cpr_status_ms;

typedef enum {
    MAX_SPEED_IDLE = 0,
    MAX_SPEED_LEFT_30,
    MAX_SPEED_LEFT_60,
    MAX_SPEED_LEFT_100,
    MAX_SPEED_PAUSE,
    MAX_SPEED_RIGHT_30,
    MAX_SPEED_RIGHT_60,
    MAX_SPEED_RIGHT_100,
    MAX_SPEED_RESULT
} MaxSpeedState;

static MaxSpeedState g_max_speed_state;
static uint32_t g_max_speed_deadline_ms;
static uint32_t g_max_speed_sample_start_ms;
static uint32_t g_max_speed_last_sample_ms;
static uint32_t g_max_speed_status_ms;
static int64_t g_max_speed_sum;
static uint32_t g_max_speed_samples;
static int32_t g_max_speed_peak;
static int32_t g_max_speed_left_average;
static int32_t g_max_speed_right_average;
static int32_t g_max_speed_left_peak;
static int32_t g_max_speed_right_peak;
static char g_max_speed_status[OLED_TEXT_COLS + 1U];
static uint32_t g_task_deadline_ms;
static LineFollowState g_line_follow_last_state;
static VisionLineState g_k230_line_last_state;
static VisionLineState g_openmv_line_last_state;
static uint32_t g_k230_status_ms;
static uint32_t g_openmv_status_ms;
static uint32_t g_k230_finish_ms;
static uint32_t g_openmv_finish_ms;
static char g_k230_status[OLED_TEXT_COLS + 1U];
static char g_openmv_status[OLED_TEXT_COLS + 1U];

static void FormatVisionStatus(char *buffer, const char *name,
                               uint16_t center_x)
{
    uint8_t index = 0U;

    LineClear(buffer);
    LineAppendStr(buffer, &index, name);
    LineAppendStr(buffer, &index, " CX:");
    LineAppendInt(buffer, &index, center_x);
}

static void MotorTask_Start(uint32_t now_ms)
{
    g_motor_task_step = 0U;
    g_motor_task_pausing = 0U;
    MotorTest_Drive(g_motor_sequence[0], MOTOR_TEST_SPEED);
    TaskManager_SetStatus(MotorTest_GetName(g_motor_sequence[0]));
    g_task_deadline_ms = now_ms + MOTOR_STEP_MS;
}

static TaskManagerResult MotorTask_Update(uint32_t now_ms)
{
    if ((int32_t) (now_ms - g_task_deadline_ms) < 0) {
        return TASK_MANAGER_RESULT_RUNNING;
    }

    if (g_motor_task_pausing == 0U) {
        MotorTest_Stop();
        g_motor_task_pausing = 1U;
        g_task_deadline_ms = now_ms + MOTOR_PAUSE_MS;
        TaskManager_SetStatus("PAUSE");
        return TASK_MANAGER_RESULT_RUNNING;
    }

    g_motor_task_step++;
    if (g_motor_task_step >=
        (sizeof(g_motor_sequence) / sizeof(g_motor_sequence[0]))) {
        MotorTest_Stop();
        return TASK_MANAGER_RESULT_COMPLETE;
    }

    g_motor_task_pausing = 0U;
    MotorTest_Drive(g_motor_sequence[g_motor_task_step], MOTOR_TEST_SPEED);
    TaskManager_SetStatus(
        MotorTest_GetName(g_motor_sequence[g_motor_task_step]));
    g_task_deadline_ms = now_ms + MOTOR_STEP_MS;
    return TASK_MANAGER_RESULT_RUNNING;
}

static void MotorTask_Cancel(void)
{
    MotorTest_Stop();
}

static void BalanceTask_Start(uint32_t now_ms)
{
    MotionControl_Stop();
    g_balance_status_ms = now_ms;
    g_balance_finish_ms = 0U;
    g_balance_uart_ms = now_ms;
    g_balance_uart_rx_index = 0U;
    g_balance_stop_requested = 0U;
    g_balance_armed = 0U;
    g_balance_uart_turn_frame = 0U;
    TaskManager_SetStatus("WAIT UART / HOLD");
    OPENMV_WriteString("READY,BALANCE_HOLD_V2,9600\r\n");
}

static void BalanceTask_ProcessCommand(char *command)
{
    long kp;
    long ki;
    long kd;
    long angle_kp;
    long angle_kd;
    long turn_command_kp;
    long turn_speed_kp;
    long turn_yaw_kd;
    long turn_pwm_limit;
    long turn_target;
    long trim;

    if (sscanf(command, "BSPD,%ld,%ld,%ld", &kp, &ki, &kd) == 3) {
        BalanceControl_SetSpeedTunings((int32_t) kp, (int32_t) ki,
                                       (int32_t) kd);
        OPENMV_WriteString("OK,BSPD\r\n");
    } else if (sscanf(command, "BAPD,%ld,%ld", &angle_kp,
                      &angle_kd) == 2) {
        BalanceControl_SetAngleTuningsX1000((int32_t) angle_kp,
                                             (int32_t) angle_kd);
        OPENMV_WriteString("OK,BAPD\r\n");
    } else if (sscanf(command, "BTUNE,%ld,%ld,%ld,%ld",
                      &turn_command_kp, &turn_speed_kp,
                      &turn_yaw_kd, &turn_pwm_limit) == 4) {
        BalanceControl_SetTurnTunings((int32_t) turn_command_kp,
            (int32_t) turn_speed_kp, (int32_t) turn_yaw_kd,
            (int32_t) turn_pwm_limit);
        OPENMV_WriteString("OK,BTUNE\r\n");
    } else if (sscanf(command, "BTURN,%ld", &turn_target) == 1) {
        if (turn_target < -500L) turn_target = -500L;
        if (turn_target > 500L) turn_target = 500L;
        BalanceControl_SetTargetTurn((float) turn_target);
        OPENMV_WriteString("OK,BTURN\r\n");
    } else if (sscanf(command, "TRIM,%ld", &trim) == 1) {
        BalanceControl_SetTrimAngleX10((int32_t) trim);
        OPENMV_WriteString("OK,TRIM\r\n");
    } else if (strcmp(command, "GET") == 0) {
        char response[80];
        int32_t kp_value;
        int32_t ki_value;
        int32_t kd_value;
        int32_t angle_kp;
        int32_t angle_kd;
        int32_t turn_command_kp;
        int32_t turn_speed_kp;
        int32_t turn_yaw_kd;
        int32_t turn_pwm_limit;

        BalanceControl_GetSpeedTunings(&kp_value, &ki_value, &kd_value);
        BalanceControl_GetAngleTuningsX1000(&angle_kp, &angle_kd);
        BalanceControl_GetTurnTunings(&turn_command_kp, &turn_speed_kp,
            &turn_yaw_kd, &turn_pwm_limit);
        (void) snprintf(response, sizeof(response),
            "BPARAM,%ld,%ld,%ld,%ld,%ld,%ld\r\n",
            (long) kp_value, (long) ki_value, (long) kd_value,
            (long) angle_kp, (long) angle_kd,
            (long) BalanceControl_GetTrimAngleX10());
        OPENMV_WriteString(response);
        (void) snprintf(response, sizeof(response),
            "TPARAM,%ld,%ld,%ld,%ld\r\n",
            (long) turn_command_kp, (long) turn_speed_kp,
            (long) turn_yaw_kd, (long) turn_pwm_limit);
        OPENMV_WriteString(response);
    } else if (strcmp(command, "ARM") == 0) {
        BalanceControlState state = BalanceControl_GetState();
        float angle = state.balance_angle;
        float side = state.side_angle;
        if (angle < 0.0f) angle = -angle;
        if (side < 0.0f) side = -side;
        if (!state.imu_valid) {
            OPENMV_WriteString("ERR,IMU_INVALID\r\n");
        } else if ((angle > 5.0f) || (side > 10.0f)) {
            OPENMV_WriteString("ERR,NOT_UPRIGHT\r\n");
        } else {
            MotionControl_Start();
            MotionControl_SetWheelSpeeds(0, 0);
            g_balance_finish_ms = 0U;
            g_balance_armed = 1U;
            OPENMV_WriteString("OK,ARM\r\n");
        }
    } else if (strcmp(command, "DISARM") == 0) {
        BalanceControl_SetTargetTurn(0.0f);
        MotionControl_Stop();
        g_balance_armed = 0U;
        OPENMV_WriteString("OK,DISARM\r\n");
    } else if (strcmp(command, "STOP") == 0) {
        BalanceControl_SetTargetTurn(0.0f);
        MotionControl_Stop();
        g_balance_armed = 0U;
        g_balance_stop_requested = 1U;
        OPENMV_WriteString("OK,STOP\r\n");
    } else {
        OPENMV_WriteString("ERR,COMMAND\r\n");
    }
}

static void BalanceTask_ServiceUart(void)
{
    uint8_t byte;

    while (OPENMV_ReadByte(&byte)) {
        if ((byte == '\r') || (byte == '\n')) {
            if (g_balance_uart_rx_index != 0U) {
                g_balance_uart_rx[g_balance_uart_rx_index] = '\0';
                BalanceTask_ProcessCommand(g_balance_uart_rx);
                g_balance_uart_rx_index = 0U;
            }
        } else if (g_balance_uart_rx_index <
                   (sizeof(g_balance_uart_rx) - 1U)) {
            g_balance_uart_rx[g_balance_uart_rx_index++] = (char) byte;
        } else {
            g_balance_uart_rx_index = 0U;
        }
    }
}

static TaskManagerResult BalanceTask_Update(uint32_t now_ms)
{
    BalanceControlState state;
    uint8_t index;

    BalanceTask_ServiceUart();
    if (g_balance_stop_requested != 0U) {
        return TASK_MANAGER_RESULT_COMPLETE;
    }
    state = BalanceControl_GetState();

 /*
     * 自动ARM：
     * IMU有效，并且前后倾角不超过5度、侧倾不超过10度时，
     * 自动启动电机和平衡控制。
     */
    if (g_balance_armed == 0U) {
        float angle = state.balance_angle;
        float side = state.side_angle;

        if (angle < 0.0f) {
            angle = -angle;
        }

        if (side < 0.0f) {
            side = -side;
        }

        if (state.imu_valid &&
            (angle <= 5.0f) &&
            (side <= 10.0f)) {

            MotionControl_SetWheelSpeeds(0, 0);
            g_balance_finish_ms = 0U;
            g_balance_stop_requested = 0U;
            g_balance_armed = 1U;
            MotionControl_Start();

            TaskManager_SetStatus("AUTO ARMED");
            OPENMV_WriteString("OK,AUTO_ARM\r\n");
        }
    }

    if ((g_balance_armed != 0U) &&
        (!state.imu_valid || state.fallen || state.side_fallen)) {
        if (g_balance_finish_ms == 0U) {
            if (!state.imu_valid) {
                TaskManager_SetStatus("IMU INVALID");
            } else if (state.side_fallen) {
                TaskManager_SetStatus("SIDE FALL STOP");
            } else {
                TaskManager_SetStatus("FALL STOP");
            }
            g_balance_finish_ms = now_ms + 800U;
        }
        return ((int32_t) (now_ms - g_balance_finish_ms) >= 0)
            ? TASK_MANAGER_RESULT_FAILED
            : TASK_MANAGER_RESULT_RUNNING;
    }

    if ((now_ms - g_balance_status_ms) >= OLED_UPDATE_MS) {
        g_balance_status_ms = now_ms;
        LineClear(g_balance_status);
        index = 0U;
        LineAppendStr(g_balance_status, &index, "A:");
        LineAppendFixed1(g_balance_status, &index,
                         (int32_t) (state.balance_angle * 10.0f));
        LineAppendStr(g_balance_status, &index, " T:");
        LineAppendFixed1(g_balance_status, &index,
                         (int32_t) (state.target_balance_angle * 10.0f));
        LineAppendStr(g_balance_status, &index, " P:");
        LineAppendInt(g_balance_status, &index,
                      (int32_t) state.average_pwm);
        TaskManager_SetStatus(g_balance_status);
    }

    if ((now_ms - g_balance_uart_ms) >= 100U) {
        char frame[144];
        g_balance_uart_ms = now_ms;
        if (g_balance_uart_turn_frame == 0U) {
            (void) snprintf(frame, sizeof(frame),
                "BAL,%lu,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%u,%ld,%ld\r\n",
                (unsigned long) now_ms,
                (long) (state.balance_angle * 10.0f),
                (long) (state.target_balance_angle * 10.0f),
                (long) state.average_speed,
                (long) state.left_speed,
                (long) state.right_speed,
                (long) state.average_pwm,
                (long) state.differential_pwm,
                (long) BalanceControl_GetTrimAngleX10(),
                (unsigned int) g_balance_armed,
                (long) state.position,
                (long) state.position_error);
            g_balance_uart_turn_frame = 1U;
        } else {
            (void) snprintf(frame, sizeof(frame),
                "TURN,%lu,%ld,%ld,%ld,%ld,%ld,%ld\r\n",
                (unsigned long) now_ms,
                (long) state.target_turn,
                (long) state.differential_speed,
                (long) (state.yaw_rate * 10.0f),
                (long) state.differential_pwm,
                (long) state.left_pwm,
                (long) state.right_pwm);
            g_balance_uart_turn_frame = 0U;
        }
        OPENMV_WriteString(frame);
    }
    return TASK_MANAGER_RESULT_RUNNING;
}

static void BalanceTask_Cancel(void)
{
    g_balance_armed = 0U;
    BalanceControl_SetTargetTurn(0.0f);
    MotionControl_Stop();
}

static void AngleDirTask_Start(uint32_t now_ms)
{
    MotionControl_Stop();
    Motor_SetDuty(MOTOR_LEFT, 0);
    Motor_SetDuty(MOTOR_RIGHT, 0);
    Motor_On();
    g_angle_dir_status_ms = now_ms - OLED_UPDATE_MS;
    TaskManager_SetStatus("HOLD AND TILT");
}

static TaskManagerResult AngleDirTask_Update(uint32_t now_ms)
{
    BalanceControlState state = BalanceControl_GetState();
    int32_t angle_x10 = (int32_t) (state.balance_angle * 10.0f);
    int32_t magnitude = (angle_x10 < 0L) ? -angle_x10 : angle_x10;
    int32_t pwm = 0L;
    uint8_t index;

    if (!state.imu_valid) {
        Motor_SetDuty(MOTOR_LEFT, 0);
        Motor_SetDuty(MOTOR_RIGHT, 0);
        TaskManager_SetStatus("IMU INVALID");
        return TASK_MANAGER_RESULT_RUNNING;
    }

    if (magnitude >= ANGLE_DIR_STOP_X10) {
        pwm = 0L;
    } else if (magnitude >= ANGLE_DIR_DEADBAND_X10) {
        /* 与正式角度环的error=target-angle符号保持一致。 */
        pwm = (angle_x10 > 0L) ? -ANGLE_DIR_TEST_PWM : ANGLE_DIR_TEST_PWM;
    }
    Motor_SetDuty(MOTOR_LEFT, (int8_t) pwm);
    Motor_SetDuty(MOTOR_RIGHT, (int8_t) pwm);

    if ((now_ms - g_angle_dir_status_ms) >= OLED_UPDATE_MS) {
        g_angle_dir_status_ms = now_ms;
        LineClear(g_angle_dir_status);
        index = 0U;
        LineAppendStr(g_angle_dir_status, &index, "A:");
        LineAppendFixed1(g_angle_dir_status, &index, angle_x10);
        LineAppendStr(g_angle_dir_status, &index, " PWM:");
        LineAppendInt(g_angle_dir_status, &index, pwm);
        TaskManager_SetStatus(g_angle_dir_status);
    }
    return TASK_MANAGER_RESULT_RUNNING;
}

static void AngleDirTask_Cancel(void)
{
    Motor_SetDuty(MOTOR_LEFT, 0);
    Motor_SetDuty(MOTOR_RIGHT, 0);
    Motor_Off();
    MotionControl_Stop();
}

static void AngleTune_ProcessCommand(char *command)
{
    long kp;
    long kd;
    long trim;

    if (sscanf(command, "APD,%ld,%ld", &kp, &kd) == 2) {
        BalanceControl_SetAngleTuningsX1000((int32_t) kp, (int32_t) kd);
        OPENMV_WriteString("OK,APD\r\n");
    } else if (sscanf(command, "TRIM,%ld", &trim) == 1) {
        BalanceControl_SetTrimAngleX10((int32_t) trim);
        OPENMV_WriteString("OK,TRIM\r\n");
    } else if (strcmp(command, "GET") == 0) {
        char response[64];
        int32_t current_kp;
        int32_t current_kd;
        BalanceControl_GetAngleTuningsX1000(&current_kp, &current_kd);
        (void) snprintf(response, sizeof(response),
            "APARAM,%ld,%ld,%ld\r\n", (long) current_kp,
            (long) current_kd,
            (long) BalanceControl_GetTrimAngleX10());
        OPENMV_WriteString(response);
    } else if (strcmp(command, "ARM") == 0) {
        BalanceControlState state = BalanceControl_GetState();
        float angle = state.balance_angle;
        if (angle < 0.0f) angle = -angle;
        if (!state.imu_valid) {
            OPENMV_WriteString("ERR,IMU_INVALID\r\n");
        } else if (angle > 5.0f) {
            OPENMV_WriteString("ERR,NOT_UPRIGHT\r\n");
        } else {
            MotionControl_Start();
            MotionControl_SetWheelSpeeds(0, 0);
            BalanceControl_SetSpeedLoopEnabled(false);
            g_angle_tune_saturation_ms = 0U;
            g_angle_tune_armed = 1U;
            OPENMV_WriteString("OK,ARM\r\n");
        }
    } else if (strcmp(command, "DISARM") == 0) {
        MotionControl_Stop();
        g_angle_tune_armed = 0U;
        OPENMV_WriteString("OK,DISARM\r\n");
    } else if (strcmp(command, "STOP") == 0) {
        g_angle_tune_stop_requested = 1U;
        OPENMV_WriteString("OK,STOP\r\n");
    } else {
        OPENMV_WriteString("ERR,COMMAND\r\n");
    }
}

static void AngleTune_ServiceUart(void)
{
    uint8_t byte;

    /* UART3 owns the angle-tuner text protocol. Bluetooth is exclusively
     * consumed by Param_Process(), otherwise the two parsers split frames. */
    while (OPENMV_ReadByte(&byte)) {
        if ((byte == '\r') || (byte == '\n')) {
            if (g_angle_tune_uart_rx_index != 0U) {
                g_angle_tune_uart_rx[g_angle_tune_uart_rx_index] = '\0';
                AngleTune_ProcessCommand(g_angle_tune_uart_rx);
                g_angle_tune_uart_rx_index = 0U;
            }
        } else if (g_angle_tune_uart_rx_index <
                   (sizeof(g_angle_tune_uart_rx) - 1U)) {
            g_angle_tune_uart_rx[g_angle_tune_uart_rx_index++] = (char) byte;
        } else {
            g_angle_tune_uart_rx_index = 0U;
        }
    }
}

static void AngleTuneTask_Start(uint32_t now_ms)
{
    g_angle_tune_status_ms = now_ms - OLED_UPDATE_MS;
    g_angle_tune_uart_ms = now_ms - BLUETOOTH_WAVEFORM_MS;
    g_angle_tune_saturation_ms = 0U;
    g_angle_tune_uart_rx_index = 0U;
    g_angle_tune_stop_requested = 0U;

    /*
     * 自动启动角度环调试
     * 不需要ARM
     */
    g_angle_tune_armed = 1U;

    /*
     * 关闭速度环，只调角度PD
     */
    BalanceControl_SetSpeedLoopEnabled(false);

    /*
     * 清零速度目标
     */
    MotionControl_SetWheelSpeeds(0, 0);

    /*
     * 启动电机控制
     */
    MotionControl_Start();

    OPENMV_WriteString("READY,ANGLE_PD_AUTO_RUN\r\n");
    TaskManager_SetStatus("ANGLE PD RUNNING");
}

static TaskManagerResult AngleTuneTask_Update(uint32_t now_ms)
{
    BalanceControlState state = BalanceControl_GetState();
    int32_t angle_x10 = (int32_t) (state.balance_angle * 10.0f);
    int32_t target_x10 = (int32_t) (state.target_balance_angle * 10.0f);
    int32_t rate_x10 = (int32_t) (state.balance_rate * 10.0f);
    int32_t pwm = (int32_t) state.average_pwm;
    int32_t abs_angle = (angle_x10 < 0L) ? -angle_x10 : angle_x10;
    int32_t abs_rate = (rate_x10 < 0L) ? -rate_x10 : rate_x10;
    int32_t abs_pwm = (pwm < 0L) ? -pwm : pwm;
    uint8_t index;

    AngleTune_ServiceUart();
    if (g_angle_tune_stop_requested != 0U) {
        return TASK_MANAGER_RESULT_COMPLETE;
    }

    if ((g_angle_tune_armed != 0U) &&
        (abs_pwm >= (BALANCE_AVERAGE_PWM_LIMIT_DEFAULT - 1L))) {
        if (g_angle_tune_saturation_ms == 0U) {
            g_angle_tune_saturation_ms = now_ms;
        }
    } else {
        g_angle_tune_saturation_ms = 0U;
    }

    if ((g_angle_tune_armed != 0U) &&
        (!state.imu_valid || state.fallen || state.side_fallen ||
        (abs_angle >= ANGLE_TUNE_STOP_X10) ||
        (abs_rate >= ANGLE_TUNE_RATE_STOP_X10) ||
        ((g_angle_tune_saturation_ms != 0U) &&
         ((now_ms - g_angle_tune_saturation_ms) >=
          ANGLE_TUNE_SATURATION_MS)))) {
        MotionControl_Stop();
        g_angle_tune_armed = 0U;
        g_angle_tune_saturation_ms = 0U;
        OPENMV_WriteString("FAULT,ANGLE_TUNE_STOP\r\n");
        TaskManager_SetStatus("STOPPED / WAIT UART");
    }

    if ((now_ms - g_angle_tune_uart_ms) >= BLUETOOTH_WAVEFORM_MS) {
        char frame[64];
        g_angle_tune_uart_ms = now_ms;
        (void) snprintf(frame, sizeof(frame),
            "ANGLE,%ld,%ld,%ld,%ld\r\n", (long) target_x10,
            (long) angle_x10, (long) rate_x10, (long) pwm);
        OPENMV_WriteString(frame);
    }

    if ((now_ms - g_angle_tune_status_ms) >= OLED_UPDATE_MS) {
        g_angle_tune_status_ms = now_ms;
        LineClear(g_angle_tune_status);
        index = 0U;
        LineAppendStr(g_angle_tune_status, &index, "A:");
        LineAppendFixed1(g_angle_tune_status, &index, angle_x10);
        LineAppendStr(g_angle_tune_status, &index, " P:");
        LineAppendInt(g_angle_tune_status, &index, pwm);
        TaskManager_SetStatus(g_angle_tune_status);
    }
    return TASK_MANAGER_RESULT_RUNNING;
}

static void AngleTuneTask_Cancel(void)
{
    BalanceControl_SetSpeedLoopEnabled(true);
    MotionControl_Stop();
    g_angle_tune_armed = 0U;
}

static void SpeedTuneTask_Start(uint32_t now_ms)
{
    g_speed_tune_active = 1U;
    g_speed_tune_status_ms = now_ms - OLED_UPDATE_MS;
    g_speed_tune_wave_ms = now_ms - BLUETOOTH_WAVEFORM_MS;
    g_speed_tune_control_ms = now_ms;
    g_speed_tune_target = SPEED_TUNE_TARGET_PERCENT;
    MotionControl_Stop();
    g_speed_tune_left_filtered = 0.0f;
    g_speed_tune_right_filtered = 0.0f;
    g_speed_tune_left_integral = 0.0f;
    g_speed_tune_right_integral = 0.0f;
    g_speed_tune_left_last_error = 0.0f;
    g_speed_tune_right_last_error = 0.0f;
    g_speed_tune_filter_valid = 0U;
    g_speed_tune_left_pwm = 0L;
    g_speed_tune_right_pwm = 0L;
    g_speed_tune_uart_rx_index = 0U;
    g_speed_tune_stop_requested = 0U;
    Motor_SetDuty(MOTOR_LEFT, 0);
    Motor_SetDuty(MOTOR_RIGHT, 0);
    Motor_On();
    TaskManager_SetStatus("TARGET:15 WAIT");
    OPENMV_WriteString("READY,SPEED_PID_TUNE,9600\r\n");
}

static int32_t SpeedTune_LimitInt(int32_t value,
                                  int32_t minimum,
                                  int32_t maximum)
{
    if (value < minimum) return minimum;
    if (value > maximum) return maximum;
    return value;
}

static void SpeedTune_ResetController(void)
{
    g_speed_tune_left_integral = 0.0f;
    g_speed_tune_right_integral = 0.0f;
    g_speed_tune_left_last_error = 0.0f;
    g_speed_tune_right_last_error = 0.0f;
    g_speed_tune_filter_valid = 0U;
}

static void SpeedTune_ProcessUartCommand(char *command)
{
    long kp;
    long ki;
    long kd;
    long target;
    float kp_float;
    float ki_float;
    float kd_float;
    float target_float;

    if (sscanf(command, "PID,%ld,%ld,%ld", &kp, &ki, &kd) == 3) {
        g_speed_tune_kp_x1000 = SpeedTune_LimitInt((int32_t) kp, 0L, 5000L);
        g_speed_tune_ki_x1000 = SpeedTune_LimitInt((int32_t) ki, 0L, 1000L);
        g_speed_tune_kd_x1000 = SpeedTune_LimitInt((int32_t) kd, 0L, 5000L);
        SpeedTune_ResetController();
        OPENMV_WriteString("OK,PID\r\n");
    } else if ((sscanf(command, "PID %f %f %f", &kp_float, &ki_float,
                       &kd_float) == 3) ||
               (sscanf(command, "SET P:%f I:%f D:%f", &kp_float,
                       &ki_float, &kd_float) == 3) ||
               (sscanf(command, "SET KP:%f KI:%f KD:%f", &kp_float,
                       &ki_float, &kd_float) == 3)) {
        /* llm-pid-tuner generic_serial_csv兼容协议，固件内部仍用x1000定点值。 */
        if ((kp_float >= 0.0f) && (kp_float <= 5.0f) &&
            (ki_float >= 0.0f) && (ki_float <= 1.0f) &&
            (kd_float >= 0.0f) && (kd_float <= 5.0f)) {
            g_speed_tune_kp_x1000 = (int32_t) (kp_float * 1000.0f + 0.5f);
            g_speed_tune_ki_x1000 = (int32_t) (ki_float * 1000.0f + 0.5f);
            g_speed_tune_kd_x1000 = (int32_t) (kd_float * 1000.0f + 0.5f);
            SpeedTune_ResetController();
            OPENMV_WriteString("# PID Updated\r\n");
        } else {
            OPENMV_WriteString("# ERROR: PID params rejected\r\n");
        }
    } else if (sscanf(command, "TARGET,%ld", &target) == 1) {
        g_speed_tune_target = SpeedTune_LimitInt((int32_t) target, 0L, 30L);
        SpeedTune_ResetController();
        OPENMV_WriteString("OK,TARGET\r\n");
    } else if (sscanf(command, "SETPOINT:%f", &target_float) == 1) {
        if ((target_float >= 0.0f) && (target_float <= 30.0f)) {
            g_speed_tune_target = (int32_t) (target_float + 0.5f);
            SpeedTune_ResetController();
            OPENMV_WriteString("# Setpoint Updated\r\n");
        } else {
            OPENMV_WriteString("# ERROR: Setpoint rejected (0..30)\r\n");
        }
    } else if (strcmp(command, "RESET") == 0) {
        SpeedTune_ResetController();
        OPENMV_WriteString("OK,RESET\r\n");
    } else if (strcmp(command, "STOP") == 0) {
        g_speed_tune_stop_requested = 1U;
        OPENMV_WriteString("OK,STOP\r\n");
    } else if (strcmp(command, "GET") == 0) {
        char response[64];
        (void) snprintf(response, sizeof(response),
            "PARAM,%ld,%ld,%ld,%ld\r\n",
            (long) g_speed_tune_kp_x1000,
            (long) g_speed_tune_ki_x1000,
            (long) g_speed_tune_kd_x1000,
            (long) g_speed_tune_target);
        OPENMV_WriteString(response);
    } else if (strcmp(command, "STATUS") == 0) {
        char response[64];
        (void) snprintf(response, sizeof(response),
            "# STATUS: P=%ld.%03ld I=%ld.%03ld D=%ld.%03ld SP=%ld\r\n",
            (long) (g_speed_tune_kp_x1000 / 1000L),
            (long) (g_speed_tune_kp_x1000 % 1000L),
            (long) (g_speed_tune_ki_x1000 / 1000L),
            (long) (g_speed_tune_ki_x1000 % 1000L),
            (long) (g_speed_tune_kd_x1000 / 1000L),
            (long) (g_speed_tune_kd_x1000 % 1000L),
            (long) g_speed_tune_target);
        OPENMV_WriteString(response);
    } else {
        OPENMV_WriteString("ERR,COMMAND\r\n");
    }
}

static void SpeedTune_ServiceUart(void)
{
    uint8_t byte;

    while (OPENMV_ReadByte(&byte)) {
        if ((byte == '\r') || (byte == '\n')) {
            if (g_speed_tune_uart_rx_index != 0U) {
                g_speed_tune_uart_rx[g_speed_tune_uart_rx_index] = '\0';
                SpeedTune_ProcessUartCommand(g_speed_tune_uart_rx);
                g_speed_tune_uart_rx_index = 0U;
            }
        } else if (g_speed_tune_uart_rx_index <
                   (sizeof(g_speed_tune_uart_rx) - 1U)) {
            g_speed_tune_uart_rx[g_speed_tune_uart_rx_index++] = (char) byte;
        } else {
            g_speed_tune_uart_rx_index = 0U;
        }
    }
}

static TaskManagerResult SpeedTuneTask_Update(uint32_t now_ms)
{
    EncoderData left = Encoder_GetData(ENCODER_1);
    EncoderData right = Encoder_GetData(ENCODER_2);
    uint8_t edges = TaskManager_GetPressedEdges();
    float left_pwm;
    float right_pwm;
    int32_t left_display;
    int32_t right_display;
    uint8_t index;

    SpeedTune_ServiceUart();
    if (g_speed_tune_stop_requested != 0U) {
        return TASK_MANAGER_RESULT_COMPLETE;
    }

    if ((edges & BUTTON_1_MASK) != 0U) {
        if (g_speed_tune_target < 100L) g_speed_tune_target++;
    }
    if ((edges & BUTTON_2_MASK) != 0U) {
        if (g_speed_tune_target > 0L) g_speed_tune_target--;
    }
    if ((edges & BUTTON_3_MASK) != 0U) {
        g_speed_tune_target = SPEED_TUNE_TARGET_PERCENT;
        g_speed_tune_left_integral = 0.0f;
        g_speed_tune_right_integral = 0.0f;
        g_speed_tune_left_last_error = 0.0f;
        g_speed_tune_right_last_error = 0.0f;
    }

    if ((now_ms - g_speed_tune_control_ms) >= ENCODER_SAMPLE_PERIOD_MS) {
        float raw_left = (float) left.counts_per_second * 100.0f /
                         (float) ENCODER_SPEED_100_PERCENT_CPS;
        float raw_right = (float) right.counts_per_second * 100.0f /
                          (float) ENCODER_SPEED_100_PERCENT_CPS;
        float left_error;
        float right_error;
        float kp = (float) g_speed_tune_kp_x1000 / 1000.0f;
        float ki = (float) g_speed_tune_ki_x1000 / 1000.0f;
        float kd = (float) g_speed_tune_kd_x1000 / 1000.0f;

        if (raw_left < 0.0f) raw_left = -raw_left;
        if (raw_right < 0.0f) raw_right = -raw_right;

        g_speed_tune_control_ms += ENCODER_SAMPLE_PERIOD_MS;
        if (g_speed_tune_filter_valid == 0U) {
            g_speed_tune_left_filtered = raw_left;
            g_speed_tune_right_filtered = raw_right;
            g_speed_tune_filter_valid = 1U;
        } else {
            /* 100 Hz一阶低通，约50 ms时间常数，抑制单计数造成的2.5级跳变。 */
            g_speed_tune_left_filtered += 0.20f *
                (raw_left - g_speed_tune_left_filtered);
            g_speed_tune_right_filtered += 0.20f *
                (raw_right - g_speed_tune_right_filtered);
        }

        left_error = (float) g_speed_tune_target -
                     g_speed_tune_left_filtered;
        right_error = (float) g_speed_tune_target -
                      g_speed_tune_right_filtered;
        g_speed_tune_left_integral += left_error * 0.01f;
        g_speed_tune_right_integral += right_error * 0.01f;
        if (g_speed_tune_left_integral > 100.0f)
            g_speed_tune_left_integral = 100.0f;
        if (g_speed_tune_left_integral < 0.0f)
            g_speed_tune_left_integral = 0.0f;
        if (g_speed_tune_right_integral > 100.0f)
            g_speed_tune_right_integral = 100.0f;
        if (g_speed_tune_right_integral < 0.0f)
            g_speed_tune_right_integral = 0.0f;

        left_pwm = kp * left_error + ki * g_speed_tune_left_integral +
                   kd * (left_error - g_speed_tune_left_last_error) / 0.01f;
        right_pwm = kp * right_error + ki * g_speed_tune_right_integral +
                    kd * (right_error - g_speed_tune_right_last_error) / 0.01f;
        g_speed_tune_left_last_error = left_error;
        g_speed_tune_right_last_error = right_error;
        if (left_pwm > 100.0f) left_pwm = 100.0f;
        if (left_pwm < 0.0f) left_pwm = 0.0f;
        if (right_pwm > 100.0f) right_pwm = 100.0f;
        if (right_pwm < 0.0f) right_pwm = 0.0f;
        Motor_SetDuty(MOTOR_LEFT, (int8_t) (left_pwm + 0.5f));
        Motor_SetDuty(MOTOR_RIGHT, (int8_t) (right_pwm + 0.5f));
        g_speed_tune_left_pwm = (int32_t) (left_pwm + 0.5f);
        g_speed_tune_right_pwm = (int32_t) (right_pwm + 0.5f);
    }

    left_display = (int32_t) (g_speed_tune_left_filtered + 0.5f);
    right_display = (int32_t) (g_speed_tune_right_filtered + 0.5f);

    if ((now_ms - g_speed_tune_wave_ms) >= SPEED_TUNE_UART_PERIOD_MS) {
        char uart_frame[96];
        int32_t input_x100;
        int32_t pwm_x100;
        int32_t error_x100;
        int32_t error_abs_x100;

        g_speed_tune_wave_ms = now_ms;
        Param_SendSpeedWaveform(g_speed_tune_target,
                                left_display,
                                right_display);
        input_x100 = (int32_t) (((g_speed_tune_left_filtered +
                                  g_speed_tune_right_filtered) * 50.0f) + 0.5f);
        pwm_x100 = (g_speed_tune_left_pwm + g_speed_tune_right_pwm) * 50L;
        error_x100 = g_speed_tune_target * 100L - input_x100;
        error_abs_x100 = (error_x100 < 0L) ? -error_x100 : error_x100;
        /* llm-pid-tuner: timestamp,setpoint,input,pwm,error,p,i,d */
        (void) snprintf(uart_frame, sizeof(uart_frame),
            "%lu,%ld.00,%ld.%02ld,%ld.%02ld,%c%ld.%02ld,"
            "%ld.%03ld,%ld.%03ld,%ld.%03ld\r\n",
            (unsigned long) now_ms, (long) g_speed_tune_target,
            (long) (input_x100 / 100L), (long) (input_x100 % 100L),
            (long) (pwm_x100 / 100L), (long) (pwm_x100 % 100L),
            (error_x100 < 0L) ? '-' : '+',
            (long) (error_abs_x100 / 100L),
            (long) (error_abs_x100 % 100L),
            (long) (g_speed_tune_kp_x1000 / 1000L),
            (long) (g_speed_tune_kp_x1000 % 1000L),
            (long) (g_speed_tune_ki_x1000 / 1000L),
            (long) (g_speed_tune_ki_x1000 % 1000L),
            (long) (g_speed_tune_kd_x1000 / 1000L),
            (long) (g_speed_tune_kd_x1000 % 1000L));
        OPENMV_WriteString(uart_frame);
    }

    if ((now_ms - g_speed_tune_status_ms) >= OLED_UPDATE_MS) {
        g_speed_tune_status_ms = now_ms;
        LineClear(g_speed_tune_status);
        index = 0U;
        LineAppendStr(g_speed_tune_status, &index, "T:");
        LineAppendInt(g_speed_tune_status, &index, g_speed_tune_target);
        LineAppendStr(g_speed_tune_status, &index, " L:");
        LineAppendInt(g_speed_tune_status, &index, left_display);
        LineAppendStr(g_speed_tune_status, &index, " R:");
        LineAppendInt(g_speed_tune_status, &index, right_display);
        TaskManager_SetStatus(g_speed_tune_status);
    }
    return TASK_MANAGER_RESULT_RUNNING;
}

static void SpeedTuneTask_Cancel(void)
{
    g_speed_tune_active = 0U;
    Motor_SetDuty(MOTOR_LEFT, 0);
    Motor_SetDuty(MOTOR_RIGHT, 0);
    Motor_Off();
    g_speed_tune_left_integral = 0.0f;
    g_speed_tune_right_integral = 0.0f;
}

static char EncoderCheckSignChar(int8_t sign)
{
    if (sign > 0) return '+';
    if (sign < 0) return '-';
    return '?';
}

static void EncoderCheckTask_Start(uint32_t now_ms)
{
    /* 映射未知时禁止闭环参与，本任务直接使用低PWM测试。 */
    MotionControl_Stop();
    MotorEncoderCheck_Start(now_ms);
    g_encoder_check_last_state = MotorEncoderCheck_GetState();
    g_encoder_check_finish_ms = 0U;
    g_encoder_check_reported = 0U;
    TaskManager_SetStatus("WATCH MOTOR LEFT");
}

static TaskManagerResult EncoderCheckTask_Update(uint32_t now_ms)
{
    MotorEncoderCheckState state = MotorEncoderCheck_Update(now_ms);

    if (state != g_encoder_check_last_state) {
        g_encoder_check_last_state = state;
        if (state == MOTOR_ENCODER_CHECK_PAUSE) {
            TaskManager_SetStatus("PAUSE");
        } else if (state == MOTOR_ENCODER_CHECK_RIGHT_RUNNING) {
            TaskManager_SetStatus("WATCH MOTOR RIGHT");
        }
    }

    if (state == MOTOR_ENCODER_CHECK_RESULT) {
        MotorEncoderCheckResult result = MotorEncoderCheck_GetResult();
        uint8_t index = 0U;
        uint8_t mapping_valid;

        mapping_valid =
            (result.left_motor_encoder != 0U) &&
            (result.right_motor_encoder != 0U) &&
            (result.left_motor_encoder != result.right_motor_encoder);

        if (g_encoder_check_reported == 0U) {
            g_encoder_check_reported = 1U;
            LineClear(g_encoder_check_status);
            LineAppendStr(g_encoder_check_status, &index, "ML:E");
            LineAppendChar(g_encoder_check_status, &index,
                (result.left_motor_encoder != 0U) ?
                (char) ('0' + result.left_motor_encoder) : '?');
            LineAppendChar(g_encoder_check_status, &index,
                EncoderCheckSignChar(result.left_encoder_sign));
            LineAppendStr(g_encoder_check_status, &index, " MR:E");
            LineAppendChar(g_encoder_check_status, &index,
                (result.right_motor_encoder != 0U) ?
                (char) ('0' + result.right_motor_encoder) : '?');
            LineAppendChar(g_encoder_check_status, &index,
                EncoderCheckSignChar(result.right_encoder_sign));
            TaskManager_SetStatus(g_encoder_check_status);

            Param_SendResponse("CHECK",
                "ML:E%u%c,D1=%ld,D2=%ld;MR:E%u%c,D1=%ld,D2=%ld;REV1=%u,REV2=%u",
                (unsigned int) result.left_motor_encoder,
                EncoderCheckSignChar(result.left_encoder_sign),
                (long) result.left_test_encoder1_delta,
                (long) result.left_test_encoder2_delta,
                (unsigned int) result.right_motor_encoder,
                EncoderCheckSignChar(result.right_encoder_sign),
                (long) result.right_test_encoder1_delta,
                (long) result.right_test_encoder2_delta,
                (unsigned int) (ENCODER_1_REVERSED ? 1U : 0U),
                (unsigned int) (ENCODER_2_REVERSED ? 1U : 0U));
            g_encoder_check_finish_ms = now_ms + 2000U;
        }

        if ((int32_t) (now_ms - g_encoder_check_finish_ms) >= 0) {
            return (mapping_valid != 0U) ?
                TASK_MANAGER_RESULT_COMPLETE : TASK_MANAGER_RESULT_FAILED;
        }
    }
    return TASK_MANAGER_RESULT_RUNNING;
}

static void EncoderCheckTask_Cancel(void)
{
    MotorEncoderCheck_Stop();
    MotionControl_Stop();
}

static void ImuZeroTask_Start(uint32_t now_ms)
{
    BMI088_Gyro_ResetAngles();
    TaskManager_SetStatus("ANGLES RESET");
    g_task_deadline_ms = now_ms + 600U;
}

static TaskManagerResult ShortTask_Update(uint32_t now_ms)
{
    if ((int32_t) (now_ms - g_task_deadline_ms) >= 0) {
        return TASK_MANAGER_RESULT_COMPLETE;
    }
    return TASK_MANAGER_RESULT_RUNNING;
}

static void EncoderResetTask_Start(uint32_t now_ms)
{
    Encoder_ResetAll();
    TaskManager_SetStatus("COUNTERS RESET");
    g_task_deadline_ms = now_ms + 600U;
}

/*
 * 每圈计数标定：启动时清零，随后只显示累计位置，不驱动电机。
 * 用户分别将左右轮向前手转10圈，最终计数绝对值除以10即为每圈有效计数。
 */
static void EncoderCprTask_Start(uint32_t now_ms)
{
    MotionControl_Stop();
    Motor_Off();
    Encoder_ResetAll();
    g_encoder_cpr_status_ms = now_ms - OLED_UPDATE_MS;
    TaskManager_SetStatus("TURN BOTH 10 TURNS");
}

static TaskManagerResult EncoderCprTask_Update(uint32_t now_ms)
{
    EncoderData encoder1;
    EncoderData encoder2;
    uint8_t index;

    if ((now_ms - g_encoder_cpr_status_ms) < OLED_UPDATE_MS) {
        return TASK_MANAGER_RESULT_RUNNING;
    }
    g_encoder_cpr_status_ms = now_ms;
    encoder1 = Encoder_GetData(ENCODER_1);
    encoder2 = Encoder_GetData(ENCODER_2);

    LineClear(g_encoder_cpr_status);
    index = 0U;
    LineAppendStr(g_encoder_cpr_status, &index, "E1:");
    LineAppendInt(g_encoder_cpr_status, &index, encoder1.count);
    LineAppendStr(g_encoder_cpr_status, &index, " E2:");
    LineAppendInt(g_encoder_cpr_status, &index, encoder2.count);
    TaskManager_SetStatus(g_encoder_cpr_status);
    return TASK_MANAGER_RESULT_RUNNING;
}

static void EncoderCprTask_Cancel(void)
{
    MotionControl_Stop();
    Motor_Off();
}

static int32_t MaxSpeed_Abs(int32_t value)
{
    return (value < 0L) ? (int32_t) (-(int64_t) value) : value;
}

static void MaxSpeed_ShowLive(const char *wheel, uint8_t duty,
                              int32_t counts_per_second)
{
    uint8_t index = 0U;

    LineClear(g_max_speed_status);
    LineAppendStr(g_max_speed_status, &index, wheel);
    LineAppendChar(g_max_speed_status, &index, ':');
    LineAppendInt(g_max_speed_status, &index, duty);
    LineAppendStr(g_max_speed_status, &index, "% CPS:");
    LineAppendInt(g_max_speed_status, &index,
                  MaxSpeed_Abs(counts_per_second));
    TaskManager_SetStatus(g_max_speed_status);
}

static void MaxSpeed_StartSampling(uint32_t now_ms)
{
    g_max_speed_sample_start_ms = now_ms + MAX_SPEED_SETTLE_MS;
    g_max_speed_last_sample_ms = g_max_speed_sample_start_ms;
    g_max_speed_deadline_ms = g_max_speed_sample_start_ms +
                              MAX_SPEED_SAMPLE_MS;
    g_max_speed_sum = 0LL;
    g_max_speed_samples = 0U;
    g_max_speed_peak = 0L;
}

static void MaxSpeed_Sample(uint32_t now_ms, EncoderId encoder)
{
    int32_t speed;

    if ((int32_t) (now_ms - g_max_speed_sample_start_ms) < 0) {
        return;
    }
    if ((now_ms - g_max_speed_last_sample_ms) < ENCODER_SAMPLE_PERIOD_MS) {
        return;
    }
    g_max_speed_last_sample_ms += ENCODER_SAMPLE_PERIOD_MS;
    speed = MaxSpeed_Abs(Encoder_GetData(encoder).counts_per_second);
    g_max_speed_sum += speed;
    g_max_speed_samples++;
    if (speed > g_max_speed_peak) {
        g_max_speed_peak = speed;
    }
}

static int32_t MaxSpeed_GetAverage(void)
{
    if (g_max_speed_samples == 0U) {
        return 0L;
    }
    return (int32_t) (g_max_speed_sum / g_max_speed_samples);
}

static void MaxSpeedTask_Start(uint32_t now_ms)
{
    MotionControl_Stop();
    Motor_Off();
    Encoder_ResetAll();
    g_max_speed_left_average = 0L;
    g_max_speed_right_average = 0L;
    g_max_speed_left_peak = 0L;
    g_max_speed_right_peak = 0L;
    g_max_speed_status_ms = now_ms;
    Motor_SetDuty(MOTOR_RIGHT, 0);
    Motor_SetDuty(MOTOR_LEFT, 30);
    Motor_On();
    g_max_speed_deadline_ms = now_ms + MAX_SPEED_RAMP_MS;
    g_max_speed_state = MAX_SPEED_LEFT_30;
    TaskManager_SetStatus("LEFT 30% RAMP");
}

static TaskManagerResult MaxSpeedTask_Update(uint32_t now_ms)
{
    EncoderData data;
    uint8_t index;

    if ((now_ms - g_max_speed_status_ms) >= OLED_UPDATE_MS) {
        g_max_speed_status_ms = now_ms;
        if ((g_max_speed_state >= MAX_SPEED_LEFT_30) &&
            (g_max_speed_state <= MAX_SPEED_LEFT_100)) {
            data = Encoder_GetData(ENCODER_1);
            MaxSpeed_ShowLive("L",
                (g_max_speed_state == MAX_SPEED_LEFT_30) ? 30U :
                ((g_max_speed_state == MAX_SPEED_LEFT_60) ? 60U : 100U),
                data.counts_per_second);
        } else if ((g_max_speed_state >= MAX_SPEED_RIGHT_30) &&
                   (g_max_speed_state <= MAX_SPEED_RIGHT_100)) {
            data = Encoder_GetData(ENCODER_2);
            MaxSpeed_ShowLive("R",
                (g_max_speed_state == MAX_SPEED_RIGHT_30) ? 30U :
                ((g_max_speed_state == MAX_SPEED_RIGHT_60) ? 60U : 100U),
                data.counts_per_second);
        }
    }

    if (g_max_speed_state == MAX_SPEED_LEFT_100) {
        MaxSpeed_Sample(now_ms, ENCODER_1);
    } else if (g_max_speed_state == MAX_SPEED_RIGHT_100) {
        MaxSpeed_Sample(now_ms, ENCODER_2);
    }

    if ((int32_t) (now_ms - g_max_speed_deadline_ms) < 0) {
        return TASK_MANAGER_RESULT_RUNNING;
    }

    switch (g_max_speed_state) {
        case MAX_SPEED_LEFT_30:
            Motor_SetDuty(MOTOR_LEFT, 60);
            g_max_speed_deadline_ms = now_ms + MAX_SPEED_RAMP_MS;
            g_max_speed_state = MAX_SPEED_LEFT_60;
            break;
        case MAX_SPEED_LEFT_60:
            Motor_SetDuty(MOTOR_LEFT, 100);
            MaxSpeed_StartSampling(now_ms);
            g_max_speed_state = MAX_SPEED_LEFT_100;
            break;
        case MAX_SPEED_LEFT_100:
            g_max_speed_left_average = MaxSpeed_GetAverage();
            g_max_speed_left_peak = g_max_speed_peak;
            Motor_SetDuty(MOTOR_LEFT, 0);
            Motor_Off();
            g_max_speed_deadline_ms = now_ms + MAX_SPEED_PAUSE_MS;
            g_max_speed_state = MAX_SPEED_PAUSE;
            TaskManager_SetStatus("PAUSE");
            break;
        case MAX_SPEED_PAUSE:
            Encoder_Reset(ENCODER_2);
            Motor_SetDuty(MOTOR_LEFT, 0);
            Motor_SetDuty(MOTOR_RIGHT, 30);
            Motor_On();
            g_max_speed_deadline_ms = now_ms + MAX_SPEED_RAMP_MS;
            g_max_speed_state = MAX_SPEED_RIGHT_30;
            break;
        case MAX_SPEED_RIGHT_30:
            Motor_SetDuty(MOTOR_RIGHT, 60);
            g_max_speed_deadline_ms = now_ms + MAX_SPEED_RAMP_MS;
            g_max_speed_state = MAX_SPEED_RIGHT_60;
            break;
        case MAX_SPEED_RIGHT_60:
            Motor_SetDuty(MOTOR_RIGHT, 100);
            MaxSpeed_StartSampling(now_ms);
            g_max_speed_state = MAX_SPEED_RIGHT_100;
            break;
        case MAX_SPEED_RIGHT_100:
            g_max_speed_right_average = MaxSpeed_GetAverage();
            g_max_speed_right_peak = g_max_speed_peak;
            Motor_SetDuty(MOTOR_RIGHT, 0);
            Motor_Off();
            LineClear(g_max_speed_status);
            index = 0U;
            LineAppendStr(g_max_speed_status, &index, "L:");
            LineAppendInt(g_max_speed_status, &index,
                          g_max_speed_left_average);
            LineAppendStr(g_max_speed_status, &index, " R:");
            LineAppendInt(g_max_speed_status, &index,
                          g_max_speed_right_average);
            TaskManager_SetStatus(g_max_speed_status);
            LineClear(g_max_speed_status);
            index = 0U;
            LineAppendStr(g_max_speed_status, &index, "L AVG:");
            LineAppendInt(g_max_speed_status, &index,
                          g_max_speed_left_average);
            LineAppendStr(g_max_speed_status, &index, " P:");
            LineAppendInt(g_max_speed_status, &index,
                          g_max_speed_left_peak);
            Bluetooth_WriteString(g_max_speed_status);
            Bluetooth_WriteString("\r\n");
            LineClear(g_max_speed_status);
            index = 0U;
            LineAppendStr(g_max_speed_status, &index, "R AVG:");
            LineAppendInt(g_max_speed_status, &index,
                          g_max_speed_right_average);
            LineAppendStr(g_max_speed_status, &index, " P:");
            LineAppendInt(g_max_speed_status, &index,
                          g_max_speed_right_peak);
            Bluetooth_WriteString(g_max_speed_status);
            Bluetooth_WriteString("\r\n");

            /* OLED最终只显示两侧稳定平均值，蓝牙额外报告峰值。 */
            LineClear(g_max_speed_status);
            index = 0U;
            LineAppendStr(g_max_speed_status, &index, "L:");
            LineAppendInt(g_max_speed_status, &index,
                          g_max_speed_left_average);
            LineAppendStr(g_max_speed_status, &index, " R:");
            LineAppendInt(g_max_speed_status, &index,
                          g_max_speed_right_average);
            TaskManager_SetStatus(g_max_speed_status);
            g_max_speed_deadline_ms = now_ms + MAX_SPEED_RESULT_MS;
            g_max_speed_state = MAX_SPEED_RESULT;
            break;
        case MAX_SPEED_RESULT:
            return TASK_MANAGER_RESULT_COMPLETE;
        default:
            return TASK_MANAGER_RESULT_FAILED;
    }
    return TASK_MANAGER_RESULT_RUNNING;
}

static void MaxSpeedTask_Cancel(void)
{
    Motor_SetDuty(MOTOR_LEFT, 0);
    Motor_SetDuty(MOTOR_RIGHT, 0);
    Motor_Off();
    MotionControl_Stop();
    g_max_speed_state = MAX_SPEED_IDLE;
}

static void SensorViewTask_Start(uint32_t now_ms)
{
    TaskManager_SetStatus("OBSERVE 3 SEC");
    g_task_deadline_ms = now_ms + 3000U;
}

static void LineFollowTask_Start(uint32_t now_ms)
{
    LineFollow_Start(now_ms);
    g_line_follow_last_state = LineFollow_GetState();
    TaskManager_SetStatus(LineFollow_GetStatusText());
}

static TaskManagerResult LineFollowTask_Update(uint32_t now_ms)
{
    LineFollowState state;

    LineFollow_Update(now_ms);
    state = LineFollow_GetState();
    if (state != g_line_follow_last_state) {
        g_line_follow_last_state = state;
        TaskManager_SetStatus(LineFollow_GetStatusText());
    }
    return TASK_MANAGER_RESULT_RUNNING;
}

static void LineFollowTask_Cancel(void)
{
    LineFollow_Stop();
}

static void K230LineTask_Start(uint32_t now_ms)
{
    K230LineFollow_Start(now_ms);
    g_k230_line_last_state = K230LineFollow_GetState();
    g_k230_status_ms = now_ms;
    g_k230_finish_ms = 0U;
    TaskManager_SetStatus(K230LineFollow_GetStatusText());
}

static TaskManagerResult K230LineTask_Update(uint32_t now_ms)
{
    VisionLineState state = K230LineFollow_Update(now_ms);

    if (state != g_k230_line_last_state) {
        g_k230_line_last_state = state;
        if (state != VISION_LINE_STATE_TRACKING) {
            TaskManager_SetStatus(K230LineFollow_GetStatusText());
        }
        if ((state == VISION_LINE_STATE_REMOTE_STOP) ||
            (state == VISION_LINE_STATE_TIMEOUT)) {
            g_k230_finish_ms = now_ms + 600U;
        }
    }
    if ((state == VISION_LINE_STATE_TRACKING) &&
        ((now_ms - g_k230_status_ms) >= OLED_UPDATE_MS)) {
        g_k230_status_ms = now_ms;
        FormatVisionStatus(g_k230_status, "K230",
                           K230LineFollow_GetCenterX());
        TaskManager_SetStatus(g_k230_status);
    }
    if (state == VISION_LINE_STATE_REMOTE_STOP) {
        return ((int32_t) (now_ms - g_k230_finish_ms) >= 0)
            ? TASK_MANAGER_RESULT_COMPLETE
            : TASK_MANAGER_RESULT_RUNNING;
    }
    if (state == VISION_LINE_STATE_TIMEOUT) {
        return ((int32_t) (now_ms - g_k230_finish_ms) >= 0)
            ? TASK_MANAGER_RESULT_FAILED
            : TASK_MANAGER_RESULT_RUNNING;
    }
    return TASK_MANAGER_RESULT_RUNNING;
}

static void K230LineTask_Cancel(void)
{
    K230LineFollow_Stop();
}

static void OpenMVLineTask_Start(uint32_t now_ms)
{
    OpenMVLineFollow_Start(now_ms);
    g_openmv_line_last_state = OpenMVLineFollow_GetState();
    g_openmv_status_ms = now_ms;
    g_openmv_finish_ms = 0U;
    TaskManager_SetStatus(OpenMVLineFollow_GetStatusText());
}

static TaskManagerResult OpenMVLineTask_Update(uint32_t now_ms)
{
    VisionLineState state = OpenMVLineFollow_Update(now_ms);

    if (state != g_openmv_line_last_state) {
        g_openmv_line_last_state = state;
        if (state != VISION_LINE_STATE_TRACKING) {
            TaskManager_SetStatus(OpenMVLineFollow_GetStatusText());
        }
        if ((state == VISION_LINE_STATE_REMOTE_STOP) ||
            (state == VISION_LINE_STATE_TIMEOUT)) {
            g_openmv_finish_ms = now_ms + 600U;
        }
    }
    if ((state == VISION_LINE_STATE_TRACKING) &&
        ((now_ms - g_openmv_status_ms) >= OLED_UPDATE_MS)) {
        g_openmv_status_ms = now_ms;
        FormatVisionStatus(g_openmv_status, "OPENMV",
                           OpenMVLineFollow_GetCenterX());
        TaskManager_SetStatus(g_openmv_status);
    }
    if (state == VISION_LINE_STATE_REMOTE_STOP) {
        return ((int32_t) (now_ms - g_openmv_finish_ms) >= 0)
            ? TASK_MANAGER_RESULT_COMPLETE
            : TASK_MANAGER_RESULT_RUNNING;
    }
    if (state == VISION_LINE_STATE_TIMEOUT) {
        return ((int32_t) (now_ms - g_openmv_finish_ms) >= 0)
            ? TASK_MANAGER_RESULT_FAILED
            : TASK_MANAGER_RESULT_RUNNING;
    }
    return TASK_MANAGER_RESULT_RUNNING;
}

static void OpenMVLineTask_Cancel(void)
{
    OpenMVLineFollow_Stop();
}

static void MagCalibrationTask_Start(uint32_t now_ms)
{
    if (g_mag_init_status != MMC5983MA_OK) {
        TaskManager_SetStatus("MAG NOT FOUND");
        g_task_deadline_ms = now_ms + 1000U;
        return;
    }
    MMC5983MA_StartCalibration();
    BMI088_Gyro_ResetMagReference();
    TaskManager_SetStatus("ROTATE ALL AXES");
    g_task_deadline_ms = now_ms + MAG_CALIBRATION_MS;
}

static TaskManagerResult MagCalibrationTask_Update(uint32_t now_ms)
{
    if (g_mag_init_status != MMC5983MA_OK) {
        return ((int32_t) (now_ms - g_task_deadline_ms) >= 0)
            ? TASK_MANAGER_RESULT_FAILED
            : TASK_MANAGER_RESULT_RUNNING;
    }
    if ((int32_t) (now_ms - g_task_deadline_ms) < 0) {
        return TASK_MANAGER_RESULT_RUNNING;
    }
    if (MMC5983MA_FinishCalibration() != MMC5983MA_OK) {
        TaskManager_SetStatus("MAG CAL FAILED");
        return TASK_MANAGER_RESULT_FAILED;
    }
    BMI088_Gyro_ResetMagReference();
    TaskManager_SetStatus("MAG CAL OK");
    return TASK_MANAGER_RESULT_COMPLETE;
}

static void MagCalibrationTask_Cancel(void)
{
    MMC5983MA_CancelCalibration();
}

static const TaskManagerTask g_tasks[] = {
    {"MOTOR ENCODER CHECK", EncoderCheckTask_Start,
     EncoderCheckTask_Update, EncoderCheckTask_Cancel},
    {"BALANCE HOLD", BalanceTask_Start, BalanceTask_Update,
     BalanceTask_Cancel},
    {"ANGLE DIR CHECK", AngleDirTask_Start,
     AngleDirTask_Update, AngleDirTask_Cancel},
    {"ANGLE PD TUNE", AngleTuneTask_Start,
     AngleTuneTask_Update, AngleTuneTask_Cancel},
    {"SPEED PID TUNE", SpeedTuneTask_Start,
     SpeedTuneTask_Update, SpeedTuneTask_Cancel},
    {"MOTOR TEST", MotorTask_Start, MotorTask_Update, MotorTask_Cancel},
    {"IMU ZERO", ImuZeroTask_Start, ShortTask_Update, 0},
    {"ENCODER RESET", EncoderResetTask_Start, ShortTask_Update, 0},
    {"ENCODER CPR 10T", EncoderCprTask_Start,
     EncoderCprTask_Update, EncoderCprTask_Cancel},
    {"MAX SPEED TEST", MaxSpeedTask_Start,
     MaxSpeedTask_Update, MaxSpeedTask_Cancel},
    {"SENSOR VIEW", SensorViewTask_Start, ShortTask_Update, 0},
    {"MAG CAL 20S", MagCalibrationTask_Start,
     MagCalibrationTask_Update, MagCalibrationTask_Cancel},
    {"LINE FOLLOW", LineFollowTask_Start, LineFollowTask_Update,
     LineFollowTask_Cancel},
    {"K230 VISION LINE", K230LineTask_Start, K230LineTask_Update,
     K230LineTask_Cancel},
    {"OPENMV VISION LINE", OpenMVLineTask_Start, OpenMVLineTask_Update,
     OpenMVLineTask_Cancel}
};

void PeripheralTest_Init(void)
{
    g_millis = 0U;
    (void) DL_SYSTICK_config(CPUCLK_FREQ / 1000U);

    Param_Init();
    OPENMV_Init();
    openmv_Init();

    OLED_Init();

    MotionControl_Init();
    BalanceControl_RegisterBluetoothParams();
    (void) Param_RegisterSlider("SPD_PID_KP_X1000",
        &g_speed_tune_kp_x1000, PARAM_TYPE_INT32, 0L, 5000L);
    (void) Param_RegisterSlider("SPD_PID_KI_X1000",
        &g_speed_tune_ki_x1000, PARAM_TYPE_INT32, 0L, 1000L);
    (void) Param_RegisterSlider("SPD_PID_KD_X1000",
        &g_speed_tune_kd_x1000, PARAM_TYPE_INT32, 0L, 5000L);
    MotorEncoderCheck_Init();
    Encoder_Init(ENCODER_1_COUNTS_PER_REV, ENCODER_2_COUNTS_PER_REV);
    Encoder_SetReversed(ENCODER_1, ENCODER_1_REVERSED);
    Encoder_SetReversed(ENCODER_2, ENCODER_2_REVERSED);
    MotorTest_Init();
    LineFollow_Init();
    K230LineFollow_Init();
    OpenMVLineFollow_Init();
    TaskManager_Init(g_tasks, (uint8_t) (sizeof(g_tasks) / sizeof(g_tasks[0])));
    g_gyro_init_status = BMI088_Gyro_Init();
    g_mag_init_status = MMC5983MA_Init();

    if (g_gyro_init_status == BMI088_GYRO_OK) {
        /* 校准用于消除引起漂移的静止角速度零偏。 */
        ShowLine(1U, "IMU CALIBRATING");
        ShowLine(2U, "KEEP BOARD STILL");
        g_gyro_init_status = BMI088_Gyro_CalibrationStart(
            GYRO_CALIBRATE_SAMPLES, g_millis);
    }

    g_last_bluetooth_ms = g_millis;
    g_speed_tune_active = 0U;
}

void PeripheralTest_Run(void)
{
    BMI088_GyroData gyro;
    uint8_t key_pressed = 0U;
    uint8_t key_configured;
    uint8_t line_raw;
    uint8_t line_active;
    uint32_t last_imu_update_ms;
    uint32_t last_gyro_retry_ms;
    uint32_t last_oled_update_ms;
    uint32_t last_mag_retry_ms;

    gyro = *BMI088_Gyro_GetData();
    key_configured = PT_Button_GetConfiguredMask();
    last_imu_update_ms = g_millis;
    last_gyro_retry_ms = g_millis;
    last_oled_update_ms = g_millis - OLED_UPDATE_MS;
    last_mag_retry_ms = g_millis;

    while (1) {
        OLED_Service();
        Param_ServiceTx();
        OPENMV_Service();
        openmv_Service();
        Param_Process();
        TaskManager_Update(g_millis);
        key_pressed = PT_Button_GetPressedMask();
        line_raw = LineSensor_ReadRawMask();
        line_active = LineSensor_ReadActiveMask();

        if (g_gyro_init_status == BMI088_GYRO_CALIBRATING) {
            g_gyro_init_status = BMI088_Gyro_CalibrationService(g_millis);
            gyro = *BMI088_Gyro_GetData();
            if (g_gyro_init_status == BMI088_GYRO_OK) {
                last_imu_update_ms = g_millis;
                BalanceControl_NotifyImuSample(true);
            }
        } else if (g_gyro_init_status == BMI088_GYRO_OK) {
            ServiceImu(&gyro, &last_imu_update_ms);
        } else {
            gyro = *BMI088_Gyro_GetData();
            if ((g_millis - last_gyro_retry_ms) >= 1000U) {
                last_gyro_retry_ms = g_millis;
                g_gyro_init_status = BMI088_Gyro_Init();
                if (g_gyro_init_status == BMI088_GYRO_OK) {
                    g_gyro_init_status = BMI088_Gyro_CalibrationStart(
                        GYRO_CALIBRATE_SAMPLES, g_millis);
                }
                gyro = *BMI088_Gyro_GetData();
                last_imu_update_ms = g_millis;
            }
        }

        /* 这里只更新100 Hz位置兼容层，三环计算在2 ms定时中断完成。 */
        MotionControl_Service();

        /* 磁力计低优先级读取；标定有效后缓慢修正BMI088的yaw漂移。 */
        if (g_mag_init_status == MMC5983MA_OK) {
            int mag_result = MMC5983MA_Service(
                g_millis, gyro.roll_x10, gyro.pitch_x10);
            const MMC5983MA_Data *mag = MMC5983MA_GetData();

            if ((mag_result == MMC5983MA_OK) &&
                (mag != 0) && mag->heading_valid) {
                BMI088_Gyro_FuseMagHeading(
                    mag->heading_x10, MAG_YAW_GAIN_X1000);
                gyro = *BMI088_Gyro_GetData();
            } else if (mag_result == MMC5983MA_ERR_I2C) {
                g_mag_init_status = mag_result;
            }
        } else if ((g_millis - last_mag_retry_ms) >= 1000U) {
            last_mag_retry_ms = g_millis;
            g_mag_init_status = MMC5983MA_Init();
        }

        /* IMU以500 Hz调度；OLED仍限制到10 Hz，避免影响控制采样。 */
        if (TaskManager_IsIdle()) {
            if ((g_millis - last_oled_update_ms) >= OLED_UPDATE_MS) {
                last_oled_update_ms = g_millis;
                ShowPage(&gyro, key_pressed, key_configured,
                         line_raw, line_active);
            }
        } else {
            /* 返回初始状态后立即恢复外设监视页。 */
            last_oled_update_ms = g_millis - OLED_UPDATE_MS;
        }
        /* 短暂空闲延时让 SysTick 提供真实时基，避免依赖循环耗时假设。 */
    }
}
