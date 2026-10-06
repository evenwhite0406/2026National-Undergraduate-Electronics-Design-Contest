/**
 * @file line_sensor_bt_test.c
 * @brief 八路灰度传感器蓝牙静态测试。
 */

#include "line_sensor_bt_test.h"
#include "line_sensor.h"
#include "../Hardware/bluetooth.h"
#include "../Hardware/motor.h"
#include <stdio.h>

#define LINE_SENSOR_BT_SAMPLE_MS    (5U)
#define LINE_SENSOR_BT_SEND_MS      (100U)

static uint32_t s_last_sample_ms;
static uint32_t s_last_send_ms;

static uint8_t s_raw_mask;
static uint8_t s_active_mask;
static uint8_t s_filtered_mask;

static void LineSensorBtTest_Sample(void)
{
    uint8_t configured_mask;

    /*
     * 只读取一次物理电平，再由同一份数据计算active，
     * 避免RAW和ACT来自两个不同时刻。
     */
    s_raw_mask = LineSensor_ReadRawMask();
    configured_mask = LineSensor_GetConfiguredMask();

    if (LINE_SENSOR_ACTIVE_LOW != 0U) {
        s_active_mask =
            (uint8_t) ((~s_raw_mask) & configured_mask);
    } else {
        s_active_mask =
            (uint8_t) (s_raw_mask & configured_mask);
    }

    s_filtered_mask =
        LineSensor_FilterSample(s_active_mask);
}

void LineSensorBtTest_Init(void)
{
    s_last_sample_ms = 0U;
    s_last_send_ms = 0U;

    s_raw_mask = 0U;
    s_active_mask = 0U;
    s_filtered_mask = 0U;

    LineSensor_FilterReset();
}

void LineSensorBtTest_Start(uint32_t now_ms)
{
    /* 本任务只观察传感器，绝不允许电机运行。 */
    Motor_Off();

    LineSensor_FilterReset();

    s_raw_mask = 0U;
    s_active_mask = 0U;
    s_filtered_mask = 0U;

    s_last_sample_ms = now_ms;
    s_last_send_ms = now_ms;

    LineSensorBtTest_Sample();

    Bluetooth_WriteString(
        "\r\nLINE SENSOR BT START\r\n");
}

TaskManagerResult LineSensorBtTest_Update(uint32_t now_ms)
{
    char raw_text[LINE_SENSOR_COUNT + 1U];
    char active_text[LINE_SENSOR_COUNT + 1U];
    char filtered_text[LINE_SENSOR_COUNT + 1U];
    char message[80];
    uint8_t active_count;

    /* 每5 ms采样一次八路灰度。 */
    if ((uint32_t) (now_ms - s_last_sample_ms) >=
        LINE_SENSOR_BT_SAMPLE_MS) {
        s_last_sample_ms = now_ms;
        LineSensorBtTest_Sample();
    }

    /* 每100 ms通过蓝牙发送一次，避免刷屏过快。 */
    if ((uint32_t) (now_ms - s_last_send_ms) >=
        LINE_SENSOR_BT_SEND_MS) {
        s_last_send_ms = now_ms;

        LineSensor_FormatBits(s_raw_mask, raw_text);
        LineSensor_FormatBits(s_active_mask, active_text);
        LineSensor_FormatBits(s_filtered_mask, filtered_text);

        active_count =
            LineSensor_CountActive(s_filtered_mask);

        snprintf(message, sizeof(message),
                 "RAW:%s ACT:%s FIL:%s N:%u\r\n",
                 raw_text,
                 active_text,
                 filtered_text,
                 (unsigned int) active_count);

        (void) Bluetooth_WriteString(message);
    }

    return TASK_MANAGER_RESULT_RUNNING;
}

void LineSensorBtTest_Cancel(void)
{
    Motor_Off();

    s_raw_mask = 0U;
    s_active_mask = 0U;
    s_filtered_mask = 0U;

    LineSensor_FilterReset();

    (void) Bluetooth_WriteString(
        "LINE SENSOR BT STOP\r\n");
}