/**
 * @file line_follow_task.c
 * @brief 八路数字灰度循迹任务及蓝牙滑杆在线调参。
 *
 * 蓝牙滑杆协议：
 *   slider,参数名,数值\r\n
 *
 * 示例：
 *   slider,LINE_KP_X100,160
 *   slider,LINE_KD_X100,40
 *   slider,FILTER_NORMAL,5
 *   slider,FILTER_FAST,8
 *
 * 查询全部参数：
 *   get,all
 */

#include "line_follow_task.h"

#include "../Control/line_follow.h"
#include "vehicle_tuning.h"
#include "../Hardware/bluetooth.h"
#include "../Hardware/motor.h"

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* 一条蓝牙命令的最大长度。 */
#define LINE_FOLLOW_BT_RX_SIZE       (96U)

/* ============================= 内部状态 ============================= */

static LineFollowState s_last_state;

static char s_bt_rx_buffer[LINE_FOLLOW_BT_RX_SIZE];
static uint8_t s_bt_rx_index;
static uint8_t s_bt_frame_active;

/* ============================= 工具函数 ============================= */

static long LineFollowTask_LimitLong(long value,
                                     long minimum,
                                     long maximum)
{
    if (value < minimum) {
        return minimum;
    }

    if (value > maximum) {
        return maximum;
    }

    return value;
}

/**
 * 对相关参数做二次约束，避免滑杆组合产生明显不合理配置。
 */
static void LineFollowTask_NormalizeParams(void)
{
    if (LINE_MAX_SPEED < 1) {
        LINE_MAX_SPEED = 1;
    }

    if (LINE_MIN_SPEED < 0) {
        LINE_MIN_SPEED = 0;
    }

    if (LINE_MIN_SPEED > LINE_MAX_SPEED) {
        LINE_MIN_SPEED = LINE_MAX_SPEED;
    }

    if (LINE_BASE_SPEED < LINE_MIN_SPEED) {
        LINE_BASE_SPEED = LINE_MIN_SPEED;
    }

    if (LINE_BASE_SPEED > LINE_MAX_SPEED) {
        LINE_BASE_SPEED = LINE_MAX_SPEED;
    }

    if (LINE_STRAIGHT_SPEED < LINE_MIN_SPEED) {
        LINE_STRAIGHT_SPEED = LINE_MIN_SPEED;
    }

    if (LINE_STRAIGHT_SPEED > LINE_MAX_SPEED) {
        LINE_STRAIGHT_SPEED = LINE_MAX_SPEED;
    }

    if (LINE_INTERSECTION_SPEED < 0) {
        LINE_INTERSECTION_SPEED = 0;
    }

    if (LINE_INTERSECTION_SPEED > LINE_MAX_SPEED) {
        LINE_INTERSECTION_SPEED = LINE_MAX_SPEED;
    }

    if (LINE_MAX_CORRECTION < 0) {
        LINE_MAX_CORRECTION = 0;
    }

    if (LINE_MAX_CORRECTION > 100) {
        LINE_MAX_CORRECTION = 100;
    }

    if (LINE_RAMP_NORMAL < 1) {
        LINE_RAMP_NORMAL = 1;
    }

    if (LINE_RAMP_FAST < 1) {
        LINE_RAMP_FAST = 1;
    }

    if (g_line_filter_alpha_normal < 1) {
        g_line_filter_alpha_normal = 1;
    }

    if (g_line_filter_alpha_normal > 10) {
        g_line_filter_alpha_normal = 10;
    }

    if (g_line_filter_alpha_fast < 1) {
        g_line_filter_alpha_fast = 1;
    }

    if (g_line_filter_alpha_fast > 10) {
        g_line_filter_alpha_fast = 10;
    }

    /*
     * 丢线总停车时间不能短于继续前进时间，
     * 否则搜索阶段会被直接跳过。
     */
    if (LINE_LOST_STOP_MS < LINE_LOST_COAST_MS) {
        LINE_LOST_STOP_MS = LINE_LOST_COAST_MS;
    }
}

/* ============================= 参数上报 ============================= */

static void LineFollowTask_SendParams(void)
{
    char message[112];

    (void) snprintf(
        message,
        sizeof(message),
        "LF1 KP:%ld KD:%ld FN:%ld FF:%ld\r\n",
        (long) LINE_KP_X100,
        (long) LINE_KD_X100,
        (long) g_line_filter_alpha_normal,
        (long) g_line_filter_alpha_fast);
    (void) Bluetooth_WriteString(message);

    (void) snprintf(
        message,
        sizeof(message),
        "LF2 BASE:%d STR:%d MIN:%d MAX:%d\r\n",
        (int) LINE_BASE_SPEED,
        (int) LINE_STRAIGHT_SPEED,
        (int) LINE_MIN_SPEED,
        (int) LINE_MAX_SPEED);
    (void) Bluetooth_WriteString(message);

    (void) snprintf(
        message,
        sizeof(message),
        "LF3 COR:%d SLOW:%ld INT:%d RN:%d RF:%d\r\n",
        (int) LINE_MAX_CORRECTION,
        (long) LINE_CURVE_SLOWDOWN_X100,
        (int) LINE_INTERSECTION_SPEED,
        (int) LINE_RAMP_NORMAL,
        (int) LINE_RAMP_FAST);
    (void) Bluetooth_WriteString(message);

    (void) snprintf(
        message,
        sizeof(message),
        "LF4 LOSTF:%d LOSTT:%d COAST:%lu STOP:%lu\r\n",
        (int) LINE_LOST_FORWARD_SPEED,
        (int) LINE_LOST_TURN_SPEED,
        (unsigned long) LINE_LOST_COAST_MS,
        (unsigned long) LINE_LOST_STOP_MS);
    (void) Bluetooth_WriteString(message);
}

static void LineFollowTask_SendSliderResult(const char *name,
                                            long value)
{
    char message[96];

    (void) snprintf(
        message,
        sizeof(message),
        "OK:slider,%s,%ld\r\n",
        name,
        value);

    (void) Bluetooth_WriteString(message);
}

/* ============================= 滑杆处理 ============================= */

/**
 * 根据滑杆名称修改对应的全局循迹参数。
 *
 * 返回：
 *   1：参数存在并修改成功
 *   0：参数名不存在
 */
static uint8_t LineFollowTask_SetSlider(const char *name,
                                        long value)
{
    if (name == NULL) {
        return 0U;
    }

    /* ======================== PD与滤波 ======================== */

    if (strcmp(name, "LINE_KP_X100") == 0) {
        LINE_KP_X100 =
            (int32_t) LineFollowTask_LimitLong(value, 0L, 1000L);
    }
    else if (strcmp(name, "LINE_KD_X100") == 0) {
        LINE_KD_X100 =
            (int32_t) LineFollowTask_LimitLong(value, 0L, 1000L);
    }
    else if (strcmp(name, "FILTER_NORMAL") == 0) {
        g_line_filter_alpha_normal =
            (int32_t) LineFollowTask_LimitLong(value, 1L, 10L);
    }
    else if (strcmp(name, "FILTER_FAST") == 0) {
        g_line_filter_alpha_fast =
            (int32_t) LineFollowTask_LimitLong(value, 1L, 10L);
    }

    /* ======================== 基础速度 ======================== */

    else if (strcmp(name, "BASE_SPEED") == 0) {
        LINE_BASE_SPEED =
            (int16_t) LineFollowTask_LimitLong(value, 0L, 100L);
    }
    else if (strcmp(name, "STRAIGHT_SPEED") == 0) {
        LINE_STRAIGHT_SPEED =
            (int16_t) LineFollowTask_LimitLong(value, 0L, 100L);
    }
    else if (strcmp(name, "MIN_SPEED") == 0) {
        LINE_MIN_SPEED =
            (int16_t) LineFollowTask_LimitLong(value, 0L, 100L);
    }
    else if (strcmp(name, "MAX_SPEED") == 0) {
        LINE_MAX_SPEED =
            (int16_t) LineFollowTask_LimitLong(value, 1L, 100L);
    }
    else if (strcmp(name, "INTERSECTION_SPEED") == 0) {
        LINE_INTERSECTION_SPEED =
            (int16_t) LineFollowTask_LimitLong(value, 0L, 100L);
    }

    /* ======================== 转弯控制 ======================== */

    else if (strcmp(name, "MAX_CORRECTION") == 0) {
        LINE_MAX_CORRECTION =
            (int16_t) LineFollowTask_LimitLong(value, 0L, 100L);
    }
    else if (strcmp(name, "CURVE_SLOW_X100") == 0) {
        LINE_CURVE_SLOWDOWN_X100 =
            (int32_t) LineFollowTask_LimitLong(value, 0L, 500L);
    }
    else if (strcmp(name, "RAMP_NORMAL") == 0) {
        LINE_RAMP_NORMAL =
            (int16_t) LineFollowTask_LimitLong(value, 1L, 20L);
    }
    else if (strcmp(name, "RAMP_FAST") == 0) {
        LINE_RAMP_FAST =
            (int16_t) LineFollowTask_LimitLong(value, 1L, 30L);
    }

    /* ======================== 丢线处理 ======================== */

    else if (strcmp(name, "LOST_FORWARD_SPEED") == 0) {
        LINE_LOST_FORWARD_SPEED =
            (int16_t) LineFollowTask_LimitLong(value, 0L, 100L);
    }
    else if (strcmp(name, "LOST_TURN_SPEED") == 0) {
        LINE_LOST_TURN_SPEED =
            (int16_t) LineFollowTask_LimitLong(value, 0L, 100L);
    }
    else if (strcmp(name, "LOST_COAST_MS") == 0) {
        LINE_LOST_COAST_MS =
            (uint32_t) LineFollowTask_LimitLong(value, 0L, 2000L);
    }
    else if (strcmp(name, "LOST_STOP_MS") == 0) {
        LINE_LOST_STOP_MS =
            (uint32_t) LineFollowTask_LimitLong(value, 100L, 10000L);
    }
    else {
        return 0U;
    }

    LineFollowTask_NormalizeParams();
    return 1U;
}

/**
 * 读取指定参数的当前值，用于发送滑杆确认信息。
 */
static long LineFollowTask_GetSliderValue(const char *name)
{
    if (strcmp(name, "KP") == 0) {
        return (long) LINE_KP_X100;
    }

    if (strcmp(name, "KD") == 0) {
        return (long) LINE_KD_X100;
    }

    if (strcmp(name, "FN") == 0) {
        return (long) g_line_filter_alpha_normal;
    }

    if (strcmp(name, "FF") == 0) {
        return (long) g_line_filter_alpha_fast;
    }

    if (strcmp(name, "BS") == 0) {
        return (long) LINE_BASE_SPEED;
    }

    if (strcmp(name, "SS") == 0) {
        return (long) LINE_STRAIGHT_SPEED;
    }

    if (strcmp(name, "MS") == 0) {
        return (long) LINE_MIN_SPEED;
    }

    if (strcmp(name, "XS") == 0) {
        return (long) LINE_MAX_SPEED;
    }

    if (strcmp(name, "IS") == 0) {
        return (long) LINE_INTERSECTION_SPEED;
    }

    if (strcmp(name, "MC") == 0) {
        return (long) LINE_MAX_CORRECTION;
    }

    if (strcmp(name, "CS") == 0) {
        return (long) LINE_CURVE_SLOWDOWN_X100;
    }

    if (strcmp(name, "RN") == 0) {
        return (long) LINE_RAMP_NORMAL;
    }

    if (strcmp(name, "RF") == 0) {
        return (long) LINE_RAMP_FAST;
    }

    if (strcmp(name, "LF") == 0) {
        return (long) LINE_LOST_FORWARD_SPEED;
    }

    if (strcmp(name, "LT") == 0) {
        return (long) LINE_LOST_TURN_SPEED;
    }

    if (strcmp(name, "LC") == 0) {
        return (long) LINE_LOST_COAST_MS;
    }

    if (strcmp(name, "LS") == 0) {
        return (long) LINE_LOST_STOP_MS;
    }

    return 0L;
}

/* ============================= 命令解析 ============================= */

static void LineFollowTask_ProcessCommand(char *command)
{
    char *command_type;

    if (command == NULL) {
        return;
    }

    command_type = strtok(command, ",");

        /*
    * 兼容手机APP发送的方括号格式：
    * [slider,LINE_KP_X100,160]
    */
    if (command[0] == '[') {
        command++;
    }

    {
        size_t length = strlen(command);

        if ((length > 0U) &&
            (command[length - 1U] == ']')) {
            command[length - 1U] = '\0';
        }
    }

    if (command_type == NULL) {
        return;
    }

    /*
     * 滑杆格式：
     * slider,参数名,数值
     */
    if (strcmp(command_type, "slider") == 0) {
        char *slider_name = strtok(NULL, ",");
        char *value_text = strtok(NULL, ",");
        char *end_pointer;
        long value;

        if ((slider_name == NULL) || (value_text == NULL)) {
            (void) Bluetooth_WriteString(
                "ERR:slider_param\r\n");
            return;
        }

        value = strtol(value_text, &end_pointer, 10);

        if (end_pointer == value_text) {
            (void) Bluetooth_WriteString(
                "ERR:slider_value\r\n");
            return;
        }

        if (LineFollowTask_SetSlider(slider_name, value) == 0U) {
            (void) Bluetooth_WriteString(
                "ERR:slider_not_found\r\n");
            return;
        }

        /*
         * 重新读取实际值。
         * 若输入超过范围，这里返回的是限幅后的最终值。
         */
        LineFollowTask_SendSliderResult(
            slider_name,
            LineFollowTask_GetSliderValue(slider_name));

        return;
    }

    /*
     * 查询全部参数：
     * get,all
     */
    if (strcmp(command_type, "get") == 0) {
        char *target = strtok(NULL, ",");

        if ((target != NULL) &&
            (strcmp(target, "all") == 0)) {
            LineFollowTask_SendParams();
            (void) Bluetooth_WriteString(
                "OK:get_all\r\n");
        } else {
            (void) Bluetooth_WriteString(
                "ERR:get_param\r\n");
        }

        return;
    }

    (void) Bluetooth_WriteString(
        "ERR:use slider,name,value or get,all\r\n");
}

/* ============================= 蓝牙服务 ============================= */

static void LineFollowTask_ServiceBluetooth(void)
{
    uint8_t byte;

    while (Bluetooth_ReadByte(&byte)) {
        if (byte == '[') {
            /*
             * 收到帧头，清空旧内容并开始接收。
             */
            s_bt_frame_active = 1U;
            s_bt_rx_index = 0U;
        }
        else if ((byte == ']') &&
                 (s_bt_frame_active != 0U)) {
            /*
             * 收到帧尾，结束字符串并立即解析。
             */
            s_bt_rx_buffer[s_bt_rx_index] = '\0';

            LineFollowTask_ProcessCommand(
                s_bt_rx_buffer);

            s_bt_frame_active = 0U;
            s_bt_rx_index = 0U;
        }
        else if (s_bt_frame_active != 0U) {
            /*
             * 只保存方括号内部内容。
             */
            if (s_bt_rx_index <
                (LINE_FOLLOW_BT_RX_SIZE - 1U)) {
                s_bt_rx_buffer[s_bt_rx_index] =
                    (char)byte;
                s_bt_rx_index++;
            }
            else {
                s_bt_frame_active = 0U;
                s_bt_rx_index = 0U;

                (void)Bluetooth_WriteString(
                    "ERR:rx_overflow\r\n");
            }
        }
    }
}

/* ============================= 任务接口 ============================= */

void LineFollowTask_Init(void)
{
    LineFollow_Init();
    s_bt_frame_active = 0U;
    s_last_state = LINE_FOLLOW_STATE_STOPPED;

    s_bt_rx_index = 0U;
    memset(s_bt_rx_buffer, 0, sizeof(s_bt_rx_buffer));

    LineFollowTask_NormalizeParams();
}

void LineFollowTask_Start(uint32_t now_ms)
{
    /*
     * 清除进入任务前残留的不完整蓝牙帧。
     */
    s_bt_rx_index = 0U;
    memset(s_bt_rx_buffer, 0, sizeof(s_bt_rx_buffer));

    /*
     * 先撤销其他任务留下的电机命令，
     * 再由正式循迹模块重新开启速度闭环。
     */
    Motor_Off();

    LineFollow_Start(now_ms);

    s_last_state = LineFollow_GetState();

    TaskManager_SetStatus(LineFollow_GetStatusText());

    (void) Bluetooth_WriteString(
        "\r\nLINE FOLLOW SLIDER START\r\n");

    (void) Bluetooth_WriteString(
        "FORMAT:slider,NAME,VALUE\r\n");

    LineFollowTask_SendParams();
}

TaskManagerResult LineFollowTask_Update(uint32_t now_ms)
{
    LineFollowState state;

    /*
     * 先处理滑杆数据，修改后的参数将在本周期立即生效。
     */
    LineFollowTask_ServiceBluetooth();

    /*
     * 持续执行八路灰度循迹。
     */
    LineFollow_Update(now_ms);

    state = LineFollow_GetState();

    /*
     * 状态变化时刷新OLED。
     */
    if (state != s_last_state) {
        s_last_state = state;
        TaskManager_SetStatus(LineFollow_GetStatusText());
    }

    return TASK_MANAGER_RESULT_RUNNING;
}

void LineFollowTask_Cancel(void)
{
    LineFollow_Stop();

    s_last_state = LINE_FOLLOW_STATE_STOPPED;
    s_bt_rx_index = 0U;

    TaskManager_SetStatus("STOPPED");

    (void) Bluetooth_WriteString(
        "LINE FOLLOW SLIDER STOP\r\n");
}