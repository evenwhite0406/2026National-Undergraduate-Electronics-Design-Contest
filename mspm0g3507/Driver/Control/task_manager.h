/**
 * @file task_manager.h
 * @brief 四按键任务选择与执行状态机
 *
 * 按键 1：下一个任务；按键 2：上一个任务；
 * 按键 3：执行当前任务；按键 4：取消并回到初始状态。
 */

#ifndef TASK_MANAGER_H
#define TASK_MANAGER_H

#include <stdbool.h>
#include <stdint.h>

/* 调用：TaskManager_Init(task_table, count)，主循环持续TaskManager_Update按键状态。 */
#define TASK_MANAGER_TEXT_COLUMNS   (21U) /* 128x64 OLED小字体列数。 */

typedef enum {
    TASK_MANAGER_RESULT_RUNNING = 0,
    TASK_MANAGER_RESULT_COMPLETE,
    TASK_MANAGER_RESULT_FAILED
} TaskManagerResult;

typedef void (*TaskManagerStartFn)(uint32_t now_ms);
typedef TaskManagerResult (*TaskManagerUpdateFn)(uint32_t now_ms);
typedef void (*TaskManagerCancelFn)(void);

typedef struct {
    const char *name;
    TaskManagerStartFn start;
    TaskManagerUpdateFn update;
    TaskManagerCancelFn cancel;
} TaskManagerTask;

/** 初始化任务表并进入初始状态。任务表必须在程序运行期间一直有效。 */
void TaskManager_Init(const TaskManagerTask *tasks, uint8_t task_count);

/** 更新按键模块并推进任务状态机；主循环中持续调用。 */
void TaskManager_Update(uint32_t now_ms);

/** 设置任务执行页面的状态文字，参数必须指向长期有效的字符串。 */
void TaskManager_SetStatus(const char *status);

/** 返回当前是否处于任务主页（未运行任务）。 */
bool TaskManager_IsIdle(void);

/** Return the current zero-based menu selection index. */
uint8_t TaskManager_GetSelectedIndex(void);

/** 返回本次TaskManager_Update识别到的消抖按下沿，供运行中的任务读取。 */
uint8_t TaskManager_GetPressedEdges(void);

/**
 * 在 OLED 指定文本行显示一行内容，并自动用空格清除行尾。
 * 任务可用该接口绘制多行实时测量数据；line 范围为 1~8。
 */
void TaskManager_ShowLine(uint8_t line, const char *text);

#endif
