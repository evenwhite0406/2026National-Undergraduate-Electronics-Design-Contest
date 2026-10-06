/**
 * @file task_manager.c
 * @brief 四按键任务管理器实现
 */

#include "task_manager.h"
#include "../PeripheralTest/button.h"
#include "../Hardware/oled.h"

typedef enum {
    TASK_MANAGER_STATE_HOME = 0,
    TASK_MANAGER_STATE_RUNNING
} TaskManagerState;

static const TaskManagerTask *s_tasks;
static uint8_t s_task_count;
static uint8_t s_selected;
static TaskManagerState s_state;
static const char *s_status;

static uint8_t s_pressed_edges;

void TaskManager_ShowLine(uint8_t line, const char *text)
{
    char buffer[TASK_MANAGER_TEXT_COLUMNS + 1U];
    uint8_t i = 0U;

    while (i < TASK_MANAGER_TEXT_COLUMNS) {
        buffer[i] = ' ';
        i++;
    }
    buffer[TASK_MANAGER_TEXT_COLUMNS] = '\0';

    i = 0U;
    while ((text != 0) && (*text != '\0') &&
           (i < TASK_MANAGER_TEXT_COLUMNS)) {
        buffer[i] = *text;
        text++;
        i++;
    }

    if ((line >= 1U) && (line <= 8U)) {
        OLED_ShowString(line, 1U, buffer, 1U);
    }
}

static void TaskManager_DrawSelection(void)
{
    char number[12];

    number[0] = 'T';
    number[1] = 'A';
    number[2] = 'S';
    number[3] = 'K';
    number[4] = ' ';
    number[5] = (char) ('0' + ((s_selected + 1U) / 10U));
    number[6] = (char) ('0' + ((s_selected + 1U) % 10U));
    number[7] = '/';
    number[8] = (char) ('0' + (s_task_count / 10U));
    number[9] = (char) ('0' + (s_task_count % 10U));
    number[10] = '\0';

    TaskManager_ShowLine(1U, "VEHICLE CAL MENU");
    TaskManager_ShowLine(2U, s_tasks[s_selected].name);
    TaskManager_ShowLine(3U, number);
    TaskManager_ShowLine(5U, "K1 NEXT  K2 PREV");
    TaskManager_ShowLine(6U, "K3 RUN   K4 HOME");
    TaskManager_ShowLine(4U, "");
    TaskManager_ShowLine(7U, "");
    TaskManager_ShowLine(8U, "");
}

static void TaskManager_DrawRunning(void)
{
    TaskManager_ShowLine(1U, "TASK RUNNING");
    TaskManager_ShowLine(2U, s_tasks[s_selected].name);
    TaskManager_ShowLine(4U, s_status);
    TaskManager_ShowLine(7U, "K4 CANCEL/HOME");
    TaskManager_ShowLine(3U, "");
    TaskManager_ShowLine(5U, "");
    TaskManager_ShowLine(6U, "");
    TaskManager_ShowLine(8U, "");
}

static void TaskManager_ReturnHome(void)
{
    s_state = TASK_MANAGER_STATE_HOME;
    s_status = "";
    TaskManager_DrawSelection();
}

void TaskManager_Init(const TaskManagerTask *tasks, uint8_t task_count)
{
    s_tasks = tasks;
    s_task_count = task_count;
    s_selected = 0U;
    s_state = TASK_MANAGER_STATE_HOME;
    s_status = "";
    s_pressed_edges = 0U;
    if ((s_tasks != 0) && (s_task_count != 0U)) {
        TaskManager_DrawSelection();
    }
}

void TaskManager_Update(uint32_t now_ms)
{
    uint8_t edges;
    TaskManagerResult result;

    if ((s_tasks == 0) || (s_task_count == 0U)) {
        return;
    }

    PT_Button_Update(now_ms);
    edges = PT_Button_TakePressedEdges();
    s_pressed_edges = edges;

    /* 按键 4 优先级最高，运行过程中也能立即取消任务。 */
    if ((edges & BUTTON_4_MASK) != 0U) {
        if ((s_state == TASK_MANAGER_STATE_RUNNING) &&
            (s_tasks[s_selected].cancel != 0)) {
            s_tasks[s_selected].cancel();
        }
        TaskManager_ReturnHome();
        return;
    }

    if (s_state == TASK_MANAGER_STATE_HOME) {
        if ((edges & BUTTON_1_MASK) != 0U) {
            s_selected = (uint8_t) ((s_selected + 1U) % s_task_count);
            TaskManager_DrawSelection();
        } else if ((edges & BUTTON_2_MASK) != 0U) {
            if (s_selected == 0U) {
                s_selected = (uint8_t) (s_task_count - 1U);
            } else {
                s_selected--;
            }
            TaskManager_DrawSelection();
        } else if ((edges & BUTTON_3_MASK) != 0U) {
            s_state = TASK_MANAGER_STATE_RUNNING;
            s_status = "START";
            TaskManager_DrawRunning();
            if (s_tasks[s_selected].start != 0) {
                s_tasks[s_selected].start(now_ms);
            }
        }
        return;
    }

    if (s_tasks[s_selected].update == 0) {
        TaskManager_ReturnHome();
        return;
    }

    result = s_tasks[s_selected].update(now_ms);
    if (result != TASK_MANAGER_RESULT_RUNNING) {
        if (s_tasks[s_selected].cancel != 0) {
            s_tasks[s_selected].cancel();
        }
        TaskManager_ReturnHome();
    }
}

void TaskManager_SetStatus(const char *status)
{
    s_status = (status != 0) ? status : "";
    if (s_state == TASK_MANAGER_STATE_RUNNING) {
        TaskManager_ShowLine(4U, s_status);
    }
}

bool TaskManager_IsIdle(void)
{
    return (s_state == TASK_MANAGER_STATE_HOME);
}

uint8_t TaskManager_GetSelectedIndex(void)
{
    return s_selected;
}

uint8_t TaskManager_GetPressedEdges(void)
{
    return s_pressed_edges;
}
