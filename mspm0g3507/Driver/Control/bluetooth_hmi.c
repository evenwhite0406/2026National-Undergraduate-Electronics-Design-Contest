#include "bluetooth_hmi.h"
#include "../Hardware/bluetooth.h"
#include <string.h>
#include <stdio.h>
#include <stdarg.h>
#include <stdlib.h>

/* ================= 内部私有变量 ================= */
static uint8_t  s_rx_buffer[PARAM_RX_BUFFER_SIZE];
static volatile uint8_t s_rx_flag = 0;
static volatile uint8_t s_rx_state = 0;
static volatile uint8_t s_rx_index = 0;
static Param_RegItem_t s_reg_items[PARAM_MAX_REG_ITEMS];
static uint8_t s_reg_count = 0;

/* 接收中断只保存字节，协议状态机在主循环中运行。 */
static void Param_AcceptRxByte(uint8_t rx_data)
{
    if (s_rx_state == 0U) {
        if ((rx_data == PARAM_FRAME_HEAD) && (s_rx_flag == 0U)) {
            s_rx_state = 1U;
            s_rx_index = 0U;
        }
    } else if (rx_data == PARAM_FRAME_TAIL) {
        s_rx_buffer[s_rx_index] = '\0';
        s_rx_state = 0U;
        s_rx_flag = 1U;
    } else if (s_rx_index < (PARAM_RX_BUFFER_SIZE - 1U)) {
        s_rx_buffer[s_rx_index++] = rx_data;
    } else {
        s_rx_state = 0U;
        s_rx_index = 0U;
    }
}

/* ================= 内部工具函数：根据类型更新变量 ================= */
static void Param_UpdateVar(Param_RegItem_t *item, int32_t val)
{
    /* 先限制范围 */
    if (val < item->min_val) val = item->min_val;
    if (val > item->max_val) val = item->max_val;

    /* 根据变量类型，写入对应的值 */
    switch (item->var_type) {
        case PARAM_TYPE_UINT8:
            *(uint8_t*)item->var_ptr = (uint8_t)val;
            break;
        case PARAM_TYPE_INT8:
            *(int8_t*)item->var_ptr = (int8_t)val;
            break;
        case PARAM_TYPE_UINT16:
            *(uint16_t*)item->var_ptr = (uint16_t)val;
            break;
        case PARAM_TYPE_INT16:
            *(int16_t*)item->var_ptr = (int16_t)val;
            break;
        case PARAM_TYPE_INT32:
            *(int32_t*)item->var_ptr = (int32_t)val;
            break;
        case PARAM_TYPE_UINT32:
            *(uint32_t*)item->var_ptr = (uint32_t)val;
            break;
        default:
            break;
    }
}

static int32_t Param_ReadVar(const Param_RegItem_t *item)
{
    switch (item->var_type) {
        case PARAM_TYPE_UINT8:  return *(uint8_t*)item->var_ptr;
        case PARAM_TYPE_INT8:   return *(int8_t*)item->var_ptr;
        case PARAM_TYPE_UINT16: return *(uint16_t*)item->var_ptr;
        case PARAM_TYPE_INT16:  return *(int16_t*)item->var_ptr;
        case PARAM_TYPE_INT32:  return *(int32_t*)item->var_ptr;
        case PARAM_TYPE_UINT32: return (int32_t)*(uint32_t*)item->var_ptr;
        default:                return 0;
    }
}

/* ================= 公开API实现 ================= */
void Param_Init(void)
{
    memset(s_rx_buffer, 0, PARAM_RX_BUFFER_SIZE);
    memset(s_reg_items, 0, sizeof(s_reg_items));
    s_rx_flag = 0;
    s_rx_state = 0;
    s_rx_index = 0;
    s_reg_count = 0;
    #if PARAM_DEBUG_ENABLE
    printf("[Param] Module Initialized\r\n");
    #endif
}

bool Param_RegisterSlider(const char *slider_name, void *var_ptr, Param_Type_t var_type, int32_t min_val, int32_t max_val)
{
    if (s_reg_count >= PARAM_MAX_REG_ITEMS) {
        #if PARAM_DEBUG_ENABLE
        printf("[Param] Error: Registry Full\r\n");
        #endif
        return false;
    }
    
    strncpy(s_reg_items[s_reg_count].slider_name, slider_name, sizeof(s_reg_items[s_reg_count].slider_name) - 1);
    s_reg_items[s_reg_count].var_ptr = var_ptr;
    s_reg_items[s_reg_count].var_type = var_type;
    s_reg_items[s_reg_count].min_val = min_val;
    s_reg_items[s_reg_count].max_val = max_val;
    s_reg_items[s_reg_count].is_valid = true;
    
    #if PARAM_DEBUG_ENABLE
    printf("[Param] Registered: Slider=%s, Type=%d, Range=%d~%d\r\n", 
           slider_name, var_type, min_val, max_val);
    #endif
    
    s_reg_count++;
    return true;
}

void Param_Process(void)
{
    uint8_t rx_data;

    while ((s_rx_flag == 0U) && Bluetooth_ReadByte(&rx_data)) {
        Param_AcceptRxByte(rx_data);
    }

    if (s_rx_flag != 1) {
        return;
    }

    #if PARAM_DEBUG_ENABLE
    printf("[Param] Raw RX: %s\r\n", s_rx_buffer);
    #endif

    /* 本地复制，避免strtok静态变量冲突 */
    char local_buf[PARAM_RX_BUFFER_SIZE];
    strncpy(local_buf, (char*)s_rx_buffer, sizeof(local_buf) - 1);
    local_buf[sizeof(local_buf) - 1U] = '\0';

    /* 解析指令 */
    char *token = strtok(local_buf, ",");
    if (token == NULL) {
        goto EXIT;
    }

    /* ========== 处理滑杆调参指令: [slider,滑杆名,数值] ========== */
    if (strcmp(token, "slider") == 0) {
        char *slider_name = strtok(NULL, ",");
        char *value_str = strtok(NULL, ",");
        
        if (slider_name != NULL && value_str != NULL) {
            int32_t val = atoi(value_str);
            
            #if PARAM_DEBUG_ENABLE
            printf("[Param] Slider Cmd: Name=%s, RawVal=%d\r\n", slider_name, val);
            #endif

            /* 遍历注册表，找到对应的滑杆并更新变量 */
            bool found = false;
            for (uint8_t i = 0; i < s_reg_count; i++) {
                if (s_reg_items[i].is_valid && strcmp(s_reg_items[i].slider_name, slider_name) == 0) {
                    /* 更新变量 */
                    Param_UpdateVar(&s_reg_items[i], val);
                    found = true;
                    
                    #if PARAM_DEBUG_ENABLE
                    printf("[Param] OK: %s set to %d (Range: %d~%d)\r\n", 
                           slider_name, val, s_reg_items[i].min_val, s_reg_items[i].max_val);
                    #endif
                    
                    Param_SendResponse("OK", "slider,%s,%ld", slider_name,
                                       (long) Param_ReadVar(&s_reg_items[i]));
                    break;
                }
            }
            
            if (!found) {
                #if PARAM_DEBUG_ENABLE
                printf("[Param] Error: Slider '%s' not registered\r\n", slider_name);
                #endif
                Param_SendResponse("ERR", "slider_not_found");
            }
        } else {
            Param_SendResponse("ERR", "slider_param");
        }
    }
    /* ========== 处理按键指令: [key,按键名,动作] ========== */
    else if (strcmp(token, "key") == 0) {
        char *key_name = strtok(NULL, ",");
        char *key_action = strtok(NULL, ",");
        
        if (key_name != NULL && key_action != NULL) {
            #if PARAM_DEBUG_ENABLE
            printf("[Param] Key Cmd: Name=%s, Action=%s\r\n", key_name, key_action);
            #endif
            Param_SendResponse("OK", "key,%s,%s", key_name, key_action);
        } else {
            Param_SendResponse("ERR", "key_param");
        }
    }
    /* ========== 处理读取指令: [get,all] 或 [get,滑杆名] ========== */
    else if (strcmp(token, "get") == 0) {
        char *target = strtok(NULL, ",");
        
        if (target != NULL && strcmp(target, "all") == 0) {
            /* 打印所有已注册的变量 */
            for (uint8_t i = 0; i < s_reg_count; i++) {
                if (s_reg_items[i].is_valid) {
                    Param_SendResponse("OK", "%s=%ld",
                        s_reg_items[i].slider_name,
                        (long) Param_ReadVar(&s_reg_items[i]));
                }
            }
            Param_SendResponse("OK", "get_all");
        } else if (target != NULL) {
            /* 打印单个变量 */
            bool found = false;
            for (uint8_t i = 0; i < s_reg_count; i++) {
                if (s_reg_items[i].is_valid && strcmp(s_reg_items[i].slider_name, target) == 0) {
                    int32_t val = 0;
                    switch (s_reg_items[i].var_type) {
                        case PARAM_TYPE_UINT8: val = *(uint8_t*)s_reg_items[i].var_ptr; break;
                        case PARAM_TYPE_INT8: val = *(int8_t*)s_reg_items[i].var_ptr; break;
                        case PARAM_TYPE_UINT16: val = *(uint16_t*)s_reg_items[i].var_ptr; break;
                        case PARAM_TYPE_INT16: val = *(int16_t*)s_reg_items[i].var_ptr; break;
                        case PARAM_TYPE_INT32: val = *(int32_t*)s_reg_items[i].var_ptr; break;
                        case PARAM_TYPE_UINT32: val = *(uint32_t*)s_reg_items[i].var_ptr; break;
                        default: break;
                    }
                    Param_SendResponse("OK", "%s=%d", target, val);
                    found = true;
                    break;
                }
            }
            if (!found) {
                Param_SendResponse("ERR", "param_not_found");
            }
        }
    }

EXIT:
    s_rx_flag = 0;
    memset(s_rx_buffer, 0, PARAM_RX_BUFFER_SIZE);
}

void Param_InputByte(uint8_t data)
{
    Param_AcceptRxByte(data);
}

void Param_SendRaw(const char *str)
{
    (void) Bluetooth_WriteString(str);
}

void Param_SendTelemetry(const char *str)
{
    /* Non-blocking queue: dropping one plot frame is preferable to delaying
     * the next control command. */
    (void) Bluetooth_WriteString(str);
}

void Param_ServiceTx(void)
{
    Bluetooth_Service();
}

void Param_SendAngles(int32_t roll_x10, int32_t pitch_x10, int32_t yaw_x10)
{
    char buffer[48];

    /*
     * 当前配套小程序的控制标签使用小写完整单词，因此绘图帧使用
     * [plot,y1,y2,...]。三个值采用 0.1 度定点单位。
     */
    (void) snprintf(buffer, sizeof(buffer), "[plot,%ld,%ld,%ld]\r\n",
                    (long) roll_x10, (long) pitch_x10, (long) yaw_x10);
    Param_SendRaw(buffer);
}

void Param_SendSpeedWaveform(int32_t target_percent,
                             int32_t left_percent,
                             int32_t right_percent)
{
    char buffer[48];

    (void) snprintf(buffer, sizeof(buffer), "[plot,%ld,%ld,%ld]\r\n",
                    (long) target_percent,
                    (long) left_percent,
                    (long) right_percent);
    Param_SendRaw(buffer);
}

void Param_SendResponse(const char *status, const char *fmt, ...)
{
    char buffer[128];
    int len = 0;
    
    len = snprintf(buffer, sizeof(buffer), "%s:", status);
    
    va_list args;
    va_start(args, fmt);
    vsnprintf(buffer + len, sizeof(buffer) - len, fmt, args);
    va_end(args);
    
    strcat(buffer, "\r\n");
    Param_SendRaw(buffer);
}

/* ================= printf 重定向 ================= */
