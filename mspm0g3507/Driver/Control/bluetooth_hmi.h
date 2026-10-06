#ifndef BLUETOOTH_PARAM_H
#define BLUETOOTH_PARAM_H

#include <stdint.h>
#include <stdbool.h>

/* 调用：Param_Init，Param_RegisterSlider注册变量，主循环持续Param_Process/Param_ServiceTx。 */

/* ================= 硬件配置宏 ================= */
#define PARAM_RX_BUFFER_SIZE    128
#define PARAM_FRAME_HEAD        '['
#define PARAM_FRAME_TAIL        ']'
#define PARAM_MAX_PARAMS        4
#define PARAM_MAX_REG_ITEMS     48  /* 循迹及自适应弧线调参项。 */

/* ================= 调试宏 ================= */
#define PARAM_DEBUG_ENABLE      0

/* ================= 变量类型枚举（支持所有常用整型） ================= */
typedef enum {
    PARAM_TYPE_UINT8 = 0,   /* 对应 uint8_t */
    PARAM_TYPE_INT8,        /* 对应 int8_t */
    PARAM_TYPE_UINT16,      /* 对应 uint16_t */
    PARAM_TYPE_INT16,       /* 对应 int16_t */
    PARAM_TYPE_INT32,       /* 对应 int32_t */
    PARAM_TYPE_UINT32,      /* 对应 uint32_t */
} Param_Type_t;

/* ================= 调参项结构体 ================= */
typedef struct {
    char        slider_name[32];  /* 滑杆名称 */
    void        *var_ptr;         /* 通用指针，支持所有类型变量 */
    Param_Type_t var_type;        /* 变量类型 */
    int32_t     min_val;          /* 最小值 */
    int32_t     max_val;          /* 最大值 */
    bool        is_valid;         /* 该项是否有效 */
} Param_RegItem_t;

/* ================= 公开API ================= */
/**
 * @brief  初始化蓝牙调参模块
 */
void Param_Init(void);

/**
 * @brief  注册滑杆调参项（核心通用版）
 * @param  slider_name 滑杆名称 (和APP里一致)
 * @param  var_ptr     指向要控制的变量的指针 (直接&变量名即可)
 * @param  var_type    变量类型 (PARAM_TYPE_UINT8/INT8/...)
 * @param  min_val     变量最小值
 * @param  max_val     变量最大值
 * @retval true=注册成功, false=注册失败
 */
bool Param_RegisterSlider(const char *slider_name, void *var_ptr, Param_Type_t var_type, int32_t min_val, int32_t max_val);

/**
 * @brief  处理调参指令（主循环必须不断调用）
 */
void Param_Process(void);
void Param_InputByte(uint8_t data);

/** 从发送队列向 UART FIFO 搬运少量数据；主循环中持续调用。 */
void Param_ServiceTx(void);

/**
 * @brief  发送原始字符串
 */
void Param_SendRaw(const char *str);
/** 后台遥测；发送失败可丢弃，不能阻塞调参命令。 */
void Param_SendTelemetry(const char *str);

/**
 * @brief 以小程序绘图协议发送三路角度波形数据
 * @param roll_x10  横滚角，单位 0.1 度
 * @param pitch_x10 俯仰角，单位 0.1 度
 * @param yaw_x10   偏航角，单位 0.1 度
 */
void Param_SendAngles(int32_t roll_x10, int32_t pitch_x10, int32_t yaw_x10);

/** 发送速度环调试波形：目标、左轮实测、右轮实测，范围均为0~100。 */
void Param_SendSpeedWaveform(int32_t target_percent,
                             int32_t left_percent,
                             int32_t right_percent);

/**
 * @brief  发送格式化响应
 */
void Param_SendResponse(const char *status, const char *fmt, ...);

#endif /* BLUETOOTH_PARAM_H */
