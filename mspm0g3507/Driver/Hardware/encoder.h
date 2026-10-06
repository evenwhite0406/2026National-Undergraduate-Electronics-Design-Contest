/**
 * @file encoder.h
 * @brief 使用 GPIO 边沿中断的双路正交编码器计数模块。
 *
 * 硬件映射来自 SysConfig 的 GPIO_QEI：
 *   - SPEED_1 / SPEED_2 为 A 相，使用 GPIO 双边沿中断；
 *   - DIRECTION_1 / DIRECTION_2 为 B 相，使用普通 GPIO 输入。
 *
 * 典型用法：
 * @code
 * EncoderData left;
 *
 * SYSCFG_DL_init();
 * Encoder_Init(780U, 780U);       // 车轮每转的有效 x2 计数。
 * Encoder_SetReversed(ENCODER_2, true);
 *
 * while (1) {
 *     left = Encoder_GetData(ENCODER_1);
 *     // left.count：有符号位置计数
 *     // left.delta_count：最近 10 ms 内的计数变化
 *     // left.rpm_x10 / 10.0f：车轮转速（RPM）
 * }
 * @endcode
 *
 * 只有 A 相的两个边沿产生中断，因此属于 x2 解码。counts_per_revolution
 * 必须包含 x2 解码倍数和减速箱传动比。不需要 RPM 时可传 0，此时 count、
 * delta_count 和 counts_per_second 仍然有效。
 */

#ifndef ENCODER_H
#define ENCODER_H

#include <stdbool.h>
#include <stdint.h>

#define ENCODER_SAMPLE_PERIOD_MS    (10U) /* 速度/PID采样周期。 */
#define ENCODER_TIMER_TICK_MS       (2U)  /* 必须与SysConfig TIMER_0周期一致。 */

/* 实车测得的速度100对应计数率；更换电机/编码器后必须重新测量。 */
extern int32_t ENCODER_SPEED_100_PERCENT_CPS;

typedef enum {
    ENCODER_1 = 0,
    ENCODER_2,
    ENCODER_CHANNEL_COUNT
} EncoderId;

typedef struct {
    int32_t count;
    int32_t delta_count;
    int32_t counts_per_second;
    int32_t rpm_x10;
    uint32_t counts_per_revolution;
    uint8_t speed_percent;
} EncoderData;

/**
 * 复位两路通道，设置每转有效 x2 计数，清除待处理标志，并使能 GPIOA
 * 和 2 ms 控制定时器中断。轮速仍在内部五分频后每10 ms更新一次。
 */
void Encoder_Init(uint32_t encoder1_counts_per_revolution,
                  uint32_t encoder2_counts_per_revolution);

/** 不改变接线，反转指定通道的输出符号。 */
void Encoder_SetReversed(EncoderId id, bool reversed);

/** 修改 RPM 计算使用的每转有效 x2 计数。 */
void Encoder_SetCountsPerRevolution(EncoderId id,
                                    uint32_t counts_per_revolution);

/** 原子化返回指定通道最新的位置和速度数据。 */
EncoderData Encoder_GetData(EncoderId id);

/** 将有符号counts/s换算为0~100的速度绝对值并自动限幅。 */
uint8_t Encoder_CountsPerSecondToPercent(int32_t counts_per_second);

/** 原子化返回累计的有符号位置计数。 */
int32_t Encoder_GetCount(EncoderId id);

/** 清除指定通道的位置和速度状态。 */
void Encoder_Reset(EncoderId id);

/** 清除两路通道。 */
void Encoder_ResetAll(void);

/**
 * 将最新计数差换算为每秒计数和 RPM。本模块每 10 ms 自动调用一次；该接口
 * 保持公开，便于使用其他调度器的应用复用。period_ms 不得为零。
 */
void Encoder_UpdateSpeed(uint16_t period_ms);

/** GPIO 组中断处理主体，通常由 encoder.c 中的 GROUP1_IRQHandler 调用。 */
void Encoder_HandleGPIOInterrupt(void);

/** 2 ms定时器中断处理主体，内部每5次更新一次10 ms轮速。 */
void Encoder_HandleTimerInterrupt(void);

/**
 * 可选控制回调，在每次 10 ms 速度更新后由定时器中断调用。PID 模块可定义
 * 同签名的强符号函数覆盖默认空实现。回调必须简短，禁止在此刷新 OLED/UART。
 */
void Encoder_10msCallback(const EncoderData *encoder1,
                          const EncoderData *encoder2);

#endif
