/**
 * @file encoder.c
 * @brief GPIO x2 正交解码与 10 ms 周期轮速计算。
 *
 * GPIO 中断只执行四项短操作：读取待处理标志、清除标志、采样 A/B 相并
 * 累加 +1 或 -1。定时器中断对计数器取快照、计算速度，然后调用
 * Encoder_10msCallback()，从而可以在不把控制逻辑放入 GPIO 中断的情况下
 * 接入 PID。
 */

#include "encoder.h"
#include "motor.h"
#include "ti_msp_dl_config.h"

#if ((ENCODER_SAMPLE_PERIOD_MS % ENCODER_TIMER_TICK_MS) != 0U)
#error "ENCODER_SAMPLE_PERIOD_MS must be a multiple of ENCODER_TIMER_TICK_MS"
#endif

static volatile int32_t g_count[ENCODER_CHANNEL_COUNT];
static volatile int32_t g_previous_count[ENCODER_CHANNEL_COUNT];
static volatile int32_t g_delta_count[ENCODER_CHANNEL_COUNT];
static volatile int32_t g_counts_per_second[ENCODER_CHANNEL_COUNT];
static volatile int32_t g_rpm_x10[ENCODER_CHANNEL_COUNT];
static volatile uint32_t g_counts_per_revolution[ENCODER_CHANNEL_COUNT];
static volatile uint8_t g_reversed[ENCODER_CHANNEL_COUNT];

static uint32_t Encoder_EnterCritical(void)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    return primask;
}

static void Encoder_ExitCritical(uint32_t primask)
{
    if (primask == 0U) {
        __enable_irq();
    }
}

static bool Encoder_IsValidId(EncoderId id)
{
    return ((uint32_t) id < (uint32_t) ENCODER_CHANNEL_COUNT);
}

uint8_t Encoder_CountsPerSecondToPercent(int32_t counts_per_second)
{
    uint32_t magnitude;
    uint32_t percent;

    if (counts_per_second < 0L) {
        /* 先转为64位，避免INT32_MIN取反溢出。 */
        magnitude = (uint32_t) (-(int64_t) counts_per_second);
    } else {
        magnitude = (uint32_t) counts_per_second;
    }

    percent = (uint32_t) (((uint64_t) magnitude * 100ULL) /
                          (uint32_t) ENCODER_SPEED_100_PERCENT_CPS);
    if (percent > 100U) {
        percent = 100U;
    }
    return (uint8_t) percent;
}

/*
 * x2 解码在 A 相每个边沿通过 A == B 判断方向。交换任意一相都会反转
 * 计数符号，可使用 Encoder_SetReversed() 修正。
 */
static int32_t Encoder_DecodeStep(EncoderId id, GPIO_Regs *phase_a_port,
                                  uint32_t phase_a_pin,
                                  GPIO_Regs *phase_b_port,
                                  uint32_t phase_b_pin)
{
    bool phase_a = (DL_GPIO_readPins(phase_a_port, phase_a_pin) != 0U);
    bool phase_b = (DL_GPIO_readPins(phase_b_port, phase_b_pin) != 0U);
    int32_t step = (phase_a == phase_b) ? 1L : -1L;

    if (g_reversed[id] != 0U) {
        step = -step;
    }

    return step;
}

static void Encoder_CopyData(EncoderId id, EncoderData *data)
{
    data->count = g_count[id];
    data->delta_count = g_delta_count[id];
    data->counts_per_second = g_counts_per_second[id];
    data->rpm_x10 = g_rpm_x10[id];
    data->counts_per_revolution = g_counts_per_revolution[id];
    data->speed_percent = Encoder_CountsPerSecondToPercent(
        g_counts_per_second[id]);
}

void Encoder_Init(uint32_t encoder1_counts_per_revolution,
                  uint32_t encoder2_counts_per_revolution)
{
    uint32_t primask = Encoder_EnterCritical();

    g_counts_per_revolution[ENCODER_1] =
        encoder1_counts_per_revolution;
    g_counts_per_revolution[ENCODER_2] =
        encoder2_counts_per_revolution;
    g_reversed[ENCODER_1] = 0U;
    g_reversed[ENCODER_2] = 0U;

    g_count[ENCODER_1] = 0L;
    g_count[ENCODER_2] = 0L;
    g_previous_count[ENCODER_1] = 0L;
    g_previous_count[ENCODER_2] = 0L;
    g_delta_count[ENCODER_1] = 0L;
    g_delta_count[ENCODER_2] = 0L;
    g_counts_per_second[ENCODER_1] = 0L;
    g_counts_per_second[ENCODER_2] = 0L;
    g_rpm_x10[ENCODER_1] = 0L;
    g_rpm_x10[ENCODER_2] = 0L;

    Encoder_ExitCritical(primask);

    DL_GPIO_clearInterruptStatus(GPIO_QEI_SPEED_1_PORT,
                                 GPIO_QEI_SPEED_1_PIN |
                                 GPIO_QEI_SPEED_2_PIN);
    NVIC_ClearPendingIRQ(GPIO_QEI_INT_IRQN);
    NVIC_EnableIRQ(GPIO_QEI_INT_IRQN);

    DL_TimerA_clearInterruptStatus(TIMER_0_INST,
                                   DL_TIMERA_INTERRUPT_ZERO_EVENT);
    NVIC_ClearPendingIRQ(TIMER_0_INST_INT_IRQN);
    NVIC_EnableIRQ(TIMER_0_INST_INT_IRQN);
    DL_TimerA_startCounter(TIMER_0_INST);
}

void Encoder_SetReversed(EncoderId id, bool reversed)
{
    if (Encoder_IsValidId(id)) {
        g_reversed[id] = reversed ? 1U : 0U;
    }
}

void Encoder_SetCountsPerRevolution(EncoderId id,
                                    uint32_t counts_per_revolution)
{
    if (Encoder_IsValidId(id)) {
        g_counts_per_revolution[id] = counts_per_revolution;
    }
}

EncoderData Encoder_GetData(EncoderId id)
{
    EncoderData data = {0L, 0L, 0L, 0L, 0U, 0U};
    uint32_t primask;

    if (!Encoder_IsValidId(id)) {
        return data;
    }

    primask = Encoder_EnterCritical();
    Encoder_CopyData(id, &data);
    Encoder_ExitCritical(primask);
    return data;
}

int32_t Encoder_GetCount(EncoderId id)
{
    int32_t count = 0L;

    if (Encoder_IsValidId(id)) {
        count = g_count[id];
    }

    return count;
}

void Encoder_Reset(EncoderId id)
{
    uint32_t primask;

    if (!Encoder_IsValidId(id)) {
        return;
    }

    primask = Encoder_EnterCritical();
    g_count[id] = 0L;
    g_previous_count[id] = 0L;
    g_delta_count[id] = 0L;
    g_counts_per_second[id] = 0L;
    g_rpm_x10[id] = 0L;
    Encoder_ExitCritical(primask);
}

void Encoder_ResetAll(void)
{
    Encoder_Reset(ENCODER_1);
    Encoder_Reset(ENCODER_2);
}

void Encoder_UpdateSpeed(uint16_t period_ms)
{
    uint32_t primask;
    uint32_t i;

    if (period_ms == 0U) {
        return;
    }

    primask = Encoder_EnterCritical();
    for (i = 0U; i < (uint32_t) ENCODER_CHANNEL_COUNT; i++) {
        int32_t current = g_count[i];
        int32_t delta = (int32_t) ((uint32_t) current -
                                  (uint32_t) g_previous_count[i]);
        int64_t scaled;

        g_previous_count[i] = current;
        g_delta_count[i] = delta;
        scaled = (int64_t) delta * 1000LL;
        g_counts_per_second[i] = (int32_t) (scaled / period_ms);

        if (g_counts_per_revolution[i] != 0U) {
            scaled = (int64_t) delta * 600000LL;
            g_rpm_x10[i] = (int32_t)
                (scaled / ((int64_t) g_counts_per_revolution[i] *
                           period_ms));
        } else {
            g_rpm_x10[i] = 0L;
        }
    }
    Encoder_ExitCritical(primask);
}

void Encoder_HandleGPIOInterrupt(void)
{
    uint32_t pending = DL_GPIO_getEnabledInterruptStatus(
        GPIO_QEI_SPEED_1_PORT,
        GPIO_QEI_SPEED_1_PIN | GPIO_QEI_SPEED_2_PIN);

    DL_GPIO_clearInterruptStatus(GPIO_QEI_SPEED_1_PORT, pending);

    if ((pending & GPIO_QEI_SPEED_1_PIN) != 0U) {
        g_count[ENCODER_1] += Encoder_DecodeStep(
            ENCODER_1,
            GPIO_QEI_SPEED_1_PORT, GPIO_QEI_SPEED_1_PIN,
            GPIO_QEI_DIRECTION_1_PORT, GPIO_QEI_DIRECTION_1_PIN);
    }

    if ((pending & GPIO_QEI_SPEED_2_PIN) != 0U) {
        g_count[ENCODER_2] += Encoder_DecodeStep(
            ENCODER_2,
            GPIO_QEI_SPEED_2_PORT, GPIO_QEI_SPEED_2_PIN,
            GPIO_QEI_DIRECTION_2_PORT, GPIO_QEI_DIRECTION_2_PIN);
    }
}

void Encoder_HandleTimerInterrupt(void)
{
    static uint8_t speed_divider;

    if (DL_TimerA_getPendingInterrupt(TIMER_0_INST) ==
        DL_TIMER_IIDX_ZERO) {
        EncoderData encoder1;
        EncoderData encoder2;

        speed_divider++;
        if (speed_divider <
            (ENCODER_SAMPLE_PERIOD_MS / ENCODER_TIMER_TICK_MS)) {
            return;
        }
        speed_divider = 0U;
        Encoder_UpdateSpeed(ENCODER_SAMPLE_PERIOD_MS);
        Encoder_CopyData(ENCODER_1, &encoder1);
        Encoder_CopyData(ENCODER_2, &encoder2);
        Motor_SpeedControlUpdate(encoder1.count,
                                 encoder2.count,
                                 encoder1.counts_per_second,
                                 encoder2.counts_per_second);
        Encoder_10msCallback(&encoder1, &encoder2);
    }
}

/* 加入闭环控制后，可在 PID 模块中覆盖这个弱定义回调。 */
__attribute__((weak))
void Encoder_10msCallback(const EncoderData *encoder1,
                          const EncoderData *encoder2)
{
    (void) encoder1;
    (void) encoder2;
}
