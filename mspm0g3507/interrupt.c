#include "interrupt.h"
#include "Driver/Hardware/encoder.h"
#include "Driver/Hardware/bluetooth.h"
#include "Driver/Hardware/UART3_OPENMV/OPENMV.h"
#include <stdint.h>
#include "ti_msp_dl_config.h"
extern volatile uint32_t g_ms;

/**
 * Cortex-M0+ 内核 SysTick 中断。
 * 来源：main中 SysTick_Config(CPUCLK_FREQ / 1000)，每1ms触发一次。
 * 含义：产生全工程毫秒时间基准；中断中只累加计数，不能执行阻塞操作。
 */
void SysTick_Handler(void)
{
    g_ms++;
}

/**
 * DriverLib GROUP1 聚合中断入口。
 * 来源：GPIO_QEI 的 SPEED_1/SPEED_2（编码器A相）双边沿GPIO中断。
 * 含义：读取编码器A/B相方向并对左右轮位置计数执行+1或-1。
 */
void GROUP1_IRQHandler(void)
{
    Encoder_HandleGPIOInterrupt();
}

/**
 * SysConfig TIMER_0（当前为TIMA1）周期中断。
 * 来源：2ms定时器零点事件；encoder.c内部5分频得到10ms控制周期。
 * 含义：计算左右轮counts/s，并运行位置、yaw和双轮速度PID控制。
 */
void TIMER_0_INST_IRQHandler(void)
{
    Encoder_HandleTimerInterrupt();
}

/**
 * SysConfig chao（当前为TIMG6）输入捕获中断。
 * 来源：PB6上的超声波Echo脉冲；Combined Capture记录高电平宽度和周期。
 * 含义：把Echo高电平时间换算为毫米距离，主循环通过Chao_GetDistanceMm读取。
 */
/**
 * SysConfig UART_Bluetooth 中断。
 * 来源：蓝牙串口RX数据到达、TX FIFO可写及UART错误事件。
 * 含义：把收发数据搬入环形缓冲区；协议解析和批量发送在主循环Service完成。
 */
void UART_Bluetooth_INST_IRQHandler(void)
{
    Bluetooth_HandleInterrupt();
}

/**
 * SysConfig UART_OPENMV（UART3）中断。
 * 来源：视觉串口RX数据到达、TX FIFO可写及UART错误事件。
 * 含义：只维护UART3收发环形缓冲区；视觉帧解析在主循环完成。
 */
void UART_OPENMV_INST_IRQHandler(void)
{
    OPENMV_HandleInterrupt();
}
