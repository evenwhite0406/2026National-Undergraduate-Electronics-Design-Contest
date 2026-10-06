#ifndef INTERRUPT_H
#define INTERRUPT_H

/*
 * 所有真正的中断入口统一定义在 interrupt.c：
 * SysTick=1ms时基；GROUP1=编码器GPIO；TIMER_0=2ms/10ms测速控制；
 * UART_Bluetooth=蓝牙缓冲；UART_OPENMV=UART3视觉缓冲。
 * 应用模块不要重复定义这些Handler，只调用对应驱动的Init/Service接口。
 */

#endif
