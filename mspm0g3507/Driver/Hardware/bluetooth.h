#ifndef BLUETOOTH_H
#define BLUETOOTH_H

#include <stdbool.h>
#include <stdint.h>

/* 调用：Bluetooth_Init；主循环Service；用ReadByte/Write发送接收。中断入口由interrupt.c转发。 */
#define BLUETOOTH_RX_BUFFER_SIZE  (256U) /* 接收突发更大时增加，注意RAM占用。 */
#define BLUETOOTH_TX_BUFFER_SIZE  (256U)
#define BLUETOOTH_TX_BUDGET       (16U)  /* 每次Service最多启动发送字节数。 */

/** 初始化 UART_Bluetooth 及其收发队列。 */
void Bluetooth_Init(void);
bool Bluetooth_Write(const uint8_t *data, uint16_t length);
bool Bluetooth_WriteString(const char *text);
bool Bluetooth_ReadByte(uint8_t *data);
void Bluetooth_Service(void);
void Bluetooth_HandleInterrupt(void);
uint32_t Bluetooth_GetRxDroppedCount(void);
uint32_t Bluetooth_GetTxDroppedCount(void);

#endif
