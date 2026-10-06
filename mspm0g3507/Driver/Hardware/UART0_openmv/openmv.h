#ifndef UART0_OPENMV_H
#define UART0_OPENMV_H

#include <stdbool.h>
#include <stdint.h>

/* 调用：openmv_Init；主循环openmv_Service；ReadByte/Write读写。 */
#define OPENMV_LOWER_RX_BUFFER_SIZE  (256U)
#define OPENMV_LOWER_TX_BUFFER_SIZE  (256U)
#define OPENMV_LOWER_TX_BUDGET       (16U)

/** UART_openmv（UART0，PB1/PB0，9600 baud）模块。 */
void openmv_Init(void);
bool openmv_Write(const uint8_t *data, uint16_t length);
bool openmv_WriteString(const char *text);
bool openmv_ReadByte(uint8_t *data);
void openmv_Service(void);
void openmv_HandleInterrupt(void);
uint32_t openmv_GetRxDroppedCount(void);
uint32_t openmv_GetTxDroppedCount(void);

#endif
