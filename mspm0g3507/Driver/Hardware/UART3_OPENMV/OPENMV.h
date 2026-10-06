#ifndef UART3_OPENMV_H
#define UART3_OPENMV_H

#include <stdbool.h>
#include <stdint.h>

/* 调用：OPENMV_Init；主循环OPENMV_Service；ReadByte/Write读写。 */
#define OPENMV_RX_BUFFER_SIZE  (256U)
#define OPENMV_TX_BUFFER_SIZE  (256U)
#define OPENMV_TX_BUDGET       (16U)

/** UART_OPENMV（UART3，PB3/PB2，9600 baud）模块。 */
/* K230 link: UART3, PB3 RX / PB2 TX, 115200 baud, 8-N-1. */
void OPENMV_Init(void);
bool OPENMV_Write(const uint8_t *data, uint16_t length);
bool OPENMV_WriteString(const char *text);
bool OPENMV_ReadByte(uint8_t *data);
void OPENMV_Service(void);
void OPENMV_HandleInterrupt(void);
uint32_t OPENMV_GetRxDroppedCount(void);
uint32_t OPENMV_GetTxDroppedCount(void);

#endif
