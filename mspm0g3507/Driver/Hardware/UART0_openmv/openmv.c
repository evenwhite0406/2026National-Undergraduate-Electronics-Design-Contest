#include "openmv.h"
#include "ti_msp_dl_config.h"
#include <string.h>

static uint8_t g_rx_buffer[OPENMV_LOWER_RX_BUFFER_SIZE];
static volatile uint16_t g_rx_head;
static volatile uint16_t g_rx_tail;
static volatile uint32_t g_rx_dropped;
static uint8_t g_tx_buffer[OPENMV_LOWER_TX_BUFFER_SIZE];
static volatile uint16_t g_tx_head;
static volatile uint16_t g_tx_tail;
static volatile uint32_t g_tx_dropped;

static uint16_t openmv_NextIndex(uint16_t index, uint16_t size)
{
    index++;
    return (index >= size) ? 0U : index;
}

static uint16_t openmv_TxFree(void)
{
    uint16_t head = g_tx_head;
    uint16_t tail = g_tx_tail;
    uint16_t used = (head >= tail)
        ? (uint16_t) (head - tail)
        : (uint16_t) (OPENMV_LOWER_TX_BUFFER_SIZE - tail + head);

    return (uint16_t) (OPENMV_LOWER_TX_BUFFER_SIZE - 1U - used);
}

void openmv_Init(void)
{
    g_rx_head = 0U;
    g_rx_tail = 0U;
    g_rx_dropped = 0U;
    g_tx_head = 0U;
    g_tx_tail = 0U;
    g_tx_dropped = 0U;

    DL_UART_Main_enableInterrupt(UART_openmv_INST,
                                 DL_UART_MAIN_INTERRUPT_RX);
    NVIC_ClearPendingIRQ(UART_openmv_INST_INT_IRQN);
    NVIC_EnableIRQ(UART_openmv_INST_INT_IRQN);
}

bool openmv_Write(const uint8_t *data, uint16_t length)
{
    uint16_t i;

    if ((data == 0) || (length == 0U)) {
        return (length == 0U);
    }
    if (length > openmv_TxFree()) {
        g_tx_dropped += length;
        return false;
    }

    for (i = 0U; i < length; i++) {
        g_tx_buffer[g_tx_head] = data[i];
        g_tx_head = openmv_NextIndex(g_tx_head,
                                     OPENMV_LOWER_TX_BUFFER_SIZE);
    }
    return true;
}

bool openmv_WriteString(const char *text)
{
    size_t length;

    if (text == 0) {
        return false;
    }
    length = strlen(text);
    return (length <= 0xFFFFU)
        ? openmv_Write((const uint8_t *) text, (uint16_t) length)
        : false;
}

bool openmv_ReadByte(uint8_t *data)
{
    if ((data == 0) || (g_rx_tail == g_rx_head)) {
        return false;
    }
    *data = g_rx_buffer[g_rx_tail];
    g_rx_tail = openmv_NextIndex(g_rx_tail,
                                 OPENMV_LOWER_RX_BUFFER_SIZE);
    return true;
}

void openmv_Service(void)
{
    uint8_t budget = OPENMV_LOWER_TX_BUDGET;

    while ((g_tx_tail != g_tx_head) && (budget != 0U)) {
        if (!DL_UART_Main_transmitDataCheck(
                UART_openmv_INST, g_tx_buffer[g_tx_tail])) {
            break;
        }
        g_tx_tail = openmv_NextIndex(g_tx_tail,
                                     OPENMV_LOWER_TX_BUFFER_SIZE);
        budget--;
    }
}

void openmv_HandleInterrupt(void)
{
    if (DL_UART_Main_getPendingInterrupt(UART_openmv_INST) ==
        DL_UART_MAIN_IIDX_RX) {
        while (!DL_UART_Main_isRXFIFOEmpty(UART_openmv_INST)) {
            uint8_t data = DL_UART_Main_receiveData(UART_openmv_INST);
            uint16_t next = openmv_NextIndex(g_rx_head,
                                             OPENMV_LOWER_RX_BUFFER_SIZE);

            if (next == g_rx_tail) {
                g_rx_dropped++;
            } else {
                g_rx_buffer[g_rx_head] = data;
                g_rx_head = next;
            }
        }
    }
}

uint32_t openmv_GetRxDroppedCount(void)
{
    return g_rx_dropped;
}

uint32_t openmv_GetTxDroppedCount(void)
{
    return g_tx_dropped;
}
