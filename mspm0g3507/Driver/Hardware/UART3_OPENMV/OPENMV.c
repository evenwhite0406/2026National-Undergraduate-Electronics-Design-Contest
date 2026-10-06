#include "OPENMV.h"
#include "ti_msp_dl_config.h"
#include <string.h>

static uint8_t g_rx_buffer[OPENMV_RX_BUFFER_SIZE];
static volatile uint16_t g_rx_head;
static volatile uint16_t g_rx_tail;
static volatile uint32_t g_rx_dropped;
static uint8_t g_tx_buffer[OPENMV_TX_BUFFER_SIZE];
static volatile uint16_t g_tx_head;
static volatile uint16_t g_tx_tail;
static volatile uint32_t g_tx_dropped;

static uint16_t OPENMV_NextIndex(uint16_t index, uint16_t size)
{
    index++;
    return (index >= size) ? 0U : index;
}

static uint16_t OPENMV_TxFree(void)
{
    uint16_t head = g_tx_head;
    uint16_t tail = g_tx_tail;
    uint16_t used = (head >= tail)
        ? (uint16_t) (head - tail)
        : (uint16_t) (OPENMV_TX_BUFFER_SIZE - tail + head);

    return (uint16_t) (OPENMV_TX_BUFFER_SIZE - 1U - used);
}

static void OPENMV_DrainRxFifo(void)
{
    while (!DL_UART_Main_isRXFIFOEmpty(UART_OPENMV_INST)) {
        uint8_t data = DL_UART_Main_receiveData(UART_OPENMV_INST);
        uint16_t next = OPENMV_NextIndex(g_rx_head,
                                         OPENMV_RX_BUFFER_SIZE);

        if (next == g_rx_tail) {
            g_rx_dropped++;
        } else {
            g_rx_buffer[g_rx_head] = data;
            g_rx_head = next;
        }
    }
}

void OPENMV_Init(void)
{
    g_rx_head = 0U;
    g_rx_tail = 0U;
    g_rx_dropped = 0U;
    g_tx_head = 0U;
    g_tx_tail = 0U;
    g_tx_dropped = 0U;

    while (!DL_UART_Main_isRXFIFOEmpty(UART_OPENMV_INST)) {
        (void) DL_UART_Main_receiveData(UART_OPENMV_INST);
    }

    DL_UART_Main_enableInterrupt(
        UART_OPENMV_INST,
        DL_UART_MAIN_INTERRUPT_RX |
        DL_UART_MAIN_INTERRUPT_RX_TIMEOUT_ERROR);
    NVIC_ClearPendingIRQ(UART_OPENMV_INST_INT_IRQN);
    NVIC_EnableIRQ(UART_OPENMV_INST_INT_IRQN);
}

bool OPENMV_Write(const uint8_t *data, uint16_t length)
{
    uint16_t i;

    if ((data == 0) || (length == 0U)) {
        return (length == 0U);
    }
    if (length > OPENMV_TxFree()) {
        g_tx_dropped += length;
        return false;
    }

    for (i = 0U; i < length; i++) {
        g_tx_buffer[g_tx_head] = data[i];
        g_tx_head = OPENMV_NextIndex(g_tx_head, OPENMV_TX_BUFFER_SIZE);
    }
    return true;
}

bool OPENMV_WriteString(const char *text)
{
    size_t length;

    if (text == 0) {
        return false;
    }
    length = strlen(text);
    return (length <= 0xFFFFU)
        ? OPENMV_Write((const uint8_t *) text, (uint16_t) length)
        : false;
}

bool OPENMV_ReadByte(uint8_t *data)
{
    if ((data == 0) || (g_rx_tail == g_rx_head)) {
        return false;
    }
    *data = g_rx_buffer[g_rx_tail];
    g_rx_tail = OPENMV_NextIndex(g_rx_tail, OPENMV_RX_BUFFER_SIZE);
    return true;
}

void OPENMV_Service(void)
{
    uint8_t budget = OPENMV_TX_BUDGET;
    uint32_t primask = __get_PRIMASK();

    /* Polling fallback: keeps RX alive even if one UART RX IRQ is missed. */
    __disable_irq();
    OPENMV_DrainRxFifo();
    if (primask == 0U) {
        __enable_irq();
    }

    while ((g_tx_tail != g_tx_head) && (budget != 0U)) {
        if (!DL_UART_Main_transmitDataCheck(
                UART_OPENMV_INST, g_tx_buffer[g_tx_tail])) {
            break;
        }
        g_tx_tail = OPENMV_NextIndex(g_tx_tail, OPENMV_TX_BUFFER_SIZE);
        budget--;
    }
}

void OPENMV_HandleInterrupt(void)
{
    uint32_t interrupt_index;

    do {
        interrupt_index =
            DL_UART_Main_getPendingInterrupt(UART_OPENMV_INST);

        if ((interrupt_index == DL_UART_MAIN_IIDX_RX) ||
            (interrupt_index ==
             DL_UART_MAIN_IIDX_RX_TIMEOUT_ERROR)) {
            OPENMV_DrainRxFifo();
        }
    } while (interrupt_index != DL_UART_MAIN_IIDX_NO_INTERRUPT);
}

uint32_t OPENMV_GetRxDroppedCount(void)
{
    return g_rx_dropped;
}

uint32_t OPENMV_GetTxDroppedCount(void)
{
    return g_tx_dropped;
}
