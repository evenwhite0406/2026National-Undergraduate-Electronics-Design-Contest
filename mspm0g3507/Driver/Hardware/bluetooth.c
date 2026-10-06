#include "bluetooth.h"
#include "ti_msp_dl_config.h"
#include <string.h>

static uint8_t g_rx_buffer[BLUETOOTH_RX_BUFFER_SIZE];
static volatile uint16_t g_rx_head;
static volatile uint16_t g_rx_tail;
static volatile uint32_t g_rx_dropped;
static uint8_t g_tx_buffer[BLUETOOTH_TX_BUFFER_SIZE];
static volatile uint16_t g_tx_head;
static volatile uint16_t g_tx_tail;
static volatile uint32_t g_tx_dropped;

static uint16_t Bluetooth_NextIndex(uint16_t index, uint16_t size)
{
    index++;
    return (index >= size) ? 0U : index;
}

static uint16_t Bluetooth_TxFree(void)
{
    uint16_t head = g_tx_head;
    uint16_t tail = g_tx_tail;
    uint16_t used = (head >= tail)
        ? (uint16_t) (head - tail)
        : (uint16_t) (BLUETOOTH_TX_BUFFER_SIZE - tail + head);

    return (uint16_t) (BLUETOOTH_TX_BUFFER_SIZE - 1U - used);
}

void Bluetooth_Init(void)
{

        /*
    * 等待蓝牙模块完成上电，避免其TXD启动波形干扰UART接收。
    */
    delay_cycles((CPUCLK_FREQ / 1000U) * 1000U);

    g_rx_head = 0U;
    g_rx_tail = 0U;
    g_rx_dropped = 0U;
    g_tx_head = 0U;
    g_tx_tail = 0U;
    g_tx_dropped = 0U;

    /*
 * 同时开启普通RX中断和RX超时中断。
 * 手机发送较短命令时，即使未达到FIFO阈值，
 * 也能通过RX超时中断把残留字节取出来。
 */
DL_UART_Main_enableInterrupt(
    UART_Bluetooth_INST,
    DL_UART_MAIN_INTERRUPT_RX |
    DL_UART_MAIN_INTERRUPT_RX_TIMEOUT_ERROR);
    /*
    * 清除蓝牙模块启动期间可能产生的乱码和残留数据。
    */
    while (!DL_UART_Main_isRXFIFOEmpty(UART_Bluetooth_INST)) {
        (void)DL_UART_Main_receiveData(UART_Bluetooth_INST);
    }
    NVIC_ClearPendingIRQ(UART_Bluetooth_INST_INT_IRQN);
    NVIC_EnableIRQ(UART_Bluetooth_INST_INT_IRQN);
}

bool Bluetooth_Write(const uint8_t *data, uint16_t length)
{
    uint16_t i;

    if ((data == 0) || (length == 0U)) {
        return (length == 0U);
    }
    if (length > Bluetooth_TxFree()) {
        g_tx_dropped += length;
        return false;
    }

    for (i = 0U; i < length; i++) {
        g_tx_buffer[g_tx_head] = data[i];
        g_tx_head = Bluetooth_NextIndex(g_tx_head,
                                        BLUETOOTH_TX_BUFFER_SIZE);
    }
    return true;
}

bool Bluetooth_WriteString(const char *text)
{
    size_t length;

    if (text == 0) {
        return false;
    }
    length = strlen(text);
    return (length <= 0xFFFFU)
        ? Bluetooth_Write((const uint8_t *) text, (uint16_t) length)
        : false;
}

bool Bluetooth_ReadByte(uint8_t *data)
{
    if ((data == 0) || (g_rx_tail == g_rx_head)) {
        return false;
    }
    *data = g_rx_buffer[g_rx_tail];
    g_rx_tail = Bluetooth_NextIndex(g_rx_tail,
                                    BLUETOOTH_RX_BUFFER_SIZE);
    return true;
}

void Bluetooth_Service(void)
{
    uint8_t budget = BLUETOOTH_TX_BUDGET;

    while ((g_tx_tail != g_tx_head) && (budget != 0U)) {
        if (!DL_UART_Main_transmitDataCheck(
                UART_Bluetooth_INST, g_tx_buffer[g_tx_tail])) {
            break;
        }
        g_tx_tail = Bluetooth_NextIndex(g_tx_tail,
                                        BLUETOOTH_TX_BUFFER_SIZE);
        budget--;
    }
}

void Bluetooth_HandleInterrupt(void)
{
    uint32_t interrupt_index;

    /*
     * 一次中断中可能存在多个待处理事件，
     * 持续读取，直到没有中断为止。
     */
    do {
        interrupt_index =
            DL_UART_Main_getPendingInterrupt(UART_Bluetooth_INST);

        /*
         * 普通RX和RX超时都需要清空接收FIFO。
         */
        if ((interrupt_index == DL_UART_MAIN_IIDX_RX) ||
            (interrupt_index ==
             DL_UART_MAIN_IIDX_RX_TIMEOUT_ERROR)) {

            while (!DL_UART_Main_isRXFIFOEmpty(
                       UART_Bluetooth_INST)) {
                uint8_t data;
                uint16_t next;

                data = DL_UART_Main_receiveData(
                    UART_Bluetooth_INST);

                next = Bluetooth_NextIndex(
                    g_rx_head,
                    BLUETOOTH_RX_BUFFER_SIZE);

                if (next == g_rx_tail) {
                    g_rx_dropped++;
                } else {
                    g_rx_buffer[g_rx_head] = data;
                    g_rx_head = next;
                }
            }
        }

    } while (interrupt_index !=
             DL_UART_MAIN_IIDX_NO_INTERRUPT);
}

uint32_t Bluetooth_GetRxDroppedCount(void)
{
    return g_rx_dropped;
}

uint32_t Bluetooth_GetTxDroppedCount(void)
{
    return g_tx_dropped;
}
