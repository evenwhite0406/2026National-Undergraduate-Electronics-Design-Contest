/**
 * @file bsp_i2c.c
 * @brief MSPM0G3507双I2C轮询驱动
 *
 * I2C_GYRO_INST：BMI088陀螺仪和加速度计
 * I2C_OLED_INST：OLED显示屏
 */

#include "bsp_i2c.h"
#include "ti_msp_dl_config.h"

/*
 * 必须在SysConfig里同时存在以下两个I2C实例。
 */
#ifndef I2C_GYRO_INST
#error "I2C_GYRO_INST is not defined in ti_msp_dl_config.h"
#endif

#ifndef I2C_OLED_INST
#error "I2C_OLED_INST is not defined in ti_msp_dl_config.h"
#endif


/* 与你现在32MHz系统时钟、400kHz设置保持一致 */

/* 控制器错误中断 */
#define BSP_I2C_ERROR_INTERRUPTS                                      \
    (DL_I2C_INTERRUPT_CONTROLLER_NACK |                               \
     DL_I2C_INTERRUPT_CONTROLLER_ARBITRATION_LOST)

/* 两路I2C分别记录初始化状态 */
static uint8_t g_gyro_i2c_initialized = 0U;
static uint8_t g_oled_i2c_initialized = 0U;


/**
 * @brief 初始化指定I2C控制器
 */
static void BSP_I2C_InitInstance(I2C_Regs *inst,
                                 uint8_t *initialized)
{
    static const DL_I2C_ClockConfig clock_config = {
        .clockSel    = DL_I2C_CLOCK_BUSCLK,
        .divideRatio = DL_I2C_CLOCK_DIVIDE_1,
    };

    if ((inst == 0) || (initialized == 0)) {
        return;
    }

    if (*initialized != 0U) {
        return;
    }

    DL_I2C_setClockConfig(
        inst,
        (DL_I2C_ClockConfig *) &clock_config);

    DL_I2C_setAnalogGlitchFilterPulseWidth(
        inst,
        DL_I2C_ANALOG_GLITCH_FILTER_WIDTH_50NS);

    DL_I2C_enableAnalogGlitchFilter(inst);

    /* 复位当前传输状态 */
    DL_I2C_resetControllerTransfer(inst);

    /*
     * 32MHz BUSCLK、分频1、周期7，
     * 对应约400kHz I2C。
     */
    DL_I2C_setTimerPeriod(
        inst,
        BSP_I2C_TIMER_PERIOD);

    DL_I2C_setControllerTXFIFOThreshold(
        inst,
        DL_I2C_TX_FIFO_LEVEL_EMPTY);

    DL_I2C_setControllerRXFIFOThreshold(
        inst,
        DL_I2C_RX_FIFO_LEVEL_BYTES_1);

    DL_I2C_enableControllerClockStretching(inst);

    /* 清除以前残留的状态 */
    DL_I2C_clearInterruptStatus(
        inst,
        DL_I2C_INTERRUPT_CONTROLLER_NACK |
        DL_I2C_INTERRUPT_CONTROLLER_ARBITRATION_LOST |
        DL_I2C_INTERRUPT_CONTROLLER_RX_DONE |
        DL_I2C_INTERRUPT_CONTROLLER_TX_DONE);

    DL_I2C_enableController(inst);

    *initialized = 1U;
}


/**
 * @brief 清除指定I2C控制器的传输状态
 */
static void BSP_I2C_RecoverInstance(I2C_Regs *inst)
{
    if (inst == 0) {
        return;
    }

    DL_I2C_resetControllerTransfer(inst);

    DL_I2C_clearInterruptStatus(
        inst,
        DL_I2C_INTERRUPT_CONTROLLER_NACK |
        DL_I2C_INTERRUPT_CONTROLLER_ARBITRATION_LOST |
        DL_I2C_INTERRUPT_CONTROLLER_RX_DONE |
        DL_I2C_INTERRUPT_CONTROLLER_TX_DONE);
}


/**
 * @brief 等待I2C控制器进入空闲状态
 */
static int BSP_I2C_WaitIdleInstance(I2C_Regs *inst)
{
    uint32_t timeout = BSP_I2C_TIMEOUT;
    uint32_t status;

    while (timeout > 0U) {
        status = DL_I2C_getControllerStatus(inst);

        if ((status & DL_I2C_CONTROLLER_STATUS_ERROR) != 0U) {
            BSP_I2C_RecoverInstance(inst);
            return BSP_I2C_ERR_BUS_ERROR;
        }

        if ((status & DL_I2C_CONTROLLER_STATUS_IDLE) != 0U) {
            return BSP_I2C_OK;
        }

        timeout--;
    }

    BSP_I2C_RecoverInstance(inst);
    return BSP_I2C_ERR_IDLE_TIMEOUT;
}


/**
 * @brief 等待TX_DONE或者RX_DONE
 */
static int BSP_I2C_WaitDoneInstance(I2C_Regs *inst,
                                    uint32_t done_interrupt,
                                    int timeout_error)
{
    uint32_t timeout = BSP_I2C_TIMEOUT;
    uint32_t interrupt_status;
    uint32_t controller_status;

    while (timeout > 0U) {
        interrupt_status =
            DL_I2C_getRawInterruptStatus(
                inst,
                done_interrupt | BSP_I2C_ERROR_INTERRUPTS);

        if ((interrupt_status &
             DL_I2C_INTERRUPT_CONTROLLER_NACK) != 0U) {

            BSP_I2C_RecoverInstance(inst);
            return BSP_I2C_ERR_NACK;
        }

        if ((interrupt_status &
             DL_I2C_INTERRUPT_CONTROLLER_ARBITRATION_LOST) != 0U) {

            BSP_I2C_RecoverInstance(inst);
            return BSP_I2C_ERR_BUS_ERROR;
        }

        if ((interrupt_status & done_interrupt) != 0U) {
            DL_I2C_clearInterruptStatus(
                inst,
                done_interrupt);

            return BSP_I2C_OK;
        }

        controller_status =
            DL_I2C_getControllerStatus(inst);

        if ((controller_status &
             DL_I2C_CONTROLLER_STATUS_ERROR) != 0U) {

            BSP_I2C_RecoverInstance(inst);
            return BSP_I2C_ERR_BUS_ERROR;
        }

        timeout--;
    }

    BSP_I2C_RecoverInstance(inst);
    return timeout_error;
}


/**
 * @brief 指定I2C实例写入若干字节
 */
static int BSP_I2C_WriteBytesInstance(I2C_Regs *inst,
                                      uint8_t *initialized,
                                      uint8_t dev_addr,
                                      const uint8_t *data,
                                      uint8_t len)
{
    int ret;

    if ((inst == 0) ||
        (initialized == 0) ||
        (data == 0) ||
        (len == 0U)) {
        return BSP_I2C_ERR_PARAM;
    }

    if (len > BSP_I2C_MAX_TX_LEN) {
        return BSP_I2C_ERR_TOO_LONG;
    }

    BSP_I2C_InitInstance(inst, initialized);

    ret = BSP_I2C_WaitIdleInstance(inst);
    if (ret != BSP_I2C_OK) {
        return ret;
    }

    /* 清除上一次传输残留标志 */
    DL_I2C_clearInterruptStatus(
        inst,
        DL_I2C_INTERRUPT_CONTROLLER_NACK |
        DL_I2C_INTERRUPT_CONTROLLER_ARBITRATION_LOST |
        DL_I2C_INTERRUPT_CONTROLLER_TX_DONE);

    /*
     * 先把短数据写入TX FIFO，再启动传输。
     * OLED每次只有控制字节+数据，共2字节。
     */
    DL_I2C_fillControllerTXFIFO(
        inst,
        (uint8_t *) data,
        len);

    DL_I2C_startControllerTransfer(
        inst,
        dev_addr,
        DL_I2C_CONTROLLER_DIRECTION_TX,
        len);

    ret = BSP_I2C_WaitDoneInstance(
        inst,
        DL_I2C_INTERRUPT_CONTROLLER_TX_DONE,
        BSP_I2C_ERR_TX_TIMEOUT);

    if (ret != BSP_I2C_OK) {
        return ret;
    }

    return BSP_I2C_WaitIdleInstance(inst);
}


/**
 * @brief 指定I2C实例读取连续寄存器
 */
static int BSP_I2C_ReadRegsInstance(I2C_Regs *inst,
                                    uint8_t *initialized,
                                    uint8_t dev_addr,
                                    uint8_t reg_addr,
                                    uint8_t *data,
                                    uint8_t len)
{
    uint8_t i;
    int ret;

    if ((inst == 0) ||
        (initialized == 0) ||
        (data == 0) ||
        (len == 0U)) {
        return BSP_I2C_ERR_PARAM;
    }

    BSP_I2C_InitInstance(inst, initialized);

    /*
     * 第一步：发送寄存器地址。
     * BMI088允许写寄存器地址和读取之间产生STOP。
     */
    ret = BSP_I2C_WriteBytesInstance(
        inst,
        initialized,
        dev_addr,
        &reg_addr,
        1U);

    if (ret != BSP_I2C_OK) {
        return ret;
    }

    ret = BSP_I2C_WaitIdleInstance(inst);
    if (ret != BSP_I2C_OK) {
        return ret;
    }

    DL_I2C_clearInterruptStatus(
        inst,
        DL_I2C_INTERRUPT_CONTROLLER_NACK |
        DL_I2C_INTERRUPT_CONTROLLER_ARBITRATION_LOST |
        DL_I2C_INTERRUPT_CONTROLLER_RX_DONE);

    /*
     * 第二步：启动接收。
     */
    DL_I2C_startControllerTransfer(
        inst,
        dev_addr,
        DL_I2C_CONTROLLER_DIRECTION_RX,
        len);

    for (i = 0U; i < len; i++) {
        uint32_t timeout = BSP_I2C_TIMEOUT;

        while (DL_I2C_isControllerRXFIFOEmpty(inst)) {
            uint32_t interrupt_status;
            uint32_t controller_status;

            interrupt_status =
                DL_I2C_getRawInterruptStatus(
                    inst,
                    BSP_I2C_ERROR_INTERRUPTS);

            if ((interrupt_status &
                 DL_I2C_INTERRUPT_CONTROLLER_NACK) != 0U) {

                BSP_I2C_RecoverInstance(inst);
                return BSP_I2C_ERR_NACK;
            }

            if ((interrupt_status &
                 DL_I2C_INTERRUPT_CONTROLLER_ARBITRATION_LOST) != 0U) {

                BSP_I2C_RecoverInstance(inst);
                return BSP_I2C_ERR_BUS_ERROR;
            }

            controller_status =
                DL_I2C_getControllerStatus(inst);

            if ((controller_status &
                 DL_I2C_CONTROLLER_STATUS_ERROR) != 0U) {

                BSP_I2C_RecoverInstance(inst);
                return BSP_I2C_ERR_BUS_ERROR;
            }

            if (timeout == 0U) {
                BSP_I2C_RecoverInstance(inst);
                return BSP_I2C_ERR_RX_TIMEOUT;
            }

            timeout--;
        }

        data[i] =
            DL_I2C_receiveControllerData(inst);
    }

    ret = BSP_I2C_WaitDoneInstance(
        inst,
        DL_I2C_INTERRUPT_CONTROLLER_RX_DONE,
        BSP_I2C_ERR_RX_TIMEOUT);

    if (ret != BSP_I2C_OK) {
        return ret;
    }

    return BSP_I2C_WaitIdleInstance(inst);
}


/* ============================================================
 * BMI088接口：固定使用I2C_GYRO_INST
 * ============================================================ */

int BSP_I2C_WriteBytes(uint8_t dev_addr,
                       const uint8_t *data,
                       uint8_t len)
{
    return BSP_I2C_WriteBytesInstance(
        I2C_GYRO_INST,
        &g_gyro_i2c_initialized,
        dev_addr,
        data,
        len);
}


int BSP_I2C_WriteReg(uint8_t dev_addr,
                     uint8_t reg_addr,
                     uint8_t data)
{
    uint8_t tx[2];

    tx[0] = reg_addr;
    tx[1] = data;

    return BSP_I2C_WriteBytesInstance(
        I2C_GYRO_INST,
        &g_gyro_i2c_initialized,
        dev_addr,
        tx,
        2U);
}


int BSP_I2C_ReadRegs(uint8_t dev_addr,
                     uint8_t reg_addr,
                     uint8_t *data,
                     uint8_t len)
{
    return BSP_I2C_ReadRegsInstance(
        I2C_GYRO_INST,
        &g_gyro_i2c_initialized,
        dev_addr,
        reg_addr,
        data,
        len);
}


/* ============================================================
 * OLED接口：固定使用I2C_OLED_INST
 * ============================================================ */

int BSP_I2C_OLED_WriteBytes(uint8_t dev_addr,
                            const uint8_t *data,
                            uint8_t len)
{
    return BSP_I2C_WriteBytesInstance(
        I2C_OLED_INST,
        &g_oled_i2c_initialized,
        dev_addr,
        data,
        len);
}


/* ============================================================
 * 恢复接口
 * ============================================================ */

void BSP_I2C_Recover(void)
{
    BSP_I2C_RecoverInstance(I2C_GYRO_INST);
}


void BSP_I2C_OLED_Recover(void)
{
    BSP_I2C_RecoverInstance(I2C_OLED_INST);
}
