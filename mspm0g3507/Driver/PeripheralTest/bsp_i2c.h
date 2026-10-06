#ifndef BSP_I2C_H
#define BSP_I2C_H

#include <stdint.h>

/* 调用：上层驱动使用ReadRegs/WriteReg；发生超时可调用对应Recover。 */
#define BSP_I2C_TIMEOUT       (100000UL) /* 总线超时循环次数。 */
#define BSP_I2C_MAX_TX_LEN    (8U)       /* 单次栈缓冲发送上限。 */
#define BSP_I2C_TIMER_PERIOD  (7U)       /* 32MHz下约400kHz。 */

/* 返回状态 */
#define BSP_I2C_OK                 (0)
#define BSP_I2C_ERR_PARAM          (-1)
#define BSP_I2C_ERR_TOO_LONG       (-2)
#define BSP_I2C_ERR_IDLE_TIMEOUT   (-3)
#define BSP_I2C_ERR_TX_TIMEOUT     (-4)
#define BSP_I2C_ERR_RX_TIMEOUT     (-5)
#define BSP_I2C_ERR_BUS_ERROR      (-6)
#define BSP_I2C_ERR_NACK           (-7)

/*
 * BMI088专用接口。
 * 这三个函数固定使用I2C_GYRO_INST。
 */
int BSP_I2C_WriteBytes(uint8_t dev_addr,
                       const uint8_t *data,
                       uint8_t len);

int BSP_I2C_WriteReg(uint8_t dev_addr,
                     uint8_t reg_addr,
                     uint8_t data);

int BSP_I2C_ReadRegs(uint8_t dev_addr,
                     uint8_t reg_addr,
                     uint8_t *data,
                     uint8_t len);

/*
 * OLED专用接口。
 * 固定使用I2C_OLED_INST。
 */
int BSP_I2C_OLED_WriteBytes(uint8_t dev_addr,
                            const uint8_t *data,
                            uint8_t len);

/* 总线恢复 */
void BSP_I2C_Recover(void);
void BSP_I2C_OLED_Recover(void);

#endif
