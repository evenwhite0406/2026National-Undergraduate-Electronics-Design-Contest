#ifndef MMC5983MA_H
#define MMC5983MA_H

#include <stdbool.h>
#include <stdint.h>

/*
 * 调用方法：MMC5983MA_Init(); 主循环调用 MMC5983MA_Service(now, roll, pitch);
 * 首次使用先 StartCalibration，充分转动车体后 FinishCalibration。
 */

/* 实车安装方向、当地磁偏角和有效磁场范围参数。 */
extern uint32_t MMC5983MA_SERVICE_PERIOD_MS;
extern int32_t MMC5983MA_MIN_CAL_SPAN;
extern uint16_t MMC5983MA_MIN_FIELD_MG;
extern uint16_t MMC5983MA_MAX_FIELD_MG;
extern int32_t MMC5983MA_X_SIGN;
extern int32_t MMC5983MA_Y_SIGN;
extern int32_t MMC5983MA_Z_SIGN;
extern float MMC5983MA_HEADING_SIGN;
extern int16_t MMC5983MA_DECLINATION_X10;

#define MMC5983MA_OK                 (0)
#define MMC5983MA_ERR_I2C            (-40)
#define MMC5983MA_ERR_ID             (-41)
#define MMC5983MA_ERR_NOT_READY      (-42)
#define MMC5983MA_ERR_CALIBRATION    (-43)

typedef struct {
    int status;
    uint8_t product_id;
    int32_t raw_x;
    int32_t raw_y;
    int32_t raw_z;
    int32_t field_x_mg;
    int32_t field_y_mg;
    int32_t field_z_mg;
    uint32_t field_strength_mg;
    int16_t heading_x10;
    bool data_valid;
    bool heading_valid;
    bool calibrated;
    bool calibration_active;
} MMC5983MA_Data;

/** 检测芯片并启动100 Hz连续测量和自动SET/RESET。 */
int MMC5983MA_Init(void);

/**
 * 非阻塞周期服务；内部最多每20 ms读取一次。
 * roll/pitch单位为0.1度，用于倾斜补偿航向。
 */
int MMC5983MA_Service(uint32_t now_ms,
                      int16_t roll_x10,
                      int16_t pitch_x10);

/** 开始采集三轴最大最小值；标定时需让模块绕三个轴充分转动。 */
void MMC5983MA_StartCalibration(void);

/** 完成硬铁偏置和三轴比例标定。 */
int MMC5983MA_FinishCalibration(void);

void MMC5983MA_CancelCalibration(void);

const MMC5983MA_Data *MMC5983MA_GetData(void);

#endif
