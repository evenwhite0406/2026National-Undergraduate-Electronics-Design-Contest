#include "mmc5983ma.h"
#include "../PeripheralTest/bsp_i2c.h"
#include <limits.h>
#include <math.h>

#define MMC5983MA_I2C_ADDR             (0x30U)
#define MMC5983MA_PRODUCT_ID            (0x30U)

#define MMC5983MA_REG_XOUT0             (0x00U)
#define MMC5983MA_REG_STATUS            (0x08U)
#define MMC5983MA_REG_CONTROL0          (0x09U)
#define MMC5983MA_REG_CONTROL1          (0x0AU)
#define MMC5983MA_REG_CONTROL2          (0x0BU)
#define MMC5983MA_REG_PRODUCT_ID        (0x2FU)

#define MMC5983MA_STATUS_MEAS_DONE      (0x01U)
#define MMC5983MA_CONTROL0_SET          (0x08U)
#define MMC5983MA_CONTROL0_AUTO_SR      (0x20U)

/* BW=00：8 ms测量；100 Hz连续模式；每100次测量自动SET。 */
#define MMC5983MA_CONTROL1_BW_100HZ     (0x00U)
#define MMC5983MA_CONTROL2_100HZ        (0xBDU)

#define MMC5983MA_CENTER_18BIT          (131072L)
#define MMC5983MA_COUNTS_PER_GAUSS      (16384L)

/* 地磁强度有效范围，过小或过大时禁止修正yaw。 */

/*
 * 芯片轴到车体轴的默认映射。当前按X/Y/Z同向处理，必须实车确认。
 * 符号相反时只修改对应SIGN，不要在航向公式中临时取反。
 */

static MMC5983MA_Data g_mag;
static uint32_t g_last_service_ms;
static int32_t g_cal_min[3];
static int32_t g_cal_max[3];
static int32_t g_offset[3];
static int32_t g_scale_x1000[3] = {1000L, 1000L, 1000L};
static bool g_calibration_before_start;

static int MMC5983MA_WriteReg(uint8_t reg, uint8_t value)
{
    return (BSP_I2C_WriteReg(MMC5983MA_I2C_ADDR, reg, value) ==
            BSP_I2C_OK) ? MMC5983MA_OK : MMC5983MA_ERR_I2C;
}

static int MMC5983MA_ReadRegs(uint8_t reg, uint8_t *data, uint8_t length)
{
    return (BSP_I2C_ReadRegs(MMC5983MA_I2C_ADDR, reg, data, length) ==
            BSP_I2C_OK) ? MMC5983MA_OK : MMC5983MA_ERR_I2C;
}

static int16_t MMC5983MA_WrapHeadingX10(int32_t heading_x10)
{
    while (heading_x10 >= 3600L) heading_x10 -= 3600L;
    while (heading_x10 < 0L) heading_x10 += 3600L;
    return (int16_t) heading_x10;
}

static void MMC5983MA_UpdateCalibration(int32_t x, int32_t y, int32_t z)
{
    int32_t value[3] = {x, y, z};
    uint8_t i;

    if (!g_mag.calibration_active) return;
    for (i = 0U; i < 3U; i++) {
        if (value[i] < g_cal_min[i]) g_cal_min[i] = value[i];
        if (value[i] > g_cal_max[i]) g_cal_max[i] = value[i];
    }
}

static void MMC5983MA_CalculateHeading(int16_t roll_x10,
                                       int16_t pitch_x10)
{
    const float deg_to_rad = 0.01745329252f;
    float roll = ((float) roll_x10 / 10.0f) * deg_to_rad;
    float pitch = ((float) pitch_x10 / 10.0f) * deg_to_rad;
    float x = (float) g_mag.field_x_mg;
    float y = (float) g_mag.field_y_mg;
    float z = (float) g_mag.field_z_mg;
    float horizontal_x;
    float horizontal_y;
    float heading;
    float magnitude;

    magnitude = sqrtf((x * x) + (y * y) + (z * z));
    g_mag.field_strength_mg = (uint32_t) magnitude;
    horizontal_x = x * cosf(pitch) + z * sinf(pitch);
    horizontal_y = x * sinf(roll) * sinf(pitch) +
                   y * cosf(roll) - z * sinf(roll) * cosf(pitch);
    heading = atan2f(horizontal_y, horizontal_x) *
              57.2957795f * MMC5983MA_HEADING_SIGN;

    g_mag.heading_x10 = MMC5983MA_WrapHeadingX10(
        (int32_t) (heading * 10.0f) + MMC5983MA_DECLINATION_X10);
    g_mag.heading_valid = g_mag.calibrated &&
        (g_mag.field_strength_mg >= MMC5983MA_MIN_FIELD_MG) &&
        (g_mag.field_strength_mg <= MMC5983MA_MAX_FIELD_MG);
}

int MMC5983MA_Init(void)
{
    uint8_t product_id = 0U;

    g_mag.status = MMC5983MA_ERR_I2C;
    g_mag.product_id = 0U;
    g_mag.data_valid = false;
    g_mag.heading_valid = false;
    g_mag.calibrated = false;
    g_mag.calibration_active = false;
    g_last_service_ms = 0U;

    if (MMC5983MA_ReadRegs(MMC5983MA_REG_PRODUCT_ID,
                           &product_id, 1U) != MMC5983MA_OK) {
        return g_mag.status;
    }
    g_mag.product_id = product_id;
    if (product_id != MMC5983MA_PRODUCT_ID) {
        g_mag.status = MMC5983MA_ERR_ID;
        return g_mag.status;
    }

    if ((MMC5983MA_WriteReg(MMC5983MA_REG_CONTROL0,
                            MMC5983MA_CONTROL0_SET) != MMC5983MA_OK) ||
        (MMC5983MA_WriteReg(MMC5983MA_REG_CONTROL1,
                            MMC5983MA_CONTROL1_BW_100HZ) != MMC5983MA_OK) ||
        (MMC5983MA_WriteReg(MMC5983MA_REG_CONTROL0,
                            MMC5983MA_CONTROL0_AUTO_SR) != MMC5983MA_OK) ||
        (MMC5983MA_WriteReg(MMC5983MA_REG_CONTROL2,
                            MMC5983MA_CONTROL2_100HZ) != MMC5983MA_OK)) {
        g_mag.status = MMC5983MA_ERR_I2C;
        return g_mag.status;
    }

    g_mag.status = MMC5983MA_OK;
    return g_mag.status;
}

int MMC5983MA_Service(uint32_t now_ms,
                      int16_t roll_x10,
                      int16_t pitch_x10)
{
    uint8_t status;
    uint8_t raw[7];
    int32_t x;
    int32_t y;
    int32_t z;

    if (g_mag.status != MMC5983MA_OK) return g_mag.status;
    if ((now_ms - g_last_service_ms) < MMC5983MA_SERVICE_PERIOD_MS) {
        return MMC5983MA_ERR_NOT_READY;
    }
    g_last_service_ms = now_ms;

    if (MMC5983MA_ReadRegs(MMC5983MA_REG_STATUS,
                           &status, 1U) != MMC5983MA_OK) {
        g_mag.status = MMC5983MA_ERR_I2C;
        g_mag.data_valid = false;
        g_mag.heading_valid = false;
        return g_mag.status;
    }
    if ((status & MMC5983MA_STATUS_MEAS_DONE) == 0U) {
        return MMC5983MA_ERR_NOT_READY;
    }
    if (MMC5983MA_ReadRegs(MMC5983MA_REG_XOUT0,
                           raw, sizeof(raw)) != MMC5983MA_OK) {
        g_mag.status = MMC5983MA_ERR_I2C;
        g_mag.data_valid = false;
        g_mag.heading_valid = false;
        return g_mag.status;
    }

    x = ((int32_t) raw[0] << 10) |
        ((int32_t) raw[1] << 2) | ((raw[6] >> 6) & 0x03U);
    y = ((int32_t) raw[2] << 10) |
        ((int32_t) raw[3] << 2) | ((raw[6] >> 4) & 0x03U);
    z = ((int32_t) raw[4] << 10) |
        ((int32_t) raw[5] << 2) | ((raw[6] >> 2) & 0x03U);
    x = (x - MMC5983MA_CENTER_18BIT) * MMC5983MA_X_SIGN;
    y = (y - MMC5983MA_CENTER_18BIT) * MMC5983MA_Y_SIGN;
    z = (z - MMC5983MA_CENTER_18BIT) * MMC5983MA_Z_SIGN;
    g_mag.raw_x = x;
    g_mag.raw_y = y;
    g_mag.raw_z = z;
    MMC5983MA_UpdateCalibration(x, y, z);

    x = ((x - g_offset[0]) * g_scale_x1000[0]) / 1000L;
    y = ((y - g_offset[1]) * g_scale_x1000[1]) / 1000L;
    z = ((z - g_offset[2]) * g_scale_x1000[2]) / 1000L;
    g_mag.field_x_mg = (x * 1000L) / MMC5983MA_COUNTS_PER_GAUSS;
    g_mag.field_y_mg = (y * 1000L) / MMC5983MA_COUNTS_PER_GAUSS;
    g_mag.field_z_mg = (z * 1000L) / MMC5983MA_COUNTS_PER_GAUSS;
    g_mag.data_valid = true;
    MMC5983MA_CalculateHeading(roll_x10, pitch_x10);
    return MMC5983MA_OK;
}

void MMC5983MA_StartCalibration(void)
{
    uint8_t i;

    /* 新标定成功前保留旧结果，取消或失败时仍可继续使用。 */
    g_calibration_before_start = g_mag.calibrated;
    for (i = 0U; i < 3U; i++) {
        g_cal_min[i] = INT32_MAX;
        g_cal_max[i] = INT32_MIN;
    }
    g_mag.calibration_active = true;
    g_mag.calibrated = false;
    g_mag.heading_valid = false;
}

int MMC5983MA_FinishCalibration(void)
{
    int32_t half_span[3];
    int32_t new_offset[3];
    int32_t new_scale_x1000[3];
    int32_t average_span;
    uint8_t i;

    g_mag.calibration_active = false;
    for (i = 0U; i < 3U; i++) {
        if ((g_cal_max[i] <= g_cal_min[i]) ||
            ((g_cal_max[i] - g_cal_min[i]) < MMC5983MA_MIN_CAL_SPAN)) {
            g_mag.calibrated = g_calibration_before_start;
            g_mag.heading_valid = false;
            return MMC5983MA_ERR_CALIBRATION;
        }
        new_offset[i] = (g_cal_max[i] + g_cal_min[i]) / 2L;
        half_span[i] = (g_cal_max[i] - g_cal_min[i]) / 2L;
    }
    average_span = (half_span[0] + half_span[1] + half_span[2]) / 3L;
    for (i = 0U; i < 3U; i++) {
        new_scale_x1000[i] = (average_span * 1000L) / half_span[i];
    }
    for (i = 0U; i < 3U; i++) {
        g_offset[i] = new_offset[i];
        g_scale_x1000[i] = new_scale_x1000[i];
    }
    g_mag.calibrated = true;
    return MMC5983MA_OK;
}

void MMC5983MA_CancelCalibration(void)
{
    g_mag.calibration_active = false;
    g_mag.calibrated = g_calibration_before_start;
    g_mag.heading_valid = false;
}

const MMC5983MA_Data *MMC5983MA_GetData(void)
{
    return &g_mag;
}
