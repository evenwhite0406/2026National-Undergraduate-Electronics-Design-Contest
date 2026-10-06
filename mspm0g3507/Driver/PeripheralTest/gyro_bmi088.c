/**
 * @file gyro_bmi088.c
 * @brief BMI088 I2C采样与平衡车姿态解算
 *
 * 适用于两轮平衡车：
 *
 * 1. 上电静止校准陀螺仪零偏；
 * 2. X/Y平衡轴不设置角速度死区；
 * 3. Roll/Pitch采用互补滤波，防止角度长期漂移；
 * 4. Yaw采用Z轴角速度低通与梯形积分；
 * 5. 仅在连续确认静止后更新Z轴零偏并冻结yaw，抑制静止零漂；
 * 6. 运动时停止学习零偏，避免把真实转向误判为传感器偏置。
 *
 * 调用顺序：
 *
 *     BMI088_Gyro_Init();
 *     BMI088_Gyro_Calibrate(20);
 *
 *     平衡控制工程每2ms调用：
 *     BMI088_Gyro_Update(&gyro, 2);
 */

#include "gyro_bmi088.h"
#include "bsp_i2c.h"
#include "ti_msp_dl_config.h"

#include <math.h>
#include <stdint.h>


/* ============================================================
 * I2C地址
 * ============================================================ */

#define BMI088_GYRO_ADDR_0              (0x68U)
#define BMI088_GYRO_ADDR_1              (0x69U)

#define BMI088_ACCEL_ADDR_0             (0x18U)
#define BMI088_ACCEL_ADDR_1             (0x19U)


/* ============================================================
 * 陀螺仪寄存器
 * ============================================================ */

#define BMI088_GYRO_REG_CHIP_ID         (0x00U)
#define BMI088_GYRO_REG_RATE_X_LSB      (0x02U)
#define BMI088_GYRO_REG_RANGE           (0x0FU)
#define BMI088_GYRO_REG_BW              (0x10U)
#define BMI088_GYRO_REG_LPM1            (0x11U)


/* ============================================================
 * 加速度计寄存器
 * ============================================================ */

#define BMI088_ACCEL_REG_CHIP_ID        (0x00U)
#define BMI088_ACCEL_REG_DATA_X_LSB     (0x12U)
#define BMI088_ACCEL_REG_CONF           (0x40U)
#define BMI088_ACCEL_REG_RANGE          (0x41U)
#define BMI088_ACCEL_REG_PWR_CONF       (0x7CU)
#define BMI088_ACCEL_REG_PWR_CTRL       (0x7DU)


/* ============================================================
 * 芯片ID
 * ============================================================ */

#define BMI088_GYRO_ID_VALUE            (0x0FU)
#define BMI088_ACCEL_ID_VALUE           (0x1EU)


/* ============================================================
 * 传感器配置
 * ============================================================ */

/* 陀螺仪±500°/s */
#define BMI088_GYRO_RANGE_500DPS        (0x02U)

/* 陀螺仪1000Hz输出，116Hz带宽，供500Hz平衡内环读取。 */
#define BMI088_GYRO_BW_1000HZ_116HZ     (0x02U)

/* 加速度计±3g */
#define BMI088_ACCEL_RANGE_3G           (0x00U)

/* 加速度计Normal滤波、800Hz输出。 */
#define BMI088_ACCEL_CONF_800HZ         (0xABU)

#define BMI088_ACCEL_ACTIVE             (0x00U)
#define BMI088_ACCEL_ENABLE             (0x04U)


/* ============================================================
 * 姿态解算参数
 * ============================================================ */

#define BMI088_RAD_TO_DEG               (57.2957795f)

/*
 * 互补滤波时间常数。
 *
 * 采样周期10ms时：
 *
 * gyro_weight = 0.5 / (0.5 + 0.01)
 *             ≈ 0.9804
 *
 * 即约98%陀螺仪、2%加速度计。
 */

/*
 * 允许使用加速度计修正角度的模长范围：
 *
 * 0.5g～1.5g。
 *
 * 小车强烈振动或加速时，暂时不使用加速度倾角修正。
 */


/* ============================================================
 * 快速校准参数
 * ============================================================ */

/* 最大允许校准次数 */
#define BMI088_MAX_CAL_SAMPLES          (1000U)

/*
 * 校准前丢弃5个样本。
 *
 * 100Hz下约50ms。
 */

/*
 * Z轴偏航角速度死区，单位为0.1°/s。
 *
 * 2表示±0.2°/s以内视为0。这里只消除量化噪声，不宜设置过大，
 * 否则会丢失真实的慢速转动。
 */

/*
 * 仅靠陀螺仪抑制yaw零漂的参数。
 *
 * 先用加速度模长和三轴角速度判断是否静止。为了避免阈值附近反复切换，
 * 进入静止和退出静止使用不同阈值。连续静止达到确认时间后：
 *
 * 1. 缓慢跟踪Z轴原始零偏，补偿温漂；
 * 2. 将用于yaw积分的Z轴角速度置零；
 * 3. 保持当前yaw，不强制回到0°。
 */

/* 静止后Z轴零偏跟踪时间常数。数值越小，温漂跟踪越快。 */

/* 用于yaw积分的Z轴角速度一阶低通时间常数。 */


/* ============================================================
 * 全局变量
 * ============================================================ */

static BMI088_GyroData g_gyro;

typedef struct {
    int16_t yaw_x10;
    uint32_t update_ms;
    uint8_t valid;
} BMI088_YawSnapshot;

/* 主循环写非活动槽后一次性切换索引，中断读取时不会遇到半更新数据。 */
static BMI088_YawSnapshot g_yaw_snapshot[2];
static volatile uint8_t g_yaw_snapshot_active;

static float g_roll_deg;
static float g_pitch_deg;
static float g_yaw_deg;
static float g_mag_yaw_offset_deg;
static uint8_t g_mag_yaw_reference_valid;

static uint8_t g_angle_initialized;

/*
 * 使用浮点数保存零偏，避免整数平均导致精度损失。
 */
static float g_bias_x;
static float g_bias_y;
static float g_bias_z;

/* 纯陀螺仪yaw零漂抑制运行状态。 */
static uint32_t g_yaw_stationary_ms;
static uint8_t g_yaw_stationary;
static float g_yaw_rate_filtered_dps;
static float g_yaw_rate_previous_dps;

static uint8_t g_calibration_active;
static uint16_t g_calibration_target;
static uint16_t g_calibration_warmup;
static uint16_t g_calibration_count;
static int64_t g_calibration_sum_x;
static int64_t g_calibration_sum_y;
static int64_t g_calibration_sum_z;
static uint32_t g_calibration_next_ms;
static uint32_t g_service_last_ms;
static uint8_t g_service_time_valid;


/* ============================================================
 * 基础辅助函数
 * ============================================================ */

static void BMI088_DelayMs(uint32_t ms)
{
    while (ms-- != 0U) {
        delay_cycles(CPUCLK_FREQ / 1000U);
    }
}


/**
 * @brief 原始陀螺仪数据转换为0.1°/s
 *
 * ±500°/s量程：
 *
 * raw = 32768 对应 500°/s。
 */
static int16_t BMI088_RawToDpsX10(int32_t raw)
{
    if (raw > 32767L) {
        raw = 32767L;
    } else if (raw < -32768L) {
        raw = -32768L;
    }

    return (int16_t) ((raw * 10000L) / 65536L);
}


/**
 * @brief 原始加速度数据转换为mg
 *
 * ±3g量程：
 *
 * raw = 32768 对应 3000mg。
 */
static int16_t BMI088_RawToMg(int16_t raw)
{
    return (int16_t) (((int32_t) raw * 3000L) / 32768L);
}


/**
 * @brief 浮点数转换为0.1单位整数
 */
static int16_t BMI088_FloatToX10(float value)
{
    if (value >= 0.0f) {
        return (int16_t) ((value * 10.0f) + 0.5f);
    }

    return (int16_t) ((value * 10.0f) - 0.5f);
}


/**
 * @brief 浮点数四舍五入为int32_t
 */
static int32_t BMI088_RoundFloatToInt32(float value)
{
    if (value >= 0.0f) {
        return (int32_t) (value + 0.5f);
    }

    return (int32_t) (value - 0.5f);
}


/**
 * @brief 将角度限制到-180°～180°
 */
static float BMI088_WrapAngle(float angle_deg)
{
    while (angle_deg >= 180.0f) {
        angle_deg -= 360.0f;
    }

    while (angle_deg < -180.0f) {
        angle_deg += 360.0f;
    }

    return angle_deg;
}


/**
 * @brief 仅对Z轴偏航角速度应用死区
 */
static int16_t BMI088_ApplyZDeadband(int16_t value_x10)
{
    if ((value_x10 >= -BMI088_Z_DEADBAND_X10) &&
        (value_x10 <= BMI088_Z_DEADBAND_X10)) {
        return 0;
    }

    return value_x10;
}


/**
 * @brief 返回有符号0.1单位数据的绝对值。
 */
static int16_t BMI088_AbsX10(int16_t value)
{
    return (value < 0) ? (int16_t) (-value) : value;
}


/**
 * @brief 重置yaw滤波、静止判定和积分前一拍状态。
 */
static void BMI088_ResetYawRuntimeState(void)
{
    g_yaw_stationary_ms = 0U;
    g_yaw_stationary = 0U;
    g_yaw_rate_filtered_dps = 0.0f;
    g_yaw_rate_previous_dps = 0.0f;

    g_gyro.yaw_rate_x10 = 0;
    g_gyro.yaw_stationary_ms = 0U;
    g_gyro.yaw_stationary = 0U;
}


/**
 * @brief 判断静止、跟踪Z轴零偏并决定是否冻结yaw。
 *
 * 加速度计不能提供绝对yaw，但可以帮助确认设备是否真的静止。只有连续满足
 * 静止条件达到确认时间后才调整Z轴零偏，运动期间绝不学习零偏。
 */
static uint8_t BMI088_UpdateYawStationary(uint16_t elapsed_ms)
{
    float ax = (float) g_gyro.accel_mg_x;
    float ay = (float) g_gyro.accel_mg_y;
    float az = (float) g_gyro.accel_mg_z;
    float magnitude_sq = (ax * ax) + (ay * ay) + (az * az);
    int16_t xy_limit_x10;
    int16_t z_limit_x10;
    uint8_t gyro_still;
    uint8_t accel_still;

    if (g_yaw_stationary != 0U) {
        xy_limit_x10 = BMI088_YAW_STILL_EXIT_XY_X10;
        z_limit_x10 = BMI088_YAW_STILL_EXIT_Z_X10;
    } else {
        xy_limit_x10 = BMI088_YAW_STILL_ENTER_XY_X10;
        z_limit_x10 = BMI088_YAW_STILL_ENTER_Z_X10;
    }

    gyro_still =
        (BMI088_AbsX10(g_gyro.dps_x10) <= xy_limit_x10) &&
        (BMI088_AbsX10(g_gyro.dps_y10) <= xy_limit_x10) &&
        (BMI088_AbsX10(g_gyro.dps_z10) <= z_limit_x10);

    accel_still =
        (magnitude_sq >= BMI088_YAW_STILL_MIN_MG_SQ) &&
        (magnitude_sq <= BMI088_YAW_STILL_MAX_MG_SQ);

    if ((gyro_still != 0U) && (accel_still != 0U)) {
        if (g_yaw_stationary_ms <
            (UINT32_MAX - (uint32_t) elapsed_ms)) {
            g_yaw_stationary_ms += (uint32_t) elapsed_ms;
        } else {
            g_yaw_stationary_ms = UINT32_MAX;
        }

        if (g_yaw_stationary_ms >= BMI088_YAW_STILL_CONFIRM_MS) {
            float dt_sec = (float) elapsed_ms / 1000.0f;
            float alpha = dt_sec /
                (BMI088_YAW_BIAS_TRACK_TIME_SEC + dt_sec);

            g_yaw_stationary = 1U;

            /*
             * raw_z是未扣除零偏的原始值。静止时将零偏缓慢拉向raw_z，
             * 用于补偿温度和供电变化造成的长期偏移。
             */
            g_bias_z += alpha * ((float) g_gyro.raw_z - g_bias_z);
            g_gyro.gyro_bias_z =
                (int16_t) BMI088_RoundFloatToInt32(g_bias_z);

            /* 静止锁定后不允许噪声继续积分为yaw。 */
            g_gyro.dps_z10 = 0;
        }
    } else {
        g_yaw_stationary_ms = 0U;
        g_yaw_stationary = 0U;
    }

    g_gyro.yaw_stationary_ms = g_yaw_stationary_ms;
    g_gyro.yaw_stationary = g_yaw_stationary;

    return g_yaw_stationary;
}


/**
 * @brief 更新用于yaw积分的Z轴滤波角速度。
 */
static float BMI088_UpdateYawRateFilter(uint16_t elapsed_ms,
                                        uint8_t stationary)
{
    float dt_sec = (float) elapsed_ms / 1000.0f;
    float input_rate_dps = (float) g_gyro.dps_z10 / 10.0f;
    float alpha;

    if (stationary != 0U) {
        g_yaw_rate_filtered_dps = 0.0f;
        g_yaw_rate_previous_dps = 0.0f;
    } else {
        alpha = dt_sec /
            (BMI088_YAW_RATE_FILTER_TIME_SEC + dt_sec);

        g_yaw_rate_filtered_dps +=
            alpha * (input_rate_dps - g_yaw_rate_filtered_dps);
    }

    g_gyro.yaw_rate_x10 =
        BMI088_FloatToX10(g_yaw_rate_filtered_dps);

    return g_yaw_rate_filtered_dps;
}


/* ============================================================
 * 芯片地址探测
 * ============================================================ */

static int BMI088_ReadIdAt(uint8_t addr,
                           uint8_t expected_id,
                           uint8_t *id)
{
    int ret;

    if (id == 0) {
        return BMI088_GYRO_ERR_PARAM;
    }

    ret = BSP_I2C_ReadRegs(
        addr,
        BMI088_GYRO_REG_CHIP_ID,
        id,
        1U);

    if (ret != BSP_I2C_OK) {
        return BMI088_GYRO_ERR_I2C;
    }

    if (*id != expected_id) {
        return BMI088_GYRO_ERR_ID;
    }

    return BMI088_GYRO_OK;
}


/**
 * @brief 探测陀螺仪地址
 */
static int BMI088_SelectGyroAddress(void)
{
    uint8_t id = 0U;
    int ret;

    ret = BMI088_ReadIdAt(
        BMI088_GYRO_ADDR_0,
        BMI088_GYRO_ID_VALUE,
        &id);

    if (ret == BMI088_GYRO_OK) {
        g_gyro.i2c_addr = BMI088_GYRO_ADDR_0;
        g_gyro.chip_id = id;

        return BMI088_GYRO_OK;
    }

    g_gyro.chip_id = id;

    ret = BMI088_ReadIdAt(
        BMI088_GYRO_ADDR_1,
        BMI088_GYRO_ID_VALUE,
        &id);

    if (ret == BMI088_GYRO_OK) {
        g_gyro.i2c_addr = BMI088_GYRO_ADDR_1;
        g_gyro.chip_id = id;

        return BMI088_GYRO_OK;
    }

    g_gyro.chip_id = id;

    if (ret == BMI088_GYRO_ERR_I2C) {
        return BMI088_GYRO_ERR_I2C;
    }

    return BMI088_GYRO_ERR_ID;
}


/**
 * @brief 探测加速度计地址
 */
static int BMI088_SelectAccelAddress(void)
{
    uint8_t id = 0U;
    int ret;

    ret = BMI088_ReadIdAt(
        BMI088_ACCEL_ADDR_0,
        BMI088_ACCEL_ID_VALUE,
        &id);

    if (ret == BMI088_GYRO_OK) {
        g_gyro.accel_i2c_addr = BMI088_ACCEL_ADDR_0;
        g_gyro.accel_chip_id = id;

        return BMI088_GYRO_OK;
    }

    g_gyro.accel_chip_id = id;

    ret = BMI088_ReadIdAt(
        BMI088_ACCEL_ADDR_1,
        BMI088_ACCEL_ID_VALUE,
        &id);

    if (ret == BMI088_GYRO_OK) {
        g_gyro.accel_i2c_addr = BMI088_ACCEL_ADDR_1;
        g_gyro.accel_chip_id = id;

        return BMI088_GYRO_OK;
    }

    g_gyro.accel_chip_id = id;

    if (ret == BMI088_GYRO_ERR_I2C) {
        return BMI088_GYRO_ERR_I2C;
    }

    return BMI088_GYRO_ERR_ACCEL_ID;
}


/* ============================================================
 * 原始数据读取
 * ============================================================ */

static int BMI088_ReadGyroRaw(int16_t *x,
                              int16_t *y,
                              int16_t *z)
{
    uint8_t rx[6];

    if ((x == 0) || (y == 0) || (z == 0)) {
        return BMI088_GYRO_ERR_PARAM;
    }

    if (BSP_I2C_ReadRegs(
            g_gyro.i2c_addr,
            BMI088_GYRO_REG_RATE_X_LSB,
            rx,
            sizeof(rx)) != BSP_I2C_OK) {
        return BMI088_GYRO_ERR_I2C;
    }

    *x = (int16_t) (
        ((uint16_t) rx[1] << 8U) |
        (uint16_t) rx[0]);

    *y = (int16_t) (
        ((uint16_t) rx[3] << 8U) |
        (uint16_t) rx[2]);

    *z = (int16_t) (
        ((uint16_t) rx[5] << 8U) |
        (uint16_t) rx[4]);

    return BMI088_GYRO_OK;
}


static int BMI088_ReadAccelRaw(int16_t *x,
                               int16_t *y,
                               int16_t *z)
{
    uint8_t rx[6];

    if ((x == 0) || (y == 0) || (z == 0)) {
        return BMI088_GYRO_ERR_PARAM;
    }

    if (BSP_I2C_ReadRegs(
            g_gyro.accel_i2c_addr,
            BMI088_ACCEL_REG_DATA_X_LSB,
            rx,
            sizeof(rx)) != BSP_I2C_OK) {
        return BMI088_GYRO_ERR_I2C;
    }

    *x = (int16_t) (
        ((uint16_t) rx[1] << 8U) |
        (uint16_t) rx[0]);

    *y = (int16_t) (
        ((uint16_t) rx[3] << 8U) |
        (uint16_t) rx[2]);

    *z = (int16_t) (
        ((uint16_t) rx[5] << 8U) |
        (uint16_t) rx[4]);

    return BMI088_GYRO_OK;
}


/* ============================================================
 * 加速度倾角
 * ============================================================ */

static uint8_t BMI088_GetAccelAngles(float *roll_deg,
                                     float *pitch_deg)
{
    float ax;
    float ay;
    float az;
    float magnitude_sq;

    if ((roll_deg == 0) || (pitch_deg == 0)) {
        return 0U;
    }

    ax = (float) g_gyro.accel_mg_x;
    ay = (float) g_gyro.accel_mg_y;
    az = (float) g_gyro.accel_mg_z;

    magnitude_sq =
        (ax * ax) +
        (ay * ay) +
        (az * az);

    /*
     * 强烈振动或线性加速时，
     * 加速度计不能准确代表重力方向。
     */
    if ((magnitude_sq < BMI088_ACCEL_MIN_MG_SQ) ||
        (magnitude_sq > BMI088_ACCEL_MAX_MG_SQ)) {
        return 0U;
    }

    *roll_deg =
        atan2f(ay, az) *
        BMI088_RAD_TO_DEG;

    *pitch_deg =
        atan2f(
            -ax,
            sqrtf((ay * ay) + (az * az))) *
        BMI088_RAD_TO_DEG;

    return 1U;
}


/* ============================================================
 * 数据清零
 * ============================================================ */

static void BMI088_ClearData(void)
{
    g_gyro.i2c_addr = BMI088_GYRO_ADDR_0;
    g_gyro.chip_id = 0U;

    g_gyro.accel_i2c_addr = BMI088_ACCEL_ADDR_0;
    g_gyro.accel_chip_id = 0U;

    g_gyro.status = BMI088_GYRO_ERR_I2C;

    g_gyro.raw_x = 0;
    g_gyro.raw_y = 0;
    g_gyro.raw_z = 0;

    g_gyro.dps_x10 = 0;
    g_gyro.dps_y10 = 0;
    g_gyro.dps_z10 = 0;

    g_gyro.accel_raw_x = 0;
    g_gyro.accel_raw_y = 0;
    g_gyro.accel_raw_z = 0;

    g_gyro.accel_mg_x = 0;
    g_gyro.accel_mg_y = 0;
    g_gyro.accel_mg_z = 0;

    g_gyro.gyro_bias_x = 0;
    g_gyro.gyro_bias_y = 0;
    g_gyro.gyro_bias_z = 0;

    g_gyro.accel_roll_x10 = 0;
    g_gyro.accel_pitch_x10 = 0;

    g_gyro.roll_x10 = 0;
    g_gyro.pitch_x10 = 0;
    g_gyro.yaw_x10 = 0;
    g_gyro.yaw_rate_x10 = 0;
    g_gyro.yaw_stationary_ms = 0U;
    g_gyro.yaw_stationary = 0U;

    g_gyro.angle_valid = 0U;

    g_roll_deg = 0.0f;
    g_pitch_deg = 0.0f;
    g_yaw_deg = 0.0f;

    g_angle_initialized = 0U;

    g_bias_x = 0.0f;
    g_bias_y = 0.0f;
    g_bias_z = 0.0f;
    BMI088_ResetYawRuntimeState();
    g_calibration_active = 0U;
    g_calibration_target = 0U;
    g_calibration_warmup = 0U;
    g_calibration_count = 0U;
    g_service_last_ms = 0U;
    g_service_time_valid = 0U;
    g_yaw_snapshot[0].valid = 0U;
    g_yaw_snapshot[1].valid = 0U;
    g_yaw_snapshot_active = 0U;
}


/* ============================================================
 * 初始化
 * ============================================================ */

int BMI088_Gyro_Init(void)
{
    int ret;

    BMI088_ClearData();

    BMI088_DelayMs(100U);

    ret = BMI088_SelectGyroAddress();

    if (ret != BMI088_GYRO_OK) {
        g_gyro.status = ret;
        return ret;
    }

    ret = BMI088_SelectAccelAddress();

    if (ret != BMI088_GYRO_OK) {
        g_gyro.status = ret;
        return ret;
    }

    /*
     * 陀螺仪进入正常模式。
     */
    ret = BSP_I2C_WriteReg(
        g_gyro.i2c_addr,
        BMI088_GYRO_REG_LPM1,
        0x00U);

    if (ret != BSP_I2C_OK) {
        g_gyro.status = BMI088_GYRO_ERR_CONFIG;
        return g_gyro.status;
    }

    BMI088_DelayMs(30U);

    /*
     * 设置陀螺仪量程。
     */
    ret = BSP_I2C_WriteReg(
        g_gyro.i2c_addr,
        BMI088_GYRO_REG_RANGE,
        BMI088_GYRO_RANGE_500DPS);

    if (ret == BSP_I2C_OK) {
        ret = BSP_I2C_WriteReg(
            g_gyro.i2c_addr,
            BMI088_GYRO_REG_BW,
            BMI088_GYRO_BW_1000HZ_116HZ);
    }

    if (ret != BSP_I2C_OK) {
        g_gyro.status = BMI088_GYRO_ERR_CONFIG;
        return g_gyro.status;
    }

    /*
     * 唤醒加速度计。
     */
    ret = BSP_I2C_WriteReg(
        g_gyro.accel_i2c_addr,
        BMI088_ACCEL_REG_PWR_CONF,
        BMI088_ACCEL_ACTIVE);

    if (ret == BSP_I2C_OK) {
        BMI088_DelayMs(5U);

        ret = BSP_I2C_WriteReg(
            g_gyro.accel_i2c_addr,
            BMI088_ACCEL_REG_PWR_CTRL,
            BMI088_ACCEL_ENABLE);
    }

    if (ret == BSP_I2C_OK) {
        BMI088_DelayMs(5U);

        ret = BSP_I2C_WriteReg(
            g_gyro.accel_i2c_addr,
            BMI088_ACCEL_REG_CONF,
            BMI088_ACCEL_CONF_800HZ);
    }

    if (ret == BSP_I2C_OK) {
        ret = BSP_I2C_WriteReg(
            g_gyro.accel_i2c_addr,
            BMI088_ACCEL_REG_RANGE,
            BMI088_ACCEL_RANGE_3G);
    }

    if (ret != BSP_I2C_OK) {
        g_gyro.status = BMI088_GYRO_ERR_ACCEL_CONFIG;
        return g_gyro.status;
    }

    BMI088_DelayMs(20U);

    g_gyro.status = BMI088_GYRO_OK;

    return BMI088_GYRO_OK;
}


/* ============================================================
 * 数据读取
 * ============================================================ */

int BMI088_Gyro_Start(void)
{
    int ret;

    ret = BMI088_Gyro_Init();
    if (ret != BMI088_GYRO_OK) {
        return ret;
    }

    ret = BMI088_Gyro_Calibrate(
        BMI088_GYRO_DEFAULT_CALIBRATION_SAMPLES);
    if (ret == BMI088_GYRO_OK) {
        g_service_time_valid = 0U;
    }

    return ret;
}

int BMI088_Gyro_Read(BMI088_GyroData *data)
{
    int ret;

    float corrected_x_float;
    float corrected_y_float;
    float corrected_z_float;

    int32_t corrected_x;
    int32_t corrected_y;
    int32_t corrected_z;

    if (data == 0) {
        return BMI088_GYRO_ERR_PARAM;
    }

    ret = BMI088_ReadGyroRaw(
        &g_gyro.raw_x,
        &g_gyro.raw_y,
        &g_gyro.raw_z);

    if (ret == BMI088_GYRO_OK) {
        ret = BMI088_ReadAccelRaw(
            &g_gyro.accel_raw_x,
            &g_gyro.accel_raw_y,
            &g_gyro.accel_raw_z);
    }

    if (ret != BMI088_GYRO_OK) {
        g_gyro.status = ret;
        *data = g_gyro;

        return ret;
    }

    g_gyro.accel_mg_x =
        BMI088_RawToMg(g_gyro.accel_raw_x);

    g_gyro.accel_mg_y =
        BMI088_RawToMg(g_gyro.accel_raw_y);

    g_gyro.accel_mg_z =
        BMI088_RawToMg(g_gyro.accel_raw_z);

    /*
     * X/Y始终使用启动校准得到的零偏。
     * Z轴零偏只会在BMI088_Gyro_Update()连续确认静止后缓慢调整。
     */
    corrected_x_float =
        (float) g_gyro.raw_x - g_bias_x;

    corrected_y_float =
        (float) g_gyro.raw_y - g_bias_y;

    corrected_z_float =
        (float) g_gyro.raw_z - g_bias_z;

    corrected_x =
        BMI088_RoundFloatToInt32(corrected_x_float);

    corrected_y =
        BMI088_RoundFloatToInt32(corrected_y_float);

    corrected_z =
        BMI088_RoundFloatToInt32(corrected_z_float);

    /*
     * X/Y平衡轴不加死区。
     */
    g_gyro.dps_x10 =
        BMI088_RawToDpsX10(corrected_x);

    g_gyro.dps_y10 =
        BMI088_RawToDpsX10(corrected_y);

    /*
     * 仅Z轴偏航加小死区。
     */
    g_gyro.dps_z10 =
        BMI088_ApplyZDeadband(
            BMI088_RawToDpsX10(corrected_z));


    g_gyro.status = BMI088_GYRO_OK;

    *data = g_gyro;

    return BMI088_GYRO_OK;
}


/* ============================================================
 * 姿态更新
 * ============================================================ */

int BMI088_Gyro_Update(BMI088_GyroData *data,
                       uint16_t elapsed_ms)
{
    float accel_roll;
    float accel_pitch;

    float dt_sec;
    float gyro_weight;
    float predicted;
    float yaw_rate_dps;

    uint8_t accel_valid;
    uint8_t yaw_stationary;
    int ret;

    if ((data == 0) ||
        (elapsed_ms == 0U) ||
        (elapsed_ms > 1000U)) {
        return BMI088_GYRO_ERR_PARAM;
    }

    ret = BMI088_Gyro_Read(data);

    if (ret != BMI088_GYRO_OK) {
        return ret;
    }

    dt_sec =
        (float) elapsed_ms / 1000.0f;

    /*
     * 先判断静止并更新Z轴零偏，再得到真正用于yaw积分的滤波角速度。
     * 静止锁定后yaw_rate_dps固定为0，因此yaw不会继续爬升或下降。
     */
    yaw_stationary =
        BMI088_UpdateYawStationary(elapsed_ms);

    yaw_rate_dps =
        BMI088_UpdateYawRateFilter(elapsed_ms,
                                   yaw_stationary);

    accel_valid =
        BMI088_GetAccelAngles(
            &accel_roll,
            &accel_pitch);

    if (accel_valid != 0U) {
        g_gyro.accel_roll_x10 =
            BMI088_FloatToX10(accel_roll);

        g_gyro.accel_pitch_x10 =
            BMI088_FloatToX10(accel_pitch);
    }

    if (g_angle_initialized == 0U) {
        if (accel_valid != 0U) {
            g_roll_deg = accel_roll;
            g_pitch_deg = accel_pitch;
        } else {
            g_roll_deg = 0.0f;
            g_pitch_deg = 0.0f;
        }

        g_yaw_deg = 0.0f;
        g_yaw_rate_previous_dps = yaw_rate_dps;
        g_angle_initialized = 1U;
    } else {
        gyro_weight =
            BMI088_FILTER_TIME_SEC /
            (BMI088_FILTER_TIME_SEC + dt_sec);

        /*
         * 横滚角：陀螺仪负责快速响应，加速度计负责长期修正。
         */
        predicted =
            BMI088_WrapAngle(
                g_roll_deg +
                ((float) g_gyro.dps_x10 *
                 dt_sec / 10.0f));

        if (accel_valid != 0U) {
            predicted =
                BMI088_WrapAngle(
                    predicted +
                    ((1.0f - gyro_weight) *
                     BMI088_WrapAngle(
                         accel_roll - predicted)));
        }

        g_roll_deg = predicted;

        /*
         * 俯仰角。
         */
        predicted =
            BMI088_WrapAngle(
                g_pitch_deg +
                ((float) g_gyro.dps_y10 *
                 dt_sec / 10.0f));

        if (accel_valid != 0U) {
            predicted =
                BMI088_WrapAngle(
                    predicted +
                    ((1.0f - gyro_weight) *
                     BMI088_WrapAngle(
                         accel_pitch - predicted)));
        }

        g_pitch_deg = predicted;

        /*
         * 偏航角：
         *
         * 运动时使用低通后的Z轴角速度进行梯形积分；
         * 连续确认静止后保持当前yaw，并在线更新Z轴零偏。
         */
        if (yaw_stationary == 0U) {
            g_yaw_deg =
                BMI088_WrapAngle(
                    g_yaw_deg +
                    (0.5f *
                     (g_yaw_rate_previous_dps + yaw_rate_dps) *
                     dt_sec));

            g_yaw_rate_previous_dps = yaw_rate_dps;
        } else {
            g_yaw_rate_previous_dps = 0.0f;
        }
    }

    g_gyro.roll_x10 =
        BMI088_FloatToX10(g_roll_deg);

    g_gyro.pitch_x10 =
        BMI088_FloatToX10(g_pitch_deg);

    g_gyro.yaw_x10 =
        BMI088_FloatToX10(g_yaw_deg);

    g_gyro.angle_valid =
        g_angle_initialized;

    *data = g_gyro;

    return BMI088_GYRO_OK;
}


/* ============================================================
 * 快速零偏校准
 * ============================================================ */

int BMI088_Gyro_Service(uint32_t now_ms)
{
    uint32_t elapsed_ms;
    int ret;

    if (g_gyro.status != BMI088_GYRO_OK) {
        return g_gyro.status;
    }

    if (g_service_time_valid == 0U) {
        g_service_last_ms = now_ms;
        g_service_time_valid = 1U;
        return BMI088_GYRO_OK;
    }

    elapsed_ms = now_ms - g_service_last_ms;
    if (elapsed_ms < BMI088_GYRO_DEFAULT_UPDATE_PERIOD_MS) {
        return BMI088_GYRO_OK;
    }

    if (elapsed_ms > 100U) {
        elapsed_ms = 100U;
    }

    g_service_last_ms = now_ms;
    ret = BMI088_Gyro_Update(&g_gyro, (uint16_t) elapsed_ms);
    if ((ret == BMI088_GYRO_OK) && (g_gyro.angle_valid != 0U)) {
        uint8_t next = (uint8_t) (g_yaw_snapshot_active ^ 1U);

        g_yaw_snapshot[next].yaw_x10 = g_gyro.yaw_x10;
        g_yaw_snapshot[next].update_ms = now_ms;
        g_yaw_snapshot[next].valid = 1U;
        g_yaw_snapshot_active = next;
    }
    return ret;
}

int BMI088_Gyro_Calibrate(uint16_t sample_count)
{
    int64_t sum_x = 0;
    int64_t sum_y = 0;
    int64_t sum_z = 0;

    int16_t x;
    int16_t y;
    int16_t z;

    uint16_t i;
    int ret;

    if ((sample_count == 0U) ||
        (sample_count > BMI088_MAX_CAL_SAMPLES)) {
        return BMI088_GYRO_ERR_PARAM;
    }

    /*
     * 丢弃前5个样本，约50ms。
     *
     * 不要求平衡车自行站立，
     * 上电时手扶车体或者让车靠在支架上即可。
     */
    for (i = 0U;
         i < BMI088_CAL_WARMUP_SAMPLES;
         i++) {

        ret = BMI088_ReadGyroRaw(&x, &y, &z);

        if (ret != BMI088_GYRO_OK) {
            g_gyro.status = ret;
            return ret;
        }

        BMI088_DelayMs(10U);
    }

    /*
     * 推荐sample_count为20。
     *
     * 20次采样约200ms。
     */
    for (i = 0U; i < sample_count; i++) {
        ret = BMI088_ReadGyroRaw(&x, &y, &z);

        if (ret != BMI088_GYRO_OK) {
            g_gyro.status = ret;
            return ret;
        }

        sum_x += (int64_t) x;
        sum_y += (int64_t) y;
        sum_z += (int64_t) z;

        BMI088_DelayMs(10U);
    }

    g_bias_x =
        (float) sum_x /
        (float) sample_count;

    g_bias_y =
        (float) sum_y /
        (float) sample_count;

    g_bias_z =
        (float) sum_z /
        (float) sample_count;
    BMI088_ResetYawRuntimeState();

    g_gyro.gyro_bias_x =
        (int16_t)
        BMI088_RoundFloatToInt32(g_bias_x);

    g_gyro.gyro_bias_y =
        (int16_t)
        BMI088_RoundFloatToInt32(g_bias_y);

    g_gyro.gyro_bias_z =
        (int16_t)
        BMI088_RoundFloatToInt32(g_bias_z);

    /*
     * 读取一次加速度数据，
     * 初始化平衡车的起始倾角。
     */
    ret = BMI088_ReadAccelRaw(
        &g_gyro.accel_raw_x,
        &g_gyro.accel_raw_y,
        &g_gyro.accel_raw_z);

    if (ret != BMI088_GYRO_OK) {
        g_gyro.status = ret;
        return ret;
    }

    g_gyro.accel_mg_x =
        BMI088_RawToMg(g_gyro.accel_raw_x);

    g_gyro.accel_mg_y =
        BMI088_RawToMg(g_gyro.accel_raw_y);

    g_gyro.accel_mg_z =
        BMI088_RawToMg(g_gyro.accel_raw_z);

    g_gyro.status = BMI088_GYRO_OK;

    BMI088_Gyro_ResetAngles();

    return BMI088_GYRO_OK;
}


/* ============================================================
 * 姿态角复位
 * ============================================================ */

void BMI088_Gyro_ResetAngles(void)
{
    float accel_roll;
    float accel_pitch;

    if (BMI088_GetAccelAngles(
            &accel_roll,
            &accel_pitch) != 0U) {

        g_roll_deg = accel_roll;
        g_pitch_deg = accel_pitch;
        g_angle_initialized = 1U;
    } else {
        g_roll_deg = 0.0f;
        g_pitch_deg = 0.0f;
        g_angle_initialized = 0U;
    }

    g_yaw_deg = 0.0f;
    g_mag_yaw_offset_deg = 0.0f;
    g_mag_yaw_reference_valid = 0U;
    BMI088_ResetYawRuntimeState();

    g_gyro.roll_x10 =
        BMI088_FloatToX10(g_roll_deg);

    g_gyro.pitch_x10 =
        BMI088_FloatToX10(g_pitch_deg);

    g_gyro.yaw_x10 = 0;

    g_gyro.angle_valid =
        g_angle_initialized;
}

void BMI088_Gyro_FuseMagHeading(int16_t heading_x10,
                                uint16_t gain_x1000)
{
    float magnetic_heading;
    float aligned_heading;
    float error;
    float gain;

    if ((g_gyro.status != BMI088_GYRO_OK) ||
        (g_angle_initialized == 0U) || (gain_x1000 == 0U)) {
        return;
    }
    if (gain_x1000 > 1000U) gain_x1000 = 1000U;
    magnetic_heading = BMI088_WrapAngle((float) heading_x10 / 10.0f);

    if (g_mag_yaw_reference_valid == 0U) {
        /* 保留当前yaw零点，只利用磁力计消除后续长期漂移。 */
        g_mag_yaw_offset_deg = BMI088_WrapAngle(
            g_yaw_deg - magnetic_heading);
        g_mag_yaw_reference_valid = 1U;
    }

    aligned_heading = BMI088_WrapAngle(
        magnetic_heading + g_mag_yaw_offset_deg);
    error = BMI088_WrapAngle(aligned_heading - g_yaw_deg);
    gain = (float) gain_x1000 / 1000.0f;
    g_yaw_deg = BMI088_WrapAngle(g_yaw_deg + error * gain);
    g_gyro.yaw_x10 = BMI088_FloatToX10(g_yaw_deg);
}

void BMI088_Gyro_ResetMagReference(void)
{
    g_mag_yaw_offset_deg = 0.0f;
    g_mag_yaw_reference_valid = 0U;
}


/* ============================================================
 * 获取当前数据
 * ============================================================ */

const BMI088_GyroData *BMI088_Gyro_GetData(void)
{
    return &g_gyro;
}

uint8_t BMI088_Gyro_GetYawSnapshot(int16_t *yaw_x10, uint32_t *update_ms)
{
    uint8_t active;

    if ((yaw_x10 == 0) || (update_ms == 0)) {
        return 0U;
    }

    active = g_yaw_snapshot_active;
    if (g_yaw_snapshot[active].valid == 0U) {
        return 0U;
    }

    *yaw_x10 = g_yaw_snapshot[active].yaw_x10;
    *update_ms = g_yaw_snapshot[active].update_ms;
    return 1U;
}


int BMI088_Gyro_CalibrationStart(uint16_t sample_count, uint32_t now_ms)
{
    if ((sample_count == 0U) ||
        (sample_count > BMI088_MAX_CAL_SAMPLES)) {
        return BMI088_GYRO_ERR_PARAM;
    }

    g_calibration_target = sample_count;
    g_calibration_warmup = 0U;
    g_calibration_count = 0U;
    g_calibration_sum_x = 0;
    g_calibration_sum_y = 0;
    g_calibration_sum_z = 0;
    g_calibration_next_ms = now_ms;
    g_calibration_active = 1U;
    g_gyro.status = BMI088_GYRO_CALIBRATING;
    return BMI088_GYRO_CALIBRATING;
}

int BMI088_Gyro_CalibrationService(uint32_t now_ms)
{
    int16_t x;
    int16_t y;
    int16_t z;
    int ret;

    if (g_calibration_active == 0U) {
        return g_gyro.status;
    }

    if ((int32_t) (now_ms - g_calibration_next_ms) < 0) {
        return BMI088_GYRO_CALIBRATING;
    }

    ret = BMI088_ReadGyroRaw(&x, &y, &z);
    if (ret != BMI088_GYRO_OK) {
        g_calibration_active = 0U;
        g_gyro.status = ret;
        return ret;
    }

    g_calibration_next_ms = now_ms + 10U;

    if (g_calibration_warmup < BMI088_CAL_WARMUP_SAMPLES) {
        g_calibration_warmup++;
        return BMI088_GYRO_CALIBRATING;
    }

    g_calibration_sum_x += (int64_t) x;
    g_calibration_sum_y += (int64_t) y;
    g_calibration_sum_z += (int64_t) z;
    g_calibration_count++;

    if (g_calibration_count < g_calibration_target) {
        return BMI088_GYRO_CALIBRATING;
    }

    g_bias_x = (float) g_calibration_sum_x /
               (float) g_calibration_target;
    g_bias_y = (float) g_calibration_sum_y /
               (float) g_calibration_target;
    g_bias_z = (float) g_calibration_sum_z /
               (float) g_calibration_target;
    BMI088_ResetYawRuntimeState();

    g_gyro.gyro_bias_x =
        (int16_t) BMI088_RoundFloatToInt32(g_bias_x);
    g_gyro.gyro_bias_y =
        (int16_t) BMI088_RoundFloatToInt32(g_bias_y);
    g_gyro.gyro_bias_z =
        (int16_t) BMI088_RoundFloatToInt32(g_bias_z);

    ret = BMI088_ReadAccelRaw(&g_gyro.accel_raw_x,
                              &g_gyro.accel_raw_y,
                              &g_gyro.accel_raw_z);
    if (ret != BMI088_GYRO_OK) {
        g_calibration_active = 0U;
        g_gyro.status = ret;
        return ret;
    }

    g_gyro.accel_mg_x = BMI088_RawToMg(g_gyro.accel_raw_x);
    g_gyro.accel_mg_y = BMI088_RawToMg(g_gyro.accel_raw_y);
    g_gyro.accel_mg_z = BMI088_RawToMg(g_gyro.accel_raw_z);
    g_calibration_active = 0U;
    g_gyro.status = BMI088_GYRO_OK;
    BMI088_Gyro_ResetAngles();
    return BMI088_GYRO_OK;
}
