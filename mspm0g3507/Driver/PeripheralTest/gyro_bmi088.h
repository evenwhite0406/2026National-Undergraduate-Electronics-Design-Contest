/**
 * @file gyro_bmi088.h
 * @brief BMI088 陀螺仪、加速度计和姿态角模块。
 *
 * BMI088 在同一封装内包含两个 I2C 器件：
 *   - 陀螺仪地址为 0x68 或 0x69；
 *   - 加速度计地址为 0x18 或 0x19。
 *
 * 本模块同时读取两个器件。横滚角和俯仰角采用互补滤波计算：短期运动由
 * 陀螺仪角速度积分得到，加速度计测得的重力方向用于校正长期漂移。
 * BMI088本身没有磁力计，偏航角由Z轴角速度积分。本版本在连续确认静止后
 * 在线跟踪Z轴零偏并冻结yaw积分，可显著抑制静止零漂；运动过程中的长期
 * 绝对航向误差仍无法仅靠BMI088彻底消除。
 *
 * 典型用法（以稳定周期调用 Update）：
 * @code
 * BMI088_GyroData imu;
 *
 * if (BMI088_Gyro_Init() == BMI088_GYRO_OK) {
 *     BMI088_Gyro_Calibrate(100U);   // 保持开发板静止约 1 秒。
 *     while (1) {
 *         BMI088_Gyro_Update(&imu, 10U);
 *         // imu.roll_x10 / 10.0f 为横滚角，单位为度。
 *         // imu.dps_z10 / 10.0f 为 Z 轴角速度，单位为度/秒。
 *         delay_cycles(320000U);     // CPU 为 32 MHz 时约为 10 ms。
 *     }
 * }
 * @endcode
 */

#ifndef GYRO_BMI088_H
#define GYRO_BMI088_H

#include <stdint.h>

/*
 * 最简调用：BMI088_Gyro_Start(); 主循环持续 BMI088_Gyro_Service(g_ms);
 * 使用 BMI088_Gyro_GetData() 读取 roll/pitch/yaw。启动校准时必须静止。
 */

#define BMI088_GYRO_OK                 (0)
#define BMI088_GYRO_CALIBRATING        (1)
#define BMI088_GYRO_ERR_I2C            (-20)
#define BMI088_GYRO_ERR_ID             (-21)
#define BMI088_GYRO_ERR_CONFIG         (-22)
#define BMI088_GYRO_ERR_PARAM          (-23)
#define BMI088_GYRO_ERR_ACCEL_ID       (-24)
#define BMI088_GYRO_ERR_ACCEL_CONFIG   (-25)

/* 应用层默认配置；校准期间传感器必须保持静止。 */
extern uint16_t BMI088_GYRO_DEFAULT_CALIBRATION_SAMPLES;
extern uint16_t BMI088_GYRO_DEFAULT_UPDATE_PERIOD_MS;

/* 实车可调姿态与静止检测参数。除非理解其作用，否则保持默认值。 */
extern float BMI088_FILTER_TIME_SEC;
extern float BMI088_ACCEL_MIN_MG_SQ;
extern float BMI088_ACCEL_MAX_MG_SQ;
extern int16_t BMI088_Z_DEADBAND_X10;
extern int16_t BMI088_YAW_STILL_ENTER_XY_X10;
extern int16_t BMI088_YAW_STILL_ENTER_Z_X10;
extern int16_t BMI088_YAW_STILL_EXIT_XY_X10;
extern int16_t BMI088_YAW_STILL_EXIT_Z_X10;
extern float BMI088_YAW_STILL_MIN_MG_SQ;
extern float BMI088_YAW_STILL_MAX_MG_SQ;
extern uint32_t BMI088_YAW_STILL_CONFIRM_MS;
extern float BMI088_YAW_BIAS_TRACK_TIME_SEC;
extern float BMI088_YAW_RATE_FILTER_TIME_SEC;
extern uint16_t BMI088_CAL_WARMUP_SAMPLES;

/**
 * 最新一次完整的 BMI088 采样和姿态结果。
 *
 * 以 _x10 结尾的值使用定点单位。例如 roll_x10 == 123 表示 +12.3°，
 * dps_x10 == -45 表示 -4.5°/s。
 */
typedef struct {
    uint8_t i2c_addr;
    uint8_t chip_id;
    uint8_t accel_i2c_addr;
    uint8_t accel_chip_id;
    int status;

    int16_t raw_x;
    int16_t raw_y;
    int16_t raw_z;
    int16_t dps_x10;
    int16_t dps_y10;
    int16_t dps_z10;

    int16_t accel_raw_x;
    int16_t accel_raw_y;
    int16_t accel_raw_z;
    int16_t accel_mg_x;
    int16_t accel_mg_y;
    int16_t accel_mg_z;

    int16_t gyro_bias_x;
    int16_t gyro_bias_y;
    int16_t gyro_bias_z;

    int16_t accel_roll_x10;
    int16_t accel_pitch_x10;
    int16_t roll_x10;
    int16_t pitch_x10;
    int16_t yaw_x10;

    /*
     * 纯陀螺仪yaw零漂抑制状态：
     * yaw_rate_x10为实际用于yaw积分的滤波角速度；
     * yaw_stationary为1时保持当前yaw并在线跟踪Z轴零偏。
     */
    int16_t yaw_rate_x10;
    uint32_t yaw_stationary_ms;
    uint8_t yaw_stationary;

    uint8_t angle_valid;
} BMI088_GyroData;

/**
 * 检测并配置 BMI088 的两个 I2C 器件。
 * 在 SYSCFG_DL_init() 之后调用一次。
 */
int BMI088_Gyro_Init(void);

/** 一键完成器件初始化、默认零偏校准和姿态角复位。 */
int BMI088_Gyro_Start(void);

/**
 * 只读取加速度和角速度，不积分姿态角。需要姿态角时请使用
 * BMI088_Gyro_Update()。
 */
int BMI088_Gyro_Read(BMI088_GyroData *data);

/**
 * 读取一次数据并更新横滚、俯仰和偏航角。elapsed_ms 必须是距上次Update
 * 调用的真实间隔。建议保持稳定的2～10 ms周期，最大接受1000 ms。
 *
 * yaw在运动时采用滤波角速度和梯形积分；连续确认静止后冻结当前yaw，并
 * 缓慢更新Z轴零偏以补偿温漂。
 */
int BMI088_Gyro_Update(BMI088_GyroData *data, uint16_t elapsed_ms);

/** 按毫秒时间戳和默认周期更新；驱动内部维护时间差。 */
int BMI088_Gyro_Service(uint32_t now_ms);

/**
 * 测量静止状态下的陀螺仪零偏。阻塞函数运行期间必须保持开发板完全静止；
 * 采样 100 次约需一秒。
 */
int BMI088_Gyro_Calibrate(uint16_t sample_count);

/** 启动非阻塞零偏校准；之后在主循环调用 CalibrationService。 */
int BMI088_Gyro_CalibrationStart(uint16_t sample_count, uint32_t now_ms);

/**
 * 推进一次非阻塞校准。返回 CALIBRATING 表示尚未完成，返回 OK 表示完成，
 * 返回负值表示失败。
 */
int BMI088_Gyro_CalibrationService(uint32_t now_ms);

/**
 * 根据当前重力向量设置横滚角和俯仰角，并将偏航角清零。小车静止在所需
 * 参考姿态时调用。
 */
void BMI088_Gyro_ResetAngles(void);

/**
 * 使用磁力计航向缓慢修正yaw漂移。首次调用会自动对齐磁航向和当前yaw零点。
 * gain_x1000建议5～30，数值过大会把电机磁干扰直接带入姿态角。
 */
void BMI088_Gyro_FuseMagHeading(int16_t heading_x10,
                                uint16_t gain_x1000);

/** 磁力计重新标定或安装方向改变后清除磁航向对齐关系。 */
void BMI088_Gyro_ResetMagReference(void);

/** 返回指向模块最新状态的只读指针。 */
const BMI088_GyroData *BMI088_Gyro_GetData(void);

/**
 * 中断安全地读取最近一次成功更新的yaw及其毫秒时间戳。
 * 返回0表示尚无有效姿态；速度/航向控制优先使用本接口。
 */
uint8_t BMI088_Gyro_GetYawSnapshot(int16_t *yaw_x10, uint32_t *update_ms);

#endif
