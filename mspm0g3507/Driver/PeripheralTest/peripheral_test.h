/**
 * @file peripheral_test.h
 * @brief 基于 OLED 的按键、循迹、BMI088 和电机综合联调模块。
 *
 * 可将本模块作为临时应用入口：
 * @code
 * int main(void)
 * {
 *     SYSCFG_DL_init();
 *     PeripheralTest_Init();
 *     PeripheralTest_Run();           // 不会返回。
 * }
 * @endcode
 *
 * 电机测试期间必须架空小车。按键 1 启动自动电机序列，其余外设状态会
 * 持续刷新到 OLED。
 */

#ifndef PERIPHERAL_TEST_H
#define PERIPHERAL_TEST_H

#include <stdbool.h>

/* 综合联调页面的实车参数与刷新周期。 */
#define OLED_TEXT_COLS             (21U)
#define IMU_UPDATE_MS              (2U)
#define BLUETOOTH_WAVEFORM_MS      (50U)
#define OLED_UPDATE_MS             (100U)
#define GYRO_CALIBRATE_SAMPLES     (100U)
#define IMU_MAX_UPDATE_MS          (1000U)
#define MOTOR_TEST_SPEED           (20)
#define MOTOR_STEP_MS              (400U)
#define MOTOR_PAUSE_MS             (200U)
#define MAG_CALIBRATION_MS         (20000U)
#define MAG_YAW_GAIN_X1000         (20U)
#define MAX_SPEED_RAMP_MS          (300U)
#define MAX_SPEED_SETTLE_MS        (500U)
#define MAX_SPEED_SAMPLE_MS        (1000U)
#define MAX_SPEED_PAUSE_MS         (500U)
#define MAX_SPEED_RESULT_MS        (8000U)
#define SPEED_TUNE_TARGET_PERCENT  (15)
#define SPEED_TUNE_UART_PERIOD_MS  (100U)
#define ANGLE_DIR_DEADBAND_X10     (20L)
#define ANGLE_DIR_STOP_X10         (100L)
#define ANGLE_DIR_TEST_PWM         (12)
#define ANGLE_TUNE_STOP_X10        (180L)
#define ANGLE_TUNE_RATE_STOP_X10   (2500L)
#define ANGLE_TUNE_SATURATION_MS   (300U)
#define ENCODER_1_COUNTS_PER_REV   (785U) /* 实测10圈7845，取整为785。 */
#define ENCODER_2_COUNTS_PER_REV   (785U)
#define ENCODER_1_REVERSED         (false) /* 与当前正速度命令方向一致。 */
#define ENCODER_2_REVERSED         (true)

/** 初始化 OLED、电机和 BMI088，并在静止状态下校准陀螺仪零偏。 */
void PeripheralTest_Init(void);

/** 运行阻塞式测试循环；按一次按键 1 可测试两路电机。 */
void PeripheralTest_Run(void);

/** 1 ms 时基回调，仅供 interrupt.c 中的 SysTick 中断入口调用。 */
void PeripheralTest_1msTick(void);

#endif
