#ifndef PID_H
#define PID_H

#include <stdint.h>

/* 调用：PID_Init一次；固定周期调用PID_Update；切换目标/模式时PID_Reset。 */

/* PID增益采用千分制定点数，例如1500表示1.5。 */
typedef struct {
    int32_t kp_x1000;
    int32_t ki_x1000;
    int32_t kd_x1000;
    int32_t integral;
    int32_t last_error;
    int32_t output_min;
    int32_t output_max;
    int32_t integral_min;
    int32_t integral_max;
    uint8_t initialized;
} PIDController;

void PID_Init(PIDController *pid,
              int32_t kp_x1000,
              int32_t ki_x1000,
              int32_t kd_x1000,
              int32_t output_min,
              int32_t output_max);

void PID_SetTunings(PIDController *pid,
                    int32_t kp_x1000,
                    int32_t ki_x1000,
                    int32_t kd_x1000);

void PID_SetIntegralLimits(PIDController *pid,
                           int32_t integral_min,
                           int32_t integral_max);

void PID_Reset(PIDController *pid);

/** 固定周期离散PID，返回已经限幅的控制输出。 */
int32_t PID_Update(PIDController *pid,
                   int32_t setpoint,
                   int32_t feedback);

#endif
