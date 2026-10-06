#include "pid.h"

static int32_t PID_Limit(int32_t value, int32_t minimum, int32_t maximum)
{
    if (value > maximum) return maximum;
    if (value < minimum) return minimum;
    return value;
}

void PID_Init(PIDController *pid,
              int32_t kp_x1000,
              int32_t ki_x1000,
              int32_t kd_x1000,
              int32_t output_min,
              int32_t output_max)
{
    if (pid == 0) return;

    pid->kp_x1000 = kp_x1000;
    pid->ki_x1000 = ki_x1000;
    pid->kd_x1000 = kd_x1000;
    pid->output_min = output_min;
    pid->output_max = output_max;
    pid->integral_min = output_min;
    pid->integral_max = output_max;
    PID_Reset(pid);
}

void PID_SetTunings(PIDController *pid,
                    int32_t kp_x1000,
                    int32_t ki_x1000,
                    int32_t kd_x1000)
{
    if (pid == 0) return;
    pid->kp_x1000 = kp_x1000;
    pid->ki_x1000 = ki_x1000;
    pid->kd_x1000 = kd_x1000;
}

void PID_SetIntegralLimits(PIDController *pid,
                           int32_t integral_min,
                           int32_t integral_max)
{
    if (pid == 0) return;
    pid->integral_min = integral_min;
    pid->integral_max = integral_max;
    pid->integral = PID_Limit(pid->integral,
                              integral_min, integral_max);
}

void PID_Reset(PIDController *pid)
{
    if (pid == 0) return;
    pid->integral = 0;
    pid->last_error = 0;
    pid->initialized = 0U;
}

int32_t PID_Update(PIDController *pid,
                   int32_t setpoint,
                   int32_t feedback)
{
    int32_t error;
    int32_t derivative;
    int32_t proportional;
    int32_t integral_step;
    int32_t old_integral;
    int32_t output;

    if (pid == 0) return 0;

    error = setpoint - feedback;
    derivative = (pid->initialized != 0U)
        ? (error - pid->last_error) : 0;
    pid->initialized = 1U;
    pid->last_error = error;

    proportional = (int32_t) (((int64_t) pid->kp_x1000 * error) /
                              1000LL);
    integral_step = (int32_t) (((int64_t) pid->ki_x1000 * error) /
                               1000LL);
    old_integral = pid->integral;
    pid->integral = PID_Limit(pid->integral + integral_step,
                              pid->integral_min,
                              pid->integral_max);

    output = proportional + pid->integral +
        (int32_t) (((int64_t) pid->kd_x1000 * derivative) / 1000LL);

    /* 输出饱和且误差仍在推动饱和时撤销本次积分，防止积分累积过量。 */
    if (((output > pid->output_max) && (error > 0)) ||
        ((output < pid->output_min) && (error < 0))) {
        pid->integral = old_integral;
    }

    return PID_Limit(output, pid->output_min, pid->output_max);
}
