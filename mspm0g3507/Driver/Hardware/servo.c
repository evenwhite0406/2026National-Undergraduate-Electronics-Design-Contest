#include "servo.h"

#include "ti_msp_dl_config.h"

static float g_servo_angle[2];
static uint16_t g_servo_pulse_us[2];

static bool Servo_ChannelToIndex(Servo_Channel channel, uint32_t *index)
{
    if (channel == SERVO_1) {
        *index = 0U;
        return true;
    }
    if (channel == SERVO_2) {
        *index = 1U;
        return true;
    }
    return false;
}

static float Servo_ClampAngle(float angle_deg)
{
    if (angle_deg < SERVO_MIN_ANGLE_DEG) {
        return SERVO_MIN_ANGLE_DEG;
    }
    if (angle_deg > SERVO_MAX_ANGLE_DEG) {
        return SERVO_MAX_ANGLE_DEG;
    }
    return angle_deg;
}

static uint16_t Servo_ClampPulseUs(uint16_t pulse_us)
{
    if (pulse_us < SERVO_MIN_PULSE_US) {
        return SERVO_MIN_PULSE_US;
    }
    if (pulse_us > SERVO_MAX_PULSE_US) {
        return SERVO_MAX_PULSE_US;
    }
    return pulse_us;
}

static uint16_t Servo_AngleToPulseUs(float angle_deg)
{
    float angle_span = SERVO_MAX_ANGLE_DEG - SERVO_MIN_ANGLE_DEG;
    float ratio = (angle_deg - SERVO_MIN_ANGLE_DEG) / angle_span;

    if (SERVO_DIRECTION_NORMAL == 0U) {
        ratio = 1.0f - ratio;
    }

    return (uint16_t) (SERVO_MIN_PULSE_US +
        (uint16_t) (ratio * (float) (SERVO_MAX_PULSE_US - SERVO_MIN_PULSE_US) + 0.5f));
}

static void Servo_WritePulse(uint32_t index, uint16_t pulse_us)
{
    uint32_t load_value = DL_Timer_getLoadValue(Servo_INST);
    uint32_t period_ticks = load_value + 1U;
    uint32_t compare_value =
        (((uint64_t) pulse_us * period_ticks) + (SERVO_PWM_PERIOD_US / 2U)) /
        SERVO_PWM_PERIOD_US;

    if (compare_value >= period_ticks) {
        compare_value = load_value;
    }
    if (SERVO_COMPARE_INVERTED != 0U) {
        compare_value = period_ticks - compare_value;
    }

    DL_Timer_setCaptureCompareValue(Servo_INST, compare_value, index);
}

bool Servo_IsConfigured(void)
{
    return true;
}

void Servo_SetChannelPulseUs(Servo_Channel channel, uint16_t pulse_us)
{
    uint32_t index;

    if (!Servo_ChannelToIndex(channel, &index)) {
        return;
    }

    pulse_us = Servo_ClampPulseUs(pulse_us);
    g_servo_pulse_us[index] = pulse_us;
    Servo_WritePulse(index, pulse_us);
}

void Servo_SetPulseUs(uint16_t pulse_us)
{
    Servo_SetChannelPulseUs(SERVO_1, pulse_us);
}

void Servo_SetChannelAngle(Servo_Channel channel, float angle_deg)
{
    uint32_t index;

    if (!Servo_ChannelToIndex(channel, &index)) {
        return;
    }

    angle_deg = Servo_ClampAngle(angle_deg);
    g_servo_angle[index] = angle_deg;
    Servo_SetChannelPulseUs(channel, Servo_AngleToPulseUs(angle_deg));
}

void Servo_SetAngle(float angle_deg)
{
    Servo_SetChannelAngle(SERVO_1, angle_deg);
}

void Servo_SetAngles(float servo1_deg, float servo2_deg)
{
    Servo_SetChannelAngle(SERVO_1, servo1_deg);
    Servo_SetChannelAngle(SERVO_2, servo2_deg);
}

void Servo_Init(void)
{
    Servo_SetAngles(SERVO_DEFAULT_ANGLE_DEG, SERVO_DEFAULT_ANGLE_DEG);
    DL_Timer_startCounter(Servo_INST);
}

float Servo_GetAngle(void)
{
    return g_servo_angle[0];
}

uint16_t Servo_GetPulseUs(void)
{
    return g_servo_pulse_us[0];
}

float Servo_GetChannelAngle(Servo_Channel channel)
{
    uint32_t index;

    return Servo_ChannelToIndex(channel, &index) ? g_servo_angle[index] : 0.0f;
}

uint16_t Servo_GetChannelPulseUs(Servo_Channel channel)
{
    uint32_t index;

    return Servo_ChannelToIndex(channel, &index) ? g_servo_pulse_us[index] : 0U;
}
