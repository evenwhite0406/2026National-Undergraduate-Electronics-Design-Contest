#include "competition_task.h"

#include "bluetooth_hmi.h"
#include "k230_link.h"
#include "line_follow.h"
#include "vehicle_tuning.h"
#include "../Hardware/encoder.h"
#include "../Hardware/motor.h"
#include "../Hardware/oled.h"
#include "../PeripheralTest/button.h"
#include "../PeripheralTest/gyro_bmi088.h"

#include <limits.h>
#include <stdint.h>
#include <stdio.h>

#define COMPETITION_OLED_PERIOD_MS    (100U)
#define COMPETITION_TELEMETRY_PERIOD_MS (100U)
#define COMPETITION_START_LOG_PERIOD_MS (20U)

extern volatile uint32_t g_ms;

typedef enum {
    COMPETITION_MODE_NONE = 0,
    COMPETITION_MODE_TASK1_VIDEO,
    COMPETITION_MODE_TASK2_LAP_STOP_A,
    COMPETITION_MODE_TASK3_BALL_STATIC,
    COMPETITION_MODE_TASK4_A_TO_B,
    COMPETITION_MODE_TASK5_LAP_CENTER,
    COMPETITION_MODE_TASK6_LAP_TARGET,
    COMPETITION_MODE_TASK7_OTHER,
    COMPETITION_MODE_TASK8_LINE_TUNE
} CompetitionMode;

typedef enum {
    COMPETITION_PHASE_IDLE = 0,
    COMPETITION_PHASE_RUNNING,
    COMPETITION_PHASE_WAIT_LEAVE_A,
    COMPETITION_PHASE_RUNNING_LAP,
    COMPETITION_PHASE_PASS_A_EXTRA,
    COMPETITION_PHASE_TASK4_STOPPING,
    COMPETITION_PHASE_TASK56_STOPPING,
    COMPETITION_PHASE_TASK2_IMU_CALIBRATING,
    COMPETITION_PHASE_TASK8_IMU_CALIBRATING,
    COMPETITION_PHASE_K230_PREPARING,
    COMPETITION_PHASE_FINISHED
} CompetitionPhase;

typedef enum {
    COMPETITION_STOP_NONE = 0,
    COMPETITION_STOP_YAW_LAP,
    COMPETITION_STOP_LINE_LOST,
    COMPETITION_STOP_IMU_ERROR
} CompetitionStopReason;

typedef struct {
    uint8_t active;
    uint8_t task_number;
    uint8_t sequence;
    uint8_t link_started;
    uint8_t link_enabled;
    uint32_t start_ms;
    uint32_t last_sample_ms;
    uint32_t last_log_ms;
    uint32_t ramp_ms;
    int16_t cruise_speed;
    int16_t initial_measured_speed;
    int16_t last_measured_speed;
    int16_t measured_accel;
    int16_t imu_forward_zero_mg;
    uint8_t ball_baseline_valid;
    int16_t ball_baseline_x10_mm;
    int16_t ball_delta_min_x10_mm;
    int16_t ball_delta_max_x10_mm;
    int16_t ball_delta_last_x10_mm;
} CompetitionStartProfile;

static CompetitionMode s_mode;
static CompetitionPhase s_phase;
static uint32_t s_start_ms;
static uint32_t s_finish_ms;
static uint32_t s_result_elapsed_ms;
static uint32_t s_last_oled_ms;
static uint32_t s_last_telemetry_ms;
static int32_t s_encoder_left_start;
static int32_t s_encoder_right_start;
static uint32_t s_task2_distance_mm;
static int16_t s_task2_speed_limit;
static int16_t s_task2_last_yaw_x10;
static int32_t s_task2_accum_yaw_x10;
static uint8_t s_task2_yaw_valid;
static uint32_t s_task4_distance_mm;
static CompetitionStopReason s_stop_reason;
static int s_imu_status;
static uint8_t s_arc_log_seeded;
static uint8_t s_arc_log_last_raw7;
static uint8_t s_arc_log_last_filtered7;
static uint8_t s_arc_log_raw7_toggles;
static uint8_t s_arc_log_filtered7_toggles;
static int16_t s_arc_log_error_min;
static int16_t s_arc_log_error_max;
static int16_t s_arc_log_correction_min;
static int16_t s_arc_log_correction_max;
static CompetitionStartProfile s_start_profile;
static uint8_t s_start_profile_sequence;

static int32_t CompetitionTask_Abs32(int32_t value);
static void CompetitionTask_DrawElapsedTime(uint32_t elapsed_ms);
static void CompetitionTask_RecordFinishTime(uint32_t now_ms);
static void CompetitionTask_EndStartProfile(void);

static int16_t CompetitionTask_LimitInt16(int32_t value,
                                         int16_t minimum,
                                         int16_t maximum)
{
    if (value < (int32_t) minimum) {
        return minimum;
    }
    if (value > (int32_t) maximum) {
        return maximum;
    }
    return (int16_t) value;
}

static int16_t CompetitionTask_GetSelectedImuAccel(
    const BMI088_GyroData *imu)
{
    int16_t value;

    if (imu == NULL) {
        return 0;
    }

    value = (START_IMU_FORWARD_AXIS == 1U)
        ? imu->accel_mg_y : imu->accel_mg_x;
    return (START_IMU_FORWARD_INVERT != 0U)
        ? (int16_t) (-value) : value;
}

static void CompetitionTask_EndStartProfile(void)
{
    if (s_start_profile.link_started != 0U) {
        (void) K230Link_SendFeedforwardSample(
            s_start_profile.sequence, 0, 0, 0);
        (void) K230Link_SendFeedforwardEnd(s_start_profile.sequence);
    }

    s_start_profile.active = 0U;
    s_start_profile.link_started = 0U;
}

static void CompetitionTask_BeginStartProfile(uint8_t task_number,
                                              int16_t cruise_speed,
                                              uint32_t now_ms)
{
    const BMI088_GyroData *imu = BMI088_Gyro_GetData();
    int16_t measured_speed = (int16_t) (
        (Motor_GetFeedbackSpeed(MOTOR_LEFT) +
         Motor_GetFeedbackSpeed(MOTOR_RIGHT)) / 2);

    CompetitionTask_EndStartProfile();

    s_start_profile_sequence++;
    if (s_start_profile_sequence == 0U) {
        s_start_profile_sequence = 1U;
    }

    s_start_profile.active = 1U;
    s_start_profile.task_number = task_number;
    s_start_profile.sequence = s_start_profile_sequence;
    s_start_profile.start_ms = now_ms;
    s_start_profile.last_sample_ms =
        now_ms - COMPETITION_START_LOG_PERIOD_MS;
    s_start_profile.last_log_ms =
        now_ms - COMPETITION_TELEMETRY_PERIOD_MS;
    s_start_profile.ramp_ms =
        ((task_number >= 4U) && (task_number <= 6U))
        ? START_BALL_S_CURVE_MS : START_S_CURVE_MS;
    s_start_profile.cruise_speed = cruise_speed;
    s_start_profile.initial_measured_speed = measured_speed;
    s_start_profile.last_measured_speed = measured_speed;
    s_start_profile.measured_accel = 0;
    s_start_profile.imu_forward_zero_mg =
        CompetitionTask_GetSelectedImuAccel(imu);
    s_start_profile.ball_baseline_valid = 0U;
    s_start_profile.ball_baseline_x10_mm = 0;
    s_start_profile.ball_delta_min_x10_mm = 0;
    s_start_profile.ball_delta_max_x10_mm = 0;
    s_start_profile.ball_delta_last_x10_mm = 0;
    s_start_profile.link_enabled =
        ((START_FF_LINK_ENABLE != 0U) &&
         (task_number >= 4U) && (task_number <= 6U)) ? 1U : 0U;
    s_start_profile.link_started = s_start_profile.link_enabled;

    LineFollow_SetCruiseSpeedLimit(0);

    if (s_start_profile.link_started != 0U) {
        (void) K230Link_SendFeedforwardConfig(
            s_start_profile.sequence,
            START_FF_LEAD_MS,
            s_start_profile.ramp_ms,
            START_FF_GAIN_X100,
            START_FF_DAMP_X100,
            START_FF_MAX_X10,
            START_FF_SIGN);
    }
}

static void CompetitionTask_UpdateStartProfile(int16_t requested_speed,
                                               uint32_t now_ms)
{
    uint32_t elapsed_ms;
    uint32_t ramp_elapsed_ms;
    uint32_t ff_ramp_elapsed_ms;
    uint32_t ramp_ms;
    uint32_t profile_end_ms;
    uint32_t sample_dt_ms;
    uint32_t x1000;
    uint32_t ff_x1000;
    int32_t shape_x1000;
    int32_t reference_speed;
    int32_t reference_accel;
    int32_t ff_reference_accel;
    int32_t measured_accel_raw;
    int32_t accel_error;
    int32_t ff_x10;
    int16_t measured_speed;
    int16_t imu_forward_mg;
    int16_t speed_limit;
    int16_t ball_position_x10_mm = 0;
    int16_t ball_delta_x10_mm = 0;
    uint8_t ball_valid;
    const BMI088_GyroData *imu;
    char telemetry[224];

    if (s_start_profile.active == 0U) {
        LineFollow_SetCruiseSpeedLimit(requested_speed);
        return;
    }

    if (s_imu_status == BMI088_GYRO_OK) {
        s_imu_status = BMI088_Gyro_Service(now_ms);
    }

    elapsed_ms = (uint32_t) (now_ms - s_start_profile.start_ms);
    ramp_ms = (s_start_profile.ramp_ms < 100U)
        ? 100U : s_start_profile.ramp_ms;

    if (elapsed_ms <= START_FF_LEAD_MS) {
        ramp_elapsed_ms = 0U;
    } else {
        ramp_elapsed_ms = elapsed_ms - START_FF_LEAD_MS;
        if (ramp_elapsed_ms > ramp_ms) {
            ramp_elapsed_ms = ramp_ms;
        }
    }

    x1000 = (uint32_t) (((uint64_t) ramp_elapsed_ms * 1000ULL) /
                        (uint64_t) ramp_ms);
    /*
     * FF_LEAD是真正的执行器相位提前量：车轮速度仍从lead_ms之后开始，
     * 前馈则使用车轮轨迹在lead_ms之后将遇到的加速度。这样K230摆杆
     * 可以先建立小角度，而不是仅让整车和前馈一起延迟。
     */
    ff_ramp_elapsed_ms = elapsed_ms;
    if (ff_ramp_elapsed_ms > ramp_ms) {
        ff_ramp_elapsed_ms = ramp_ms;
    }
    ff_x1000 = (uint32_t) (
        ((uint64_t) ff_ramp_elapsed_ms * 1000ULL) /
        (uint64_t) ramp_ms);
    shape_x1000 = (int32_t) (
        ((3ULL * x1000 * x1000) / 1000ULL) -
        ((2ULL * x1000 * x1000 * x1000) / 1000000ULL));
    reference_speed =
        ((int32_t) s_start_profile.cruise_speed * shape_x1000 + 500L) /
        1000L;
    reference_accel = (int32_t) (
        ((int64_t) s_start_profile.cruise_speed * 6LL *
         (int64_t) x1000 * (int64_t) (1000U - x1000)) /
        ((int64_t) ramp_ms * 1000LL));
    ff_reference_accel = (int32_t) (
        ((int64_t) s_start_profile.cruise_speed * 6LL *
         (int64_t) ff_x1000 * (int64_t) (1000U - ff_x1000)) /
        ((int64_t) ramp_ms * 1000LL));

    speed_limit = CompetitionTask_LimitInt16(reference_speed, 0,
                                             LINE_MAX_SPEED);
    if (speed_limit > requested_speed) {
        speed_limit = requested_speed;
    }
    LineFollow_SetCruiseSpeedLimit(speed_limit);

    sample_dt_ms = (uint32_t) (now_ms - s_start_profile.last_sample_ms);
    if (sample_dt_ms < COMPETITION_START_LOG_PERIOD_MS) {
        return;
    }

    measured_speed = (int16_t) (
        (Motor_GetFeedbackSpeed(MOTOR_LEFT) +
         Motor_GetFeedbackSpeed(MOTOR_RIGHT)) / 2);
    measured_accel_raw =
        ((int32_t) (measured_speed -
                    s_start_profile.last_measured_speed) * 1000L) /
        (int32_t) sample_dt_ms;
    measured_accel_raw = CompetitionTask_LimitInt16(
        measured_accel_raw, -2000, 2000);
    s_start_profile.measured_accel = (int16_t) (
        ((int32_t) s_start_profile.measured_accel * 3L +
         measured_accel_raw) / 4L);
    s_start_profile.last_measured_speed = measured_speed;
    s_start_profile.last_sample_ms = now_ms;

    accel_error = reference_accel - s_start_profile.measured_accel;
    ff_x10 = ((START_FF_GAIN_X100 * ff_reference_accel) +
              (START_FF_DAMP_X100 * accel_error)) / 10L;
    if (START_FF_SIGN < 0) {
        ff_x10 = -ff_x10;
    }
    ff_x10 = CompetitionTask_LimitInt16(
        ff_x10, (int16_t) (-START_FF_MAX_X10), START_FF_MAX_X10);

    if (s_start_profile.link_started != 0U) {
        (void) K230Link_SendFeedforwardSample(
            s_start_profile.sequence,
            (int16_t) ff_x10,
            CompetitionTask_LimitInt16(ff_reference_accel, -2000, 2000),
            s_start_profile.measured_accel);
    }

    ball_valid = K230Link_HasBallPosition();
    if (ball_valid != 0U) {
        ball_position_x10_mm = K230Link_GetBallPositionX10Mm();
        if (s_start_profile.ball_baseline_valid == 0U) {
            s_start_profile.ball_baseline_valid = 1U;
            s_start_profile.ball_baseline_x10_mm = ball_position_x10_mm;
            s_start_profile.ball_delta_min_x10_mm = 0;
            s_start_profile.ball_delta_max_x10_mm = 0;
            s_start_profile.ball_delta_last_x10_mm = 0;
        } else {
            ball_delta_x10_mm = CompetitionTask_LimitInt16(
                (int32_t) ball_position_x10_mm -
                (int32_t) s_start_profile.ball_baseline_x10_mm,
                INT16_MIN, INT16_MAX);
            s_start_profile.ball_delta_last_x10_mm = ball_delta_x10_mm;
            if (ball_delta_x10_mm <
                s_start_profile.ball_delta_min_x10_mm) {
                s_start_profile.ball_delta_min_x10_mm =
                    ball_delta_x10_mm;
            }
            if (ball_delta_x10_mm >
                s_start_profile.ball_delta_max_x10_mm) {
                s_start_profile.ball_delta_max_x10_mm =
                    ball_delta_x10_mm;
            }
        }
    }

    imu = BMI088_Gyro_GetData();
    imu_forward_mg = (int16_t) (
        CompetitionTask_GetSelectedImuAccel(imu) -
        s_start_profile.imu_forward_zero_mg);

    if ((START_FF_LOG_MS != 0U) &&
        (elapsed_ms <= START_FF_LOG_MS) &&
        ((uint32_t) (now_ms - s_start_profile.last_log_ms) >=
         COMPETITION_TELEMETRY_PERIOD_MS)) {
        s_start_profile.last_log_ms = now_ms;
        (void) snprintf(telemetry, sizeof(telemetry),
            "[FF,T=%u,Q=%u,EN=%u,MS=%lu,VR=%ld,VM=%d,AR=%ld,AP=%ld,AM=%d,F=%ld,BV=%u,B=%d,D=%d,U=%d,AX=%d,AY=%d,AZ=%d]\r\n",
            (unsigned int) s_start_profile.task_number,
            (unsigned int) s_start_profile.sequence,
            (unsigned int) s_start_profile.link_enabled,
            (unsigned long) elapsed_ms,
            (long) reference_speed,
            (int) measured_speed,
            (long) reference_accel,
            (long) ff_reference_accel,
            (int) s_start_profile.measured_accel,
            (long) ff_x10,
            (unsigned int) ball_valid,
            (int) ball_position_x10_mm,
            (int) ball_delta_x10_mm,
            (int) imu_forward_mg,
            (imu != NULL) ? (int) imu->accel_mg_x : 0,
            (imu != NULL) ? (int) imu->accel_mg_y : 0,
            (imu != NULL) ? (int) imu->accel_mg_z : 0);
        Param_SendTelemetry(telemetry);
    }

    profile_end_ms = START_FF_LEAD_MS + ramp_ms;
    if ((elapsed_ms >= profile_end_ms) &&
        (s_start_profile.link_started != 0U)) {
        (void) K230Link_SendFeedforwardSample(
            s_start_profile.sequence, 0, 0,
            s_start_profile.measured_accel);
        (void) K230Link_SendFeedforwardEnd(s_start_profile.sequence);
        s_start_profile.link_started = 0U;
    }

    if ((elapsed_ms >= profile_end_ms) &&
        ((START_FF_LOG_MS == 0U) || (elapsed_ms >= START_FF_LOG_MS))) {
        if (START_FF_LOG_MS != 0U) {
            (void) snprintf(telemetry, sizeof(telemetry),
                "[FFS,T=%u,Q=%u,EN=%u,V0=%d,BV=%u,B0=%d,DN=%d,DP=%d,DE=%d]\r\n",
                (unsigned int) s_start_profile.task_number,
                (unsigned int) s_start_profile.sequence,
                (unsigned int) s_start_profile.link_enabled,
                (int) s_start_profile.initial_measured_speed,
                (unsigned int) s_start_profile.ball_baseline_valid,
                (int) s_start_profile.ball_baseline_x10_mm,
                (int) s_start_profile.ball_delta_min_x10_mm,
                (int) s_start_profile.ball_delta_max_x10_mm,
                (int) s_start_profile.ball_delta_last_x10_mm);
            Param_SendTelemetry(telemetry);
        }
        s_start_profile.active = 0U;
        LineFollow_SetCruiseSpeedLimit(requested_speed);
    }
}

static void CompetitionTask_SafeStop(void)
{
    CompetitionTask_EndStartProfile();
    LineFollow_Stop();
    Motor_SetSpeeds(0, 0);
    Motor_Off();
}

static void CompetitionTask_DrawPlaceholder(void)
{
    const char *task_name = "TASK ?";

    switch (s_mode) {
        case COMPETITION_MODE_TASK1_VIDEO:      task_name = "TASK 1"; break;
        case COMPETITION_MODE_TASK2_LAP_STOP_A: task_name = "TASK 2"; break;
        case COMPETITION_MODE_TASK3_BALL_STATIC:task_name = "TASK 3"; break;
        case COMPETITION_MODE_TASK4_A_TO_B:     task_name = "TASK 4"; break;
        case COMPETITION_MODE_TASK5_LAP_CENTER: task_name = "TASK 5"; break;
        case COMPETITION_MODE_TASK6_LAP_TARGET: task_name = "TASK 6"; break;
        case COMPETITION_MODE_TASK7_OTHER:      task_name = "TASK 7"; break;
        case COMPETITION_MODE_TASK8_LINE_TUNE:  task_name = "TASK 8"; break;
        default: break;
    }

    TaskManager_ShowLine(1U, task_name);
    TaskManager_ShowLine(2U, "NOT IMPLEMENTED");
    TaskManager_ShowLine(3U, "CAR STOP");
    TaskManager_ShowLine(4U, "K4 EXIT");
    TaskManager_ShowLine(5U, "");
    TaskManager_ShowLine(6U, "");
    TaskManager_ShowLine(7U, "");
    TaskManager_ShowLine(8U, "");
}

static uint8_t CompetitionTask_OledDue(uint32_t now_ms)
{
    if ((s_last_oled_ms != 0U) &&
        ((uint32_t) (now_ms - s_last_oled_ms) <
         COMPETITION_OLED_PERIOD_MS)) {
        return 0U;
    }

    s_last_oled_ms = now_ms;
    return 1U;
}

static void CompetitionTask_SendLineTelemetry(
    uint8_t task_number,
    const LineFollowData *data,
    const BMI088_GyroData *imu,
    uint32_t now_ms)
{
    char telemetry[128];
    if ((data == NULL) || (imu == NULL) ||
        ((s_last_telemetry_ms != 0U) &&
         ((uint32_t) (now_ms - s_last_telemetry_ms) <
          COMPETITION_TELEMETRY_PERIOD_MS))) {
        return;
    }

    s_last_telemetry_ms = now_ms;
    (void) snprintf(telemetry, sizeof(telemetry),
        "[T%u,A=%02X,F=%02X,E=%d,C=%d,B=%d,L=%d,R=%d,FL=%d,FR=%d,S=%u,G=%d,Y=%d,I=%d]\r\n",
        (unsigned int) task_number,
        (unsigned int) data->raw_mask,
        (unsigned int) data->filtered_mask,
        (int) data->error_x10,
        (int) data->current_correction,
        (int) data->target_base_speed,
        (int) data->left_speed,
        (int) data->right_speed,
        (int) Motor_GetFeedbackSpeed(MOTOR_LEFT),
        (int) Motor_GetFeedbackSpeed(MOTOR_RIGHT),
        (unsigned int) data->state,
        (int) imu->yaw_rate_x10,
        (int) imu->yaw_x10,
        s_imu_status);
    Param_SendTelemetry(telemetry);
}

static void CompetitionTask_ResetArcTelemetry(void)
{
    s_arc_log_seeded = 0U;
    s_arc_log_last_raw7 = 0U;
    s_arc_log_last_filtered7 = 0U;
    s_arc_log_raw7_toggles = 0U;
    s_arc_log_filtered7_toggles = 0U;
    s_arc_log_error_min = 0;
    s_arc_log_error_max = 0;
    s_arc_log_correction_min = 0;
    s_arc_log_correction_max = 0;
}

static void CompetitionTask_SendTask56ArcTelemetry(
    uint8_t task_number,
    const LineFollowData *data,
    const BMI088_GyroData *imu,
    uint32_t now_ms)
{
    char telemetry[192];
    uint8_t raw7;
    uint8_t filtered7;

    if ((data == NULL) || (imu == NULL)) {
        return;
    }

    raw7 = ((data->raw_mask & 0x40U) != 0U) ? 1U : 0U;
    filtered7 = ((data->filtered_mask & 0x40U) != 0U) ? 1U : 0U;

    if (s_arc_log_seeded == 0U) {
        s_arc_log_seeded = 1U;
        s_arc_log_last_raw7 = raw7;
        s_arc_log_last_filtered7 = filtered7;
        s_arc_log_error_min = data->error_x10;
        s_arc_log_error_max = data->error_x10;
        s_arc_log_correction_min = data->current_correction;
        s_arc_log_correction_max = data->current_correction;
    } else {
        if (raw7 != s_arc_log_last_raw7) {
            if (s_arc_log_raw7_toggles < UINT8_MAX) {
                s_arc_log_raw7_toggles++;
            }
            s_arc_log_last_raw7 = raw7;
        }
        if (filtered7 != s_arc_log_last_filtered7) {
            if (s_arc_log_filtered7_toggles < UINT8_MAX) {
                s_arc_log_filtered7_toggles++;
            }
            s_arc_log_last_filtered7 = filtered7;
        }
        if (data->error_x10 < s_arc_log_error_min) {
            s_arc_log_error_min = data->error_x10;
        }
        if (data->error_x10 > s_arc_log_error_max) {
            s_arc_log_error_max = data->error_x10;
        }
        if (data->current_correction < s_arc_log_correction_min) {
            s_arc_log_correction_min = data->current_correction;
        }
        if (data->current_correction > s_arc_log_correction_max) {
            s_arc_log_correction_max = data->current_correction;
        }
    }

    if ((s_last_telemetry_ms != 0U) &&
        ((uint32_t) (now_ms - s_last_telemetry_ms) <
         COMPETITION_TELEMETRY_PERIOD_MS)) {
        return;
    }

    s_last_telemetry_ms = now_ms;
    (void) snprintf(telemetry, sizeof(telemetry),
        "[T%u,7R=%u,7F=%u,RT=%u,FT=%u,A=%02X,F=%02X,RE=%d,E=%d,ER=%d:%d,P=%d,D=%d,TC=%d,C=%d,CR=%d:%d,B=%d,L=%d,R=%d,FL=%d,FR=%d,Y=%d]\r\n",
        (unsigned int) task_number,
        (unsigned int) raw7,
        (unsigned int) filtered7,
        (unsigned int) s_arc_log_raw7_toggles,
        (unsigned int) s_arc_log_filtered7_toggles,
        (unsigned int) data->raw_mask,
        (unsigned int) data->filtered_mask,
        (int) data->raw_error_x10,
        (int) data->error_x10,
        (int) s_arc_log_error_min,
        (int) s_arc_log_error_max,
        (int) data->p_correction,
        (int) data->d_correction,
        (int) data->target_correction,
        (int) data->current_correction,
        (int) s_arc_log_correction_min,
        (int) s_arc_log_correction_max,
        (int) data->current_base_speed,
        (int) data->left_speed,
        (int) data->right_speed,
        (int) Motor_GetFeedbackSpeed(MOTOR_LEFT),
        (int) Motor_GetFeedbackSpeed(MOTOR_RIGHT),
        (int) imu->yaw_x10);
    Param_SendTelemetry(telemetry);

    s_arc_log_raw7_toggles = 0U;
    s_arc_log_filtered7_toggles = 0U;
    s_arc_log_error_min = data->error_x10;
    s_arc_log_error_max = data->error_x10;
    s_arc_log_correction_min = data->current_correction;
    s_arc_log_correction_max = data->current_correction;
}

static void CompetitionTask_SendTask2YawTelemetry(uint32_t now_ms)
{
    const BMI088_GyroData *imu;
    char telemetry[96];
    int16_t yaw_x10 = 0;
    int32_t yaw_abs_x10;
    int32_t target_x10 = (int32_t) (TASK2_STOP_YAW_DEG * 10U);
    uint8_t angle_valid = 0U;

    if ((s_last_telemetry_ms != 0U) &&
        ((uint32_t) (now_ms - s_last_telemetry_ms) <
         COMPETITION_TELEMETRY_PERIOD_MS)) {
        return;
    }

    imu = BMI088_Gyro_GetData();
    if (imu != NULL) {
        yaw_x10 = imu->yaw_x10;
        angle_valid = imu->angle_valid;
    }

    if (target_x10 < 100L) {
        target_x10 = 100L;
    }
    yaw_abs_x10 = CompetitionTask_Abs32(s_task2_accum_yaw_x10);
    s_last_telemetry_ms = now_ms;

    (void) snprintf(telemetry, sizeof(telemetry),
        "[T2,Y10=%d,S10=%ld,A10=%ld,T10=%ld,V=%u,I=%d]\r\n",
        (int) yaw_x10,
        (long) s_task2_accum_yaw_x10,
        (long) yaw_abs_x10,
        (long) target_x10,
        (unsigned int) angle_valid,
        s_imu_status);
    Param_SendTelemetry(telemetry);
}

static void CompetitionTask_UpdateTask1(uint32_t now_ms)
{
    if (CompetitionTask_OledDue(now_ms) == 0U) {
        return;
    }

    TaskManager_ShowLine(1U, "T1 K230 VIDEO");
    TaskManager_ShowLine(2U, "CAR: STOP");
    TaskManager_ShowLine(3U, "VIDEO: EXTERNAL");
    TaskManager_ShowLine(4U, "K4 EXIT");
    TaskManager_ShowLine(5U, "");
    TaskManager_ShowLine(6U, "");
    TaskManager_ShowLine(7U, "");
    TaskManager_ShowLine(8U, "");
}

static void CompetitionTask_BeginTask3Run(uint32_t now_ms)
{
    CompetitionTask_SafeStop();
    s_start_ms = now_ms;
    s_finish_ms = 0U;
    s_result_elapsed_ms = 0U;
    s_last_oled_ms = 0U;
    s_last_telemetry_ms = 0U;
    s_phase = COMPETITION_PHASE_RUNNING;
}

static void CompetitionTask_UpdateTask3(uint32_t now_ms)
{
    if ((s_phase == COMPETITION_PHASE_RUNNING) &&
        (K230Link_ConsumeDone(3U) != 0U)) {
        CompetitionTask_RecordFinishTime(now_ms);
        CompetitionTask_SafeStop();
        s_phase = COMPETITION_PHASE_FINISHED;
        s_last_oled_ms = 0U;
        K230Link_RequestPrepare(4U, now_ms);
    } else {
        CompetitionTask_SafeStop();
    }

    if (CompetitionTask_OledDue(now_ms) == 0U) {
        return;
    }

    if (s_phase == COMPETITION_PHASE_FINISHED) {
        TaskManager_ShowLine(1U, "T3 FINISHED");
        CompetitionTask_DrawElapsedTime(s_result_elapsed_ms);
        TaskManager_ShowLine(3U, "BALL TASK DONE");
        TaskManager_ShowLine(4U, "K4 EXIT");
    } else {
        TaskManager_ShowLine(1U, "T3 BALL +5/-5");
        CompetitionTask_DrawElapsedTime(now_ms - s_start_ms);
        TaskManager_ShowLine(3U, "CAR:STOP WAIT D3");
        TaskManager_ShowLine(4U, "K4 CANCEL");
    }
    TaskManager_ShowLine(5U, K230Link_GetStatusText(now_ms));
    TaskManager_ShowLine(6U, "");
    TaskManager_ShowLine(7U, "");
    TaskManager_ShowLine(8U, "");
}

static void CompetitionTask_UpdateTask7(uint32_t now_ms)
{
    if (CompetitionTask_OledDue(now_ms) == 0U) {
        return;
    }

    TaskManager_ShowLine(1U, "T7 OTHER TEST");
    TaskManager_ShowLine(2U, "CAR: STOP");
    TaskManager_ShowLine(3U, "SYSTEM READY");
    TaskManager_ShowLine(4U, "K4 EXIT");
    TaskManager_ShowLine(5U, "");
    TaskManager_ShowLine(6U, "");
    TaskManager_ShowLine(7U, "");
    TaskManager_ShowLine(8U, "");
}

static void CompetitionTask_FormatTime(char *buffer,
                                       uint32_t buffer_size,
                                       uint32_t elapsed_ms)
{
    unsigned long seconds = (unsigned long) (elapsed_ms / 1000U);
    unsigned long hundredths =
        (unsigned long) ((elapsed_ms % 1000U) / 10U);

    (void) snprintf(buffer, (size_t) buffer_size, "TIME:%lu.%02lus",
                    seconds, hundredths);
}

static void CompetitionTask_DrawElapsedTime(uint32_t elapsed_ms)
{
    char text[TASK_MANAGER_TEXT_COLUMNS + 1U];

    CompetitionTask_FormatTime(text, (uint32_t) sizeof(text), elapsed_ms);
    TaskManager_ShowLine(2U, text);
}

static void CompetitionTask_RecordFinishTime(uint32_t now_ms)
{
    if (s_finish_ms == 0U) {
        s_finish_ms = now_ms;
        s_result_elapsed_ms = now_ms - s_start_ms;
    }
}

static int32_t CompetitionTask_ClampCountDelta(int32_t current,
                                               int32_t start)
{
    int64_t delta = (int64_t) current - (int64_t) start;

    if (delta > (int64_t) INT32_MAX) {
        return INT32_MAX;
    }
    if (delta < (int64_t) INT32_MIN) {
        return INT32_MIN;
    }
    return (int32_t) delta;
}

static int64_t CompetitionTask_Abs64(int64_t value)
{
    return (value < 0LL) ? -value : value;
}

static uint32_t CompetitionTask_GetTravelDistanceMm(void)
{
    EncoderData left = Encoder_GetData(ENCODER_1);
    EncoderData right = Encoder_GetData(ENCODER_2);
    int32_t left_delta = CompetitionTask_ClampCountDelta(
        left.count, s_encoder_left_start);
    int32_t right_delta = CompetitionTask_ClampCountDelta(
        right.count, s_encoder_right_start);
    int64_t left_mm = CompetitionTask_Abs64(
        (int64_t) Motor_CountsToDistanceMm(left_delta));
    int64_t right_mm = CompetitionTask_Abs64(
        (int64_t) Motor_CountsToDistanceMm(right_delta));
    int64_t average_mm = (left_mm + right_mm) / 2LL;

    if (average_mm > (int64_t) UINT32_MAX) {
        return UINT32_MAX;
    }
    return (uint32_t) average_mm;
}

static int16_t CompetitionTask_GetTask2PositionSpeedLimit(
    uint32_t distance_mm)
{
    uint32_t target_mm = TRACK_LAP_DISTANCE_CM * 10U;
    uint32_t brake_start_mm = TASK2_BRAKE_START_CM * 10U;
    uint32_t remaining_mm;
    int32_t speed_limit;
    int16_t cruise_speed = TASK2_CRUISE_SPEED;

    if (cruise_speed > LINE_MAX_SPEED) {
        cruise_speed = LINE_MAX_SPEED;
    }
    if (cruise_speed < CRAWL_SPEED) {
        cruise_speed = CRAWL_SPEED;
    }

    if ((distance_mm < brake_start_mm) ||
        (TASK2_POSITION_KP_X100 <= 0L)) {
        return cruise_speed;
    }

    remaining_mm = (distance_mm < target_mm)
        ? (target_mm - distance_mm)
        : 0U;

    /*
     * Task-2 position P loop:
     * speed = remaining_mm * Kp / 1000.
     * Kp=140 means 1.40 speed units per remaining centimetre.
     * It only limits the common base speed; line-follow differential and
     * the left/right wheel speed loops remain active.
     */
    speed_limit = (int32_t) (
        ((int64_t) remaining_mm * TASK2_POSITION_KP_X100) / 1000LL);

    if (speed_limit < (int32_t) CRAWL_SPEED) {
        speed_limit = CRAWL_SPEED;
    }
    if (speed_limit > (int32_t) cruise_speed) {
        speed_limit = cruise_speed;
    }

    return (int16_t) speed_limit;
}

static int32_t CompetitionTask_Abs32(int32_t value)
{
    if (value == INT32_MIN) {
        return INT32_MAX;
    }
    return (value < 0L) ? -value : value;
}

static int16_t CompetitionTask_WrapYawDeltaX10(int32_t delta_x10)
{
    while (delta_x10 > 1800L) {
        delta_x10 -= 3600L;
    }
    while (delta_x10 < -1800L) {
        delta_x10 += 3600L;
    }
    return (int16_t) delta_x10;
}

static uint8_t CompetitionTask_UpdateLapYaw(uint32_t now_ms,
                                            uint32_t stop_yaw_deg)
{
    const BMI088_GyroData *imu;
    int16_t current_yaw_x10;
    int16_t delta_x10;
    int32_t target_x10 = (int32_t) (stop_yaw_deg * 10U);

    s_imu_status = BMI088_Gyro_Service(now_ms);
    if (s_imu_status != BMI088_GYRO_OK) {
        return 0U;
    }

    imu = BMI088_Gyro_GetData();
    if ((imu == NULL) || (imu->angle_valid == 0U)) {
        return 0U;
    }

    current_yaw_x10 = imu->yaw_x10;
    if (s_task2_yaw_valid == 0U) {
        s_task2_last_yaw_x10 = current_yaw_x10;
        s_task2_yaw_valid = 1U;
        return 0U;
    }

    delta_x10 = CompetitionTask_WrapYawDeltaX10(
        (int32_t) current_yaw_x10 -
        (int32_t) s_task2_last_yaw_x10);
    s_task2_last_yaw_x10 = current_yaw_x10;
    s_task2_accum_yaw_x10 += (int32_t) delta_x10;

    if (target_x10 < 100L) {
        target_x10 = 100L;
    }

    return (CompetitionTask_Abs32(s_task2_accum_yaw_x10) >= target_x10)
        ? 1U
        : 0U;
}

static void CompetitionTask_UpdateTask8(uint32_t now_ms)
{
    LineFollowData data;
    const BMI088_GyroData *imu;
    char diagnostic[TASK_MANAGER_TEXT_COLUMNS + 1U];

    if (s_phase == COMPETITION_PHASE_TASK8_IMU_CALIBRATING) {
        CompetitionTask_SafeStop();
        s_imu_status = BMI088_Gyro_CalibrationService(now_ms);

        if (s_imu_status == BMI088_GYRO_OK) {
            LineFollow_StartLineReset();
            LineFollow_Start(now_ms);
            s_start_ms = now_ms;
            s_last_oled_ms = 0U;
            s_last_telemetry_ms = 0U;
            s_phase = COMPETITION_PHASE_RUNNING;
        } else if (s_imu_status < 0) {
            s_phase = COMPETITION_PHASE_FINISHED;
            s_last_oled_ms = 0U;
        }

        if (CompetitionTask_OledDue(now_ms) != 0U) {
            TaskManager_ShowLine(1U, "T8 IMU CAL");
            TaskManager_ShowLine(2U, "KEEP CAR STILL");
            TaskManager_ShowLine(3U, "CAR: STOP");
            TaskManager_ShowLine(4U, "K4 CANCEL");
        }
        return;
    }

    if (s_phase == COMPETITION_PHASE_FINISHED) {
        CompetitionTask_SafeStop();
        if (CompetitionTask_OledDue(now_ms) != 0U) {
            (void) snprintf(diagnostic, sizeof(diagnostic),
                "IMU ERR:%d", s_imu_status);
            TaskManager_ShowLine(1U, "T8 IMU ERROR");
            TaskManager_ShowLine(2U, diagnostic);
            TaskManager_ShowLine(3U, "CAR: STOP");
            TaskManager_ShowLine(4U, "K4 EXIT");
        }
        return;
    }

    if (s_phase == COMPETITION_PHASE_RUNNING) {
        LineFollow_Update(now_ms);
        (void) LineFollow_StartLineConsumeEvent();
        s_imu_status = BMI088_Gyro_Service(now_ms);
    }

    data = LineFollow_GetData();
    imu = BMI088_Gyro_GetData();
    if (data.state == LINE_FOLLOW_STATE_FAILSAFE) {
        CompetitionTask_SafeStop();
    }

    CompetitionTask_SendLineTelemetry(8U, &data, imu, now_ms);

    if (CompetitionTask_OledDue(now_ms) == 0U) {
        return;
    }

    TaskManager_ShowLine(1U, "T8 LINE TUNE");
    CompetitionTask_DrawElapsedTime(now_ms - s_start_ms);
    (void) snprintf(diagnostic, sizeof(diagnostic),
        "M:%02X L:%d R:%d", (unsigned int) data.filtered_mask,
        (int) data.left_speed, (int) data.right_speed);
    TaskManager_ShowLine(3U, diagnostic);
    (void) snprintf(diagnostic, sizeof(diagnostic),
        "G:%d Y:%d", (int) imu->yaw_rate_x10,
        (int) imu->yaw_x10);
    TaskManager_ShowLine(4U, diagnostic);
    TaskManager_ShowLine(5U, "K4 STOP");
    TaskManager_ShowLine(6U, "");
    TaskManager_ShowLine(7U, "");
    TaskManager_ShowLine(8U, "");
}

static void CompetitionTask_BeginTask2Run(uint32_t now_ms)
{
    EncoderData left = Encoder_GetData(ENCODER_1);
    EncoderData right = Encoder_GetData(ENCODER_2);

    BMI088_Gyro_ResetAngles();
    s_task2_last_yaw_x10 = 0;
    s_task2_accum_yaw_x10 = 0L;
    s_task2_yaw_valid = 0U;

    s_start_ms = now_ms;
    s_finish_ms = 0U;
    s_result_elapsed_ms = 0U;
    s_encoder_left_start = left.count;
    s_encoder_right_start = right.count;
    s_task2_distance_mm = 0U;
    s_task2_speed_limit = CompetitionTask_GetTask2PositionSpeedLimit(0U);

    /* Task 2 ignores every grayscale stop-line event; yaw ends the lap. */
    LineFollow_StartLineReset();
    LineFollow_Start(now_ms);
    LineFollow_SetBaseSpeedOverride(TASK2_CRUISE_SPEED);
    LineFollow_SetStartLineDetectionEnabled(0U);
    LineFollow_SetStartLineIgnoreMs(0U);
    CompetitionTask_BeginStartProfile(2U, s_task2_speed_limit, now_ms);
    s_phase = COMPETITION_PHASE_RUNNING_LAP;
}

static void CompetitionTask_UpdateTask2(uint32_t now_ms)
{
    uint32_t elapsed_ms;
    int32_t yaw_abs_x10;
    int32_t yaw_target_x10;
    LineFollowData line_data;
    char diagnostic[TASK_MANAGER_TEXT_COLUMNS + 1U];

    if (s_phase == COMPETITION_PHASE_TASK2_IMU_CALIBRATING) {
        CompetitionTask_SafeStop();
        s_imu_status = BMI088_Gyro_CalibrationService(now_ms);

        if (s_imu_status == BMI088_GYRO_OK) {
            CompetitionTask_BeginTask2Run(now_ms);
            s_last_oled_ms = 0U;
        } else if (s_imu_status < 0) {
            s_stop_reason = COMPETITION_STOP_IMU_ERROR;
            s_phase = COMPETITION_PHASE_FINISHED;
            s_last_oled_ms = 0U;
        }

        if (CompetitionTask_OledDue(now_ms) != 0U) {
            TaskManager_ShowLine(1U, "T2 IMU CAL");
            TaskManager_ShowLine(2U, "KEEP CAR STILL");
            TaskManager_ShowLine(3U, "CAR: STOP");
            TaskManager_ShowLine(4U, "K4 CANCEL");
            TaskManager_ShowLine(5U, "");
            TaskManager_ShowLine(6U, "");
            TaskManager_ShowLine(7U, "");
            TaskManager_ShowLine(8U, "");
        }
        return;
    }

    if (s_phase == COMPETITION_PHASE_RUNNING_LAP) {
        s_task2_distance_mm = CompetitionTask_GetTravelDistanceMm();
        s_task2_speed_limit =
            CompetitionTask_GetTask2PositionSpeedLimit(
                s_task2_distance_mm);
        LineFollow_SetBaseSpeedOverride(TASK2_CRUISE_SPEED);
        CompetitionTask_UpdateStartProfile(s_task2_speed_limit, now_ms);

        if (CompetitionTask_UpdateLapYaw(
                now_ms, TASK2_STOP_YAW_DEG) != 0U) {
            CompetitionTask_RecordFinishTime(now_ms);
            s_stop_reason = COMPETITION_STOP_YAW_LAP;
            CompetitionTask_SafeStop();
            s_phase = COMPETITION_PHASE_FINISHED;
            s_last_oled_ms = 0U;
            /* Prepare task 3 before the user presses K3 on its page. */
            K230Link_RequestPrepare(3U, now_ms);
        } else if (s_imu_status < 0) {
            s_stop_reason = COMPETITION_STOP_IMU_ERROR;
            CompetitionTask_SafeStop();
            s_phase = COMPETITION_PHASE_FINISHED;
            s_last_oled_ms = 0U;
        } else {
            LineFollow_Update(now_ms);
            /* Never let a grayscale stop-line event finish task 2. */
            (void) LineFollow_StartLineConsumeEvent();
            line_data = LineFollow_GetData();
        }

        if ((s_phase == COMPETITION_PHASE_RUNNING_LAP) &&
            (line_data.state == LINE_FOLLOW_STATE_FAILSAFE)) {
            s_stop_reason = COMPETITION_STOP_LINE_LOST;
            CompetitionTask_SafeStop();
            s_phase = COMPETITION_PHASE_FINISHED;
            s_last_oled_ms = 0U;
        }
    } else if (s_phase == COMPETITION_PHASE_FINISHED) {
        CompetitionTask_SafeStop();
    }

    CompetitionTask_SendTask2YawTelemetry(now_ms);

    if (CompetitionTask_OledDue(now_ms) == 0U) {
        return;
    }

    if (s_phase == COMPETITION_PHASE_FINISHED) {
        if (s_stop_reason == COMPETITION_STOP_YAW_LAP) {
            elapsed_ms = s_result_elapsed_ms;
            TaskManager_ShowLine(1U, "T2 FINISHED");
            CompetitionTask_DrawElapsedTime(elapsed_ms);
            TaskManager_ShowLine(3U, "STOP:YAW LAP");
        } else if (s_stop_reason == COMPETITION_STOP_IMU_ERROR) {
            TaskManager_ShowLine(1U, "T2 IMU ERROR");
            (void) snprintf(diagnostic, sizeof(diagnostic),
                "IMU ERR:%d", s_imu_status);
            TaskManager_ShowLine(2U, diagnostic);
            TaskManager_ShowLine(3U, "CAR: STOP");
        } else {
            TaskManager_ShowLine(1U, "T2 STOPPED");
            TaskManager_ShowLine(2U, "STOP:LINE LOST");
            TaskManager_ShowLine(3U, "CAR: STOP");
        }
        TaskManager_ShowLine(4U, "K4 EXIT");
        TaskManager_ShowLine(5U, "");
        TaskManager_ShowLine(6U, "");
    } else {
        line_data = LineFollow_GetData();
        elapsed_ms = now_ms - s_start_ms;
        TaskManager_ShowLine(1U, "T2 LAP STOP A");
        CompetitionTask_DrawElapsedTime(elapsed_ms);
        (void) snprintf(diagnostic, sizeof(diagnostic),
            "D:%lucm V:%d",
            (unsigned long) (s_task2_distance_mm / 10U),
            (int) s_task2_speed_limit);
        TaskManager_ShowLine(3U, diagnostic);
        yaw_abs_x10 = CompetitionTask_Abs32(s_task2_accum_yaw_x10);
        yaw_target_x10 = (int32_t) (TASK2_STOP_YAW_DEG * 10U);
        if (yaw_target_x10 < 100L) {
            yaw_target_x10 = 100L;
        }
        (void) snprintf(diagnostic, sizeof(diagnostic),
            "Y:%ld.%01ld/%ld.%01ld",
            (long) (yaw_abs_x10 / 10L),
            (long) (yaw_abs_x10 % 10L),
            (long) (yaw_target_x10 / 10L),
            (long) (yaw_target_x10 % 10L));
        TaskManager_ShowLine(4U, diagnostic);
        TaskManager_ShowLine(5U, "GYRO LAP DETECT");
        TaskManager_ShowLine(6U, "K4 CANCEL");
    }

    TaskManager_ShowLine(7U, "");
    TaskManager_ShowLine(8U, "");
}

static void CompetitionTask_BeginTask4Run(uint32_t now_ms)
{
    EncoderData left = Encoder_GetData(ENCODER_1);
    EncoderData right = Encoder_GetData(ENCODER_2);

    CompetitionTask_SafeStop();
    s_start_ms = now_ms;
    s_finish_ms = 0U;
    s_result_elapsed_ms = 0U;
    s_last_oled_ms = 0U;
    s_last_telemetry_ms = 0U;
    s_task4_distance_mm = 0U;
    s_encoder_left_start = left.count;
    s_encoder_right_start = right.count;

    LineFollow_Start(now_ms);
    LineFollow_SetBaseRampIntervalMs(TASK4_BASE_RAMP_INTERVAL_MS);
    CompetitionTask_BeginStartProfile(4U, TASK4_CRUISE_SPEED, now_ms);
    s_phase = COMPETITION_PHASE_RUNNING;
}

static void CompetitionTask_UpdateTask4(uint32_t now_ms)
{
    uint32_t elapsed_ms;
    char distance_text[TASK_MANAGER_TEXT_COLUMNS + 1U];

    if (s_phase == COMPETITION_PHASE_RUNNING) {
        CompetitionTask_UpdateStartProfile(TASK4_CRUISE_SPEED, now_ms);
        LineFollow_Update(now_ms);
        s_task4_distance_mm = CompetitionTask_GetTravelDistanceMm();

        if (s_task4_distance_mm >=
            (TASK4_PASS_B_DISTANCE_CM * 10U)) {
            CompetitionTask_RecordFinishTime(now_ms);
            LineFollow_RequestSmoothStop();
            s_phase = COMPETITION_PHASE_TASK4_STOPPING;
            s_last_oled_ms = 0U;
        }
    } else if (s_phase == COMPETITION_PHASE_TASK4_STOPPING) {
        LineFollow_Update(now_ms);

        if (LineFollow_IsSmoothStopComplete() != 0U) {
            s_phase = COMPETITION_PHASE_FINISHED;
            s_last_oled_ms = 0U;
            /* Center/hold the ball for task 5 before its K3 launch. */
            K230Link_RequestPrepare(5U, now_ms);
        }
    } else if (s_phase == COMPETITION_PHASE_FINISHED) {
        Motor_Off();
    }

    if (CompetitionTask_OledDue(now_ms) == 0U) {
        return;
    }

    if (s_phase == COMPETITION_PHASE_FINISHED) {
        elapsed_ms = s_result_elapsed_ms;
        TaskManager_ShowLine(1U, "T4 FINISHED");
        TaskManager_ShowLine(3U, "CAR: STOP");
        TaskManager_ShowLine(4U, "K4 EXIT");
    } else if (s_phase == COMPETITION_PHASE_TASK4_STOPPING) {
        elapsed_ms = s_result_elapsed_ms;
        TaskManager_ShowLine(1U, "T4 PASS B");
        TaskManager_ShowLine(3U, "SMOOTH STOP");
        TaskManager_ShowLine(4U, "K4 CANCEL");
    } else {
        elapsed_ms = now_ms - s_start_ms;
        TaskManager_ShowLine(1U, "T4 A TO B");
        (void) snprintf(distance_text, sizeof(distance_text), "DIST:%lucm",
                        (unsigned long) (s_task4_distance_mm / 10U));
        TaskManager_ShowLine(3U, distance_text);
        TaskManager_ShowLine(4U, "K4 CANCEL");
    }

    CompetitionTask_DrawElapsedTime(elapsed_ms);
    TaskManager_ShowLine(5U, "");
    TaskManager_ShowLine(6U, "");
    TaskManager_ShowLine(7U, "");
    TaskManager_ShowLine(8U, "");
}

static void CompetitionTask_BeginTask56Run(uint32_t now_ms)
{
    EncoderData left = Encoder_GetData(ENCODER_1);
    EncoderData right = Encoder_GetData(ENCODER_2);

    BMI088_Gyro_ResetAngles();
    s_task2_last_yaw_x10 = 0;
    s_task2_accum_yaw_x10 = 0L;
    s_task2_yaw_valid = 0U;

    s_start_ms = now_ms;
    s_finish_ms = 0U;
    s_result_elapsed_ms = 0U;
    s_encoder_left_start = left.count;
    s_encoder_right_start = right.count;
    s_task2_distance_mm = 0U;
    CompetitionTask_ResetArcTelemetry();

    /* Tasks 5/6 use their own configurable gyro lap endpoint. */
    LineFollow_StartLineReset();
    LineFollow_Start(now_ms);
    LineFollow_SetStartLineDetectionEnabled(0U);
    LineFollow_SetStartLineIgnoreMs(0U);
    CompetitionTask_BeginStartProfile(
        (s_mode == COMPETITION_MODE_TASK5_LAP_CENTER) ? 5U : 6U,
        TASK56_CRUISE_SPEED, now_ms);
    s_phase = COMPETITION_PHASE_RUNNING_LAP;
}

static void CompetitionTask_StartTask56YawLap(CompetitionMode mode,
                                              uint32_t now_ms)
{
    CompetitionTask_SafeStop();

    s_mode = mode;
    s_start_ms = now_ms;
    s_finish_ms = 0U;
    s_result_elapsed_ms = 0U;
    s_last_oled_ms = 0U;
    s_last_telemetry_ms = 0U;
    s_stop_reason = COMPETITION_STOP_NONE;
    s_task2_distance_mm = 0U;
    s_task2_speed_limit = TASK56_CRUISE_SPEED;
    s_task2_last_yaw_x10 = 0;
    s_task2_accum_yaw_x10 = 0L;
    s_task2_yaw_valid = 0U;

    if (s_imu_status < 0) {
        s_imu_status = BMI088_Gyro_Init();
        if (s_imu_status == BMI088_GYRO_OK) {
            s_imu_status = BMI088_Gyro_CalibrationStart(
                BMI088_GYRO_DEFAULT_CALIBRATION_SAMPLES, now_ms);
        }
    }

    if (s_imu_status == BMI088_GYRO_OK) {
        CompetitionTask_BeginTask56Run(now_ms);
    } else if (s_imu_status == BMI088_GYRO_CALIBRATING) {
        s_phase = COMPETITION_PHASE_TASK2_IMU_CALIBRATING;
    } else {
        s_stop_reason = COMPETITION_STOP_IMU_ERROR;
        s_phase = COMPETITION_PHASE_FINISHED;
    }
}

static void CompetitionTask_UpdateTask56YawLap(uint32_t now_ms)
{
    uint32_t elapsed_ms;
    int32_t yaw_abs_x10;
    int32_t yaw_target_x10;
    const char *running_title;
    const char *finished_title;
    LineFollowData line_data;
    const BMI088_GyroData *imu;
    char diagnostic[TASK_MANAGER_TEXT_COLUMNS + 1U];

    if (s_phase == COMPETITION_PHASE_TASK2_IMU_CALIBRATING) {
        CompetitionTask_SafeStop();
        s_imu_status = BMI088_Gyro_CalibrationService(now_ms);

        if (s_imu_status == BMI088_GYRO_OK) {
            CompetitionTask_BeginTask56Run(now_ms);
            s_last_oled_ms = 0U;
        } else if (s_imu_status < 0) {
            s_stop_reason = COMPETITION_STOP_IMU_ERROR;
            s_phase = COMPETITION_PHASE_FINISHED;
            s_last_oled_ms = 0U;
        }

        if (CompetitionTask_OledDue(now_ms) != 0U) {
            TaskManager_ShowLine(1U,
                (s_mode == COMPETITION_MODE_TASK5_LAP_CENTER)
                    ? "T5 IMU CAL" : "T6 IMU CAL");
            TaskManager_ShowLine(2U, "KEEP CAR STILL");
            TaskManager_ShowLine(3U, "CAR: STOP");
            TaskManager_ShowLine(4U, "K4 CANCEL");
            TaskManager_ShowLine(5U, "");
            TaskManager_ShowLine(6U, "");
            TaskManager_ShowLine(7U, "");
            TaskManager_ShowLine(8U, "");
        }
        return;
    }

    if (s_phase == COMPETITION_PHASE_RUNNING_LAP) {
        s_task2_distance_mm = CompetitionTask_GetTravelDistanceMm();
        s_task2_speed_limit = TASK56_CRUISE_SPEED;
        CompetitionTask_UpdateStartProfile(s_task2_speed_limit, now_ms);

        if (CompetitionTask_UpdateLapYaw(
                now_ms, TASK56_STOP_YAW_DEG) != 0U) {
            EncoderData left;
            EncoderData right;

            /* The yaw threshold is the scored lap endpoint, not the stop. */
            CompetitionTask_RecordFinishTime(now_ms);
            s_stop_reason = COMPETITION_STOP_YAW_LAP;

            left = Encoder_GetData(ENCODER_1);
            right = Encoder_GetData(ENCODER_2);
            s_encoder_left_start = left.count;
            s_encoder_right_start = right.count;
            s_task2_distance_mm = 0U;
            s_phase = COMPETITION_PHASE_PASS_A_EXTRA;
            s_last_oled_ms = 0U;

            LineFollow_Update(now_ms);
            (void) LineFollow_StartLineConsumeEvent();
        } else if (s_imu_status < 0) {
            s_stop_reason = COMPETITION_STOP_IMU_ERROR;
            CompetitionTask_SafeStop();
            s_phase = COMPETITION_PHASE_FINISHED;
            s_last_oled_ms = 0U;
        } else {
            LineFollow_Update(now_ms);
            /* Grayscale stop-line events never finish tasks 5/6. */
            (void) LineFollow_StartLineConsumeEvent();
            line_data = LineFollow_GetData();
            if (line_data.state == LINE_FOLLOW_STATE_FAILSAFE) {
                s_stop_reason = COMPETITION_STOP_LINE_LOST;
                CompetitionTask_SafeStop();
                s_phase = COMPETITION_PHASE_FINISHED;
                s_last_oled_ms = 0U;
            }
        }
    } else if (s_phase == COMPETITION_PHASE_PASS_A_EXTRA) {
        LineFollow_SetCruiseSpeedLimit(TASK56_CRUISE_SPEED);
        LineFollow_Update(now_ms);
        (void) LineFollow_StartLineConsumeEvent();

        line_data = LineFollow_GetData();
        if (line_data.state == LINE_FOLLOW_STATE_FAILSAFE) {
            s_stop_reason = COMPETITION_STOP_LINE_LOST;
            CompetitionTask_SafeStop();
            s_phase = COMPETITION_PHASE_FINISHED;
            s_last_oled_ms = 0U;
        } else {
            s_task2_distance_mm = CompetitionTask_GetTravelDistanceMm();
            if (s_task2_distance_mm >=
                (TASK56_PASS_A_DISTANCE_CM * 10U)) {
                LineFollow_RequestSmoothStop();
                s_phase = COMPETITION_PHASE_TASK56_STOPPING;
                s_last_oled_ms = 0U;
            }
        }
    } else if (s_phase == COMPETITION_PHASE_TASK56_STOPPING) {
        LineFollow_Update(now_ms);
        if (LineFollow_IsSmoothStopComplete() != 0U) {
            s_phase = COMPETITION_PHASE_FINISHED;
            s_last_oled_ms = 0U;
            if (s_mode == COMPETITION_MODE_TASK5_LAP_CENTER) {
                /* Keep task 5 centered, then wait for task 6 selection. */
                K230Link_RequestTask6Center(now_ms);
            }
        }
    } else if (s_phase == COMPETITION_PHASE_FINISHED) {
        CompetitionTask_SafeStop();
    }

    if ((s_phase == COMPETITION_PHASE_RUNNING_LAP) ||
        (s_phase == COMPETITION_PHASE_PASS_A_EXTRA) ||
        (s_phase == COMPETITION_PHASE_TASK56_STOPPING)) {
        line_data = LineFollow_GetData();
        imu = BMI088_Gyro_GetData();
        CompetitionTask_SendTask56ArcTelemetry(
            (s_mode == COMPETITION_MODE_TASK5_LAP_CENTER) ? 5U : 6U,
            &line_data, imu, now_ms);
    }

    if (CompetitionTask_OledDue(now_ms) == 0U) {
        return;
    }

    if (s_mode == COMPETITION_MODE_TASK5_LAP_CENTER) {
        running_title = "T5 LAP CENTER";
        finished_title = "T5 LAP DONE";
    } else {
        running_title = "T6 LAP TARGET";
        finished_title = "T6 LAP DONE";
    }

    if (s_phase == COMPETITION_PHASE_FINISHED) {
        if (s_stop_reason == COMPETITION_STOP_YAW_LAP) {
            TaskManager_ShowLine(1U, finished_title);
            CompetitionTask_DrawElapsedTime(s_result_elapsed_ms);
            TaskManager_ShowLine(3U, "STOP:PASS A");
        } else if (s_stop_reason == COMPETITION_STOP_IMU_ERROR) {
            TaskManager_ShowLine(1U, "IMU ERROR");
            (void) snprintf(diagnostic, sizeof(diagnostic),
                "IMU ERR:%d", s_imu_status);
            TaskManager_ShowLine(2U, diagnostic);
            TaskManager_ShowLine(3U, "CAR: STOP");
        } else {
            TaskManager_ShowLine(1U, "LINE LOST");
            TaskManager_ShowLine(2U, "STOP:FAILSAFE");
            TaskManager_ShowLine(3U, "CAR: STOP");
        }
        TaskManager_ShowLine(4U, "K4 EXIT");
        TaskManager_ShowLine(5U, "");
    } else {
        elapsed_ms = (s_finish_ms != 0U)
            ? s_result_elapsed_ms
            : (now_ms - s_start_ms);
        TaskManager_ShowLine(1U, running_title);
        CompetitionTask_DrawElapsedTime(elapsed_ms);
        (void) snprintf(diagnostic, sizeof(diagnostic),
            (s_phase == COMPETITION_PHASE_PASS_A_EXTRA ||
             s_phase == COMPETITION_PHASE_TASK56_STOPPING)
                ? "P:%lucm V:%d" : "D:%lucm V:%d",
            (unsigned long) (s_task2_distance_mm / 10U),
            (int) s_task2_speed_limit);
        TaskManager_ShowLine(3U, diagnostic);
        yaw_abs_x10 = CompetitionTask_Abs32(s_task2_accum_yaw_x10);
        yaw_target_x10 = (int32_t) (TASK56_STOP_YAW_DEG * 10U);
        (void) snprintf(diagnostic, sizeof(diagnostic), "Y:%ld/%ld",
            (long) (yaw_abs_x10 / 10L),
            (long) (yaw_target_x10 / 10L));
        TaskManager_ShowLine(4U, diagnostic);
        TaskManager_ShowLine(5U, "K4 CANCEL");
    }

    TaskManager_ShowLine(6U, "");
    TaskManager_ShowLine(7U, "");
    TaskManager_ShowLine(8U, "");
}

static void CompetitionTask_StartCommon(CompetitionMode mode,
                                        uint32_t now_ms)
{
    CompetitionTask_SafeStop();

    s_mode = mode;
    s_phase = COMPETITION_PHASE_RUNNING;
    s_start_ms = now_ms;
    s_finish_ms = 0U;
    s_result_elapsed_ms = 0U;
    s_last_oled_ms = 0U;
    s_last_telemetry_ms = 0U;
    s_encoder_left_start = 0L;
    s_encoder_right_start = 0L;
    s_task2_distance_mm = 0U;
    s_task2_speed_limit = 0;
    s_task2_last_yaw_x10 = 0;
    s_task2_accum_yaw_x10 = 0L;
    s_task2_yaw_valid = 0U;
    s_task4_distance_mm = 0U;
    s_stop_reason = COMPETITION_STOP_NONE;
}

static void CompetitionTask_EnterK230Prepare(CompetitionMode mode,
                                            uint8_t task,
                                            uint32_t now_ms)
{
    K230LinkState link_state;

    CompetitionTask_SafeStop();

    s_mode = mode;
    s_phase = COMPETITION_PHASE_K230_PREPARING;
    s_start_ms = 0U;
    s_finish_ms = 0U;
    s_result_elapsed_ms = 0U;
    s_last_oled_ms = 0U;
    s_last_telemetry_ms = 0U;
    s_task2_distance_mm = 0U;
    s_task4_distance_mm = 0U;
    s_stop_reason = COMPETITION_STOP_NONE;

    if (task == 6U) {
        link_state = K230Link_GetState();
        if ((link_state == K230_LINK_STATE_CENTERING_TASK6) ||
            (link_state == K230_LINK_STATE_SETTING_TASK6) ||
            (K230Link_GetReadyTask() == 6U)) {
            /* Existing C6/P6/R6 preparation owns the state; do not restart. */
        } else if (K230Link_IsTask6CenterReady() != 0U) {
            K230Link_RequestPrepare(6U, now_ms);
        } else {
            K230Link_RequestTask6Center(now_ms);
        }
    } else {
        K230Link_RequestPrepare(task, now_ms);
    }
}

void CompetitionTask_Init(void)
{
    CompetitionTask_SafeStop();

    s_mode = COMPETITION_MODE_NONE;
    s_phase = COMPETITION_PHASE_IDLE;
    s_start_ms = 0U;
    s_finish_ms = 0U;
    s_result_elapsed_ms = 0U;
    s_last_oled_ms = 0U;
    s_last_telemetry_ms = 0U;
    s_encoder_left_start = 0L;
    s_encoder_right_start = 0L;
    s_task2_distance_mm = 0U;
    s_task2_speed_limit = 0;
    s_task2_last_yaw_x10 = 0;
    s_task2_accum_yaw_x10 = 0L;
    s_task2_yaw_valid = 0U;
    s_task4_distance_mm = 0U;
    s_stop_reason = COMPETITION_STOP_NONE;
    s_imu_status = BMI088_Gyro_Init();
    if (s_imu_status == BMI088_GYRO_OK) {
        /*
         * Complete gyro zero-bias calibration before the task menu starts.
         * The vehicle must remain still during this one-time power-on step;
         * afterwards K3 can start task 2 immediately with no task-side wait.
         */
        s_imu_status = BMI088_Gyro_Calibrate(
            BMI088_GYRO_DEFAULT_CALIBRATION_SAMPLES);
    }
}

void CompetitionTask1_Start(uint32_t now_ms)
{
    CompetitionTask_StartCommon(COMPETITION_MODE_TASK1_VIDEO, now_ms);
}

void CompetitionTask2_Start(uint32_t now_ms)
{
    CompetitionTask_SafeStop();

    s_mode = COMPETITION_MODE_TASK2_LAP_STOP_A;
    s_start_ms = now_ms;
    s_finish_ms = 0U;
    s_result_elapsed_ms = 0U;
    s_last_oled_ms = 0U;
    s_last_telemetry_ms = 0U;
    s_stop_reason = COMPETITION_STOP_NONE;
    s_task2_distance_mm = 0U;
    s_task2_speed_limit = CompetitionTask_GetTask2PositionSpeedLimit(0U);
    s_task2_last_yaw_x10 = 0;
    s_task2_accum_yaw_x10 = 0L;
    s_task2_yaw_valid = 0U;

    if (s_imu_status < 0) {
        s_imu_status = BMI088_Gyro_Init();
        if (s_imu_status == BMI088_GYRO_OK) {
            s_imu_status = BMI088_Gyro_CalibrationStart(
                BMI088_GYRO_DEFAULT_CALIBRATION_SAMPLES, now_ms);
        }
    }

    if (s_imu_status == BMI088_GYRO_OK) {
        CompetitionTask_BeginTask2Run(now_ms);
    } else if (s_imu_status == BMI088_GYRO_CALIBRATING) {
        s_phase = COMPETITION_PHASE_TASK2_IMU_CALIBRATING;
    } else {
        s_stop_reason = COMPETITION_STOP_IMU_ERROR;
        s_phase = COMPETITION_PHASE_FINISHED;
    }
}

void CompetitionTask3_Start(uint32_t now_ms)
{
    s_mode = COMPETITION_MODE_TASK3_BALL_STATIC;
    if (K230Link_StartTask(3U, now_ms) != 0U) {
        CompetitionTask_BeginTask3Run(now_ms);
    } else {
        CompetitionTask_EnterK230Prepare(
            COMPETITION_MODE_TASK3_BALL_STATIC, 3U, now_ms);
    }
}

void CompetitionTask4_Start(uint32_t now_ms)
{
    s_mode = COMPETITION_MODE_TASK4_A_TO_B;
    if (K230Link_StartTask(4U, now_ms) != 0U) {
        CompetitionTask_BeginTask4Run(now_ms);
    } else {
        CompetitionTask_EnterK230Prepare(
            COMPETITION_MODE_TASK4_A_TO_B, 4U, now_ms);
    }
}

void CompetitionTask5_Start(uint32_t now_ms)
{
    s_mode = COMPETITION_MODE_TASK5_LAP_CENTER;
    if (K230Link_StartTask(5U, now_ms) != 0U) {
        CompetitionTask_StartTask56YawLap(
            COMPETITION_MODE_TASK5_LAP_CENTER, now_ms);
    } else {
        CompetitionTask_EnterK230Prepare(
            COMPETITION_MODE_TASK5_LAP_CENTER, 5U, now_ms);
    }
}

void CompetitionTask6_Start(uint32_t now_ms)
{
    s_mode = COMPETITION_MODE_TASK6_LAP_TARGET;
    if (K230Link_StartTask(6U, now_ms) != 0U) {
        CompetitionTask_StartTask56YawLap(
            COMPETITION_MODE_TASK6_LAP_TARGET, now_ms);
    } else {
        CompetitionTask_EnterK230Prepare(
            COMPETITION_MODE_TASK6_LAP_TARGET, 6U, now_ms);
    }
}

void CompetitionTask7_Start(uint32_t now_ms)
{
    CompetitionTask_StartCommon(COMPETITION_MODE_TASK7_OTHER, now_ms);
}

void CompetitionTask8_Start(uint32_t now_ms)
{
    CompetitionTask_SafeStop();

    s_mode = COMPETITION_MODE_TASK8_LINE_TUNE;
    s_phase = COMPETITION_PHASE_RUNNING;
    s_start_ms = now_ms;
    s_finish_ms = 0U;
    s_result_elapsed_ms = 0U;
    s_last_oled_ms = 0U;
    s_last_telemetry_ms = 0U;
    s_stop_reason = COMPETITION_STOP_NONE;

    if (s_imu_status == BMI088_GYRO_OK) {
        s_imu_status = BMI088_Gyro_CalibrationStart(
            BMI088_GYRO_DEFAULT_CALIBRATION_SAMPLES, now_ms);
    }

    s_phase = (s_imu_status == BMI088_GYRO_CALIBRATING)
        ? COMPETITION_PHASE_TASK8_IMU_CALIBRATING
        : COMPETITION_PHASE_FINISHED;
}

static uint8_t CompetitionTask_GetK230TaskId(void)
{
    switch (s_mode) {
        case COMPETITION_MODE_TASK3_BALL_STATIC: return 3U;
        case COMPETITION_MODE_TASK4_A_TO_B:      return 4U;
        case COMPETITION_MODE_TASK5_LAP_CENTER: return 5U;
        case COMPETITION_MODE_TASK6_LAP_TARGET: return 6U;
        default:                                 return 0U;
    }
}

static const char *CompetitionTask_GetK230PrepareTitle(void)
{
    switch (s_mode) {
        case COMPETITION_MODE_TASK3_BALL_STATIC: return "T3 BALL +5/-5";
        case COMPETITION_MODE_TASK4_A_TO_B:      return "T4 A TO B";
        case COMPETITION_MODE_TASK5_LAP_CENTER: return "T5 LAP CENTER";
        case COMPETITION_MODE_TASK6_LAP_TARGET: return "T6 LAP TARGET";
        default:                                 return "K230 PREPARE";
    }
}

static void CompetitionTask_BeginPreparedK230Task(uint8_t task,
                                                  uint32_t now_ms)
{
    if (K230Link_StartTask(task, now_ms) == 0U) {
        return;
    }

    switch (task) {
        case 3U:
            CompetitionTask_BeginTask3Run(now_ms);
            break;
        case 4U:
            CompetitionTask_BeginTask4Run(now_ms);
            break;
        case 5U:
            CompetitionTask_StartTask56YawLap(
                COMPETITION_MODE_TASK5_LAP_CENTER, now_ms);
            break;
        case 6U:
            CompetitionTask_StartTask56YawLap(
                COMPETITION_MODE_TASK6_LAP_TARGET, now_ms);
            break;
        default:
            break;
    }
}

static void CompetitionTask_UpdateK230Preparing(uint32_t now_ms)
{
    uint8_t task = CompetitionTask_GetK230TaskId();
    uint8_t ready;
    const char *prompt;

    CompetitionTask_SafeStop();

    if ((task == 6U) && (K230Link_GetReadyTask() != 6U) &&
        (K230Link_IsTask6CenterReady() != 0U)) {
        /* C6OK means centered; only now may K230 choose/hold task 6 target. */
        K230Link_RequestPrepare(6U, now_ms);
    }

    ready = K230Link_IsReady(task, now_ms);
    if ((ready != 0U) &&
        ((TaskManager_GetPressedEdges() & BUTTON_3_MASK) != 0U)) {
        /* This K3 edge is the official start and timing instant. */
        CompetitionTask_BeginPreparedK230Task(task, now_ms);
        return;
    }

    if (CompetitionTask_OledDue(now_ms) == 0U) {
        return;
    }

    TaskManager_ShowLine(1U, CompetitionTask_GetK230PrepareTitle());
    TaskManager_ShowLine(2U, K230Link_GetStatusText(now_ms));
    if (ready != 0U) {
        prompt = "READY: K3 START";
    } else if (K230Link_IsOnline(now_ms) == 0U) {
        prompt = "UART: NO DATA";
    } else {
        switch (K230Link_GetState()) {
            case K230_LINK_STATE_PREPARING:
                prompt = "WAIT BALL CENTER";
                break;
            case K230_LINK_STATE_CENTERING_TASK6:
                prompt = "BALL TO CENTER";
                break;
            case K230_LINK_STATE_SETTING_TASK6:
                prompt = "PRESS K230 KEY";
                break;
            case K230_LINK_STATE_ERROR:
                prompt = "K230 ERROR";
                break;
            default:
                prompt = "WAIT K230 READY";
                break;
        }
    }
    TaskManager_ShowLine(3U, prompt);
    TaskManager_ShowLine(4U, "CAR: STOP");
    TaskManager_ShowLine(5U, "K4 EXIT");
    TaskManager_ShowLine(6U, "");
    TaskManager_ShowLine(7U, "");
    TaskManager_ShowLine(8U, "");
}

TaskManagerResult CompetitionTask_Update(uint32_t now_ms)
{
    if (s_phase == COMPETITION_PHASE_K230_PREPARING) {
        CompetitionTask_UpdateK230Preparing(now_ms);
        return TASK_MANAGER_RESULT_RUNNING;
    }

    switch (s_mode) {
        case COMPETITION_MODE_TASK1_VIDEO:
            CompetitionTask_SafeStop();
            CompetitionTask_UpdateTask1(now_ms);
            break;

        case COMPETITION_MODE_TASK2_LAP_STOP_A:
            CompetitionTask_UpdateTask2(now_ms);
            break;

        case COMPETITION_MODE_TASK3_BALL_STATIC:
            CompetitionTask_UpdateTask3(now_ms);
            break;

        case COMPETITION_MODE_TASK4_A_TO_B:
            CompetitionTask_UpdateTask4(now_ms);
            break;

        case COMPETITION_MODE_TASK5_LAP_CENTER:
        case COMPETITION_MODE_TASK6_LAP_TARGET:
            CompetitionTask_UpdateTask56YawLap(now_ms);
            break;

        case COMPETITION_MODE_TASK7_OTHER:
            CompetitionTask_SafeStop();
            CompetitionTask_UpdateTask7(now_ms);
            break;

        case COMPETITION_MODE_TASK8_LINE_TUNE:
            CompetitionTask_UpdateTask8(now_ms);
            break;

        default:
            CompetitionTask_SafeStop();
            if (CompetitionTask_OledDue(now_ms) != 0U) {
                CompetitionTask_DrawPlaceholder();
            }
            break;
    }

    return TASK_MANAGER_RESULT_RUNNING;
}

void CompetitionTask_Cancel(void)
{
    CompetitionMode old_mode = s_mode;
    CompetitionPhase old_phase = s_phase;
    uint8_t keep_k230_preparation = 0U;

    if (old_phase == COMPETITION_PHASE_FINISHED) {
        if ((old_mode == COMPETITION_MODE_TASK2_LAP_STOP_A) &&
            (s_stop_reason == COMPETITION_STOP_YAW_LAP)) {
            keep_k230_preparation = 1U;
        } else if ((old_mode == COMPETITION_MODE_TASK3_BALL_STATIC) &&
                   (s_finish_ms != 0U)) {
            keep_k230_preparation = 1U;
        } else if (old_mode == COMPETITION_MODE_TASK4_A_TO_B) {
            keep_k230_preparation = 1U;
        } else if ((old_mode == COMPETITION_MODE_TASK5_LAP_CENTER) &&
                   (s_stop_reason == COMPETITION_STOP_YAW_LAP)) {
            keep_k230_preparation = 1U;
        }
    }

    CompetitionTask_SafeStop();

    if ((keep_k230_preparation == 0U) &&
        ((old_mode == COMPETITION_MODE_TASK3_BALL_STATIC) ||
         (old_mode == COMPETITION_MODE_TASK4_A_TO_B) ||
         (old_mode == COMPETITION_MODE_TASK5_LAP_CENTER) ||
         (old_mode == COMPETITION_MODE_TASK6_LAP_TARGET))) {
        K230Link_Stop(g_ms);
    }

    s_mode = COMPETITION_MODE_NONE;
    s_phase = COMPETITION_PHASE_IDLE;
    s_start_ms = 0U;
    s_finish_ms = 0U;
    s_result_elapsed_ms = 0U;
    s_last_oled_ms = 0U;
    s_last_telemetry_ms = 0U;
    s_encoder_left_start = 0L;
    s_encoder_right_start = 0L;
    s_task2_distance_mm = 0U;
    s_task2_speed_limit = 0;
    s_task2_last_yaw_x10 = 0;
    s_task2_accum_yaw_x10 = 0L;
    s_task2_yaw_valid = 0U;
    s_task4_distance_mm = 0U;
    s_stop_reason = COMPETITION_STOP_NONE;
}
