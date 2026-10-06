/**
 * @file vehicle_tuning.c
 * @brief 全车实车调参唯一入口。
 *
 * 修改原则：先确认传感器方向和电机方向，再按“速度环 -> yaw环 -> 位置环”
 * 的顺序调PID。这里仅放需要依据实车变化的参数；引脚、错误码、数组长度、
 * SysConfig时钟周期等编译结构参数仍保留在各模块中。
 */

#include <stdbool.h>
#include <stdint.h>

/* ============================ 电机与三环PID ============================ */
/* 来源：电机/驱动接线。正速度方向错误时改符号；左右通道接反时改swap。 */
/* Closed-loop test confirmed MOTOR_LEFT drives encoder 1 and MOTOR_RIGHT
 * drives encoder 2; swapping them creates positive cross-coupling. */
uint8_t MOTOR_LOGICAL_SIDE_SWAP = 0U;
/* Current vehicle test: use the former negative motor direction as forward. */
int32_t MOTOR_OUTPUT_SIGN = 1L;

/* 来源：编码器规格和实测一圈计数；用于距离换算和左右轮一致性。 */
/* Measured over 10 wheel turns: 7845 counts = 784.5 counts/rev. */
uint32_t MOTOR_ENCODER_LEFT_CPR = 785U;
uint32_t MOTOR_ENCODER_RIGHT_CPR = 785U;

/*
 * 车轮有效直径，单位为微米。65000 um = 65.000 mm。
 * 当前先使用标称直径；完成1 m或2 m落地标定后，可微调这个值补偿
 * 轮胎压缩、打滑和实际外径误差。
 */
uint32_t MOTOR_WHEEL_DIAMETER_UM = 65000U;

/* 来源：满速实测counts/s；把编码器速度换算成-100~100速度单位。 */
int32_t ENCODER_SPEED_100_PERCENT_CPS = 4000L;

/* 速度内环：输入左右目标速度，输出PWM；参数为实际值的1000倍。 */
/*
 * 架空实测临时控制器参数为 Kp=2.8, Ki=1.0, Kd=0。
 * 正式 PID 每10 ms直接累加误差，而临时控制器按 error*0.01 s
 * 积分，因此正式 Ki_x1000 必须缩小100倍：1000 -> 10。
 */
int32_t MOTOR_SPEED_PID_KP_X1000 = 2800L;
int32_t MOTOR_SPEED_PID_KI_X1000 = 10L;
int32_t MOTOR_SPEED_PID_KD_X1000 = 0L;

/* 位置外环：输入剩余编码器计数，输出基础目标速度。 */
int32_t MOTOR_POSITION_PID_KP_X1000 = 200L;
int32_t MOTOR_POSITION_PID_KI_X1000 = 0L;
int32_t MOTOR_POSITION_PID_KD_X1000 = 120L;

/* yaw方向环：输入目标与实际yaw误差，输出左右轮差速。 */
int32_t MOTOR_YAW_PID_KP_X1000 = 120L;
int32_t MOTOR_YAW_PID_KI_X1000 = 0L;
int32_t MOTOR_YAW_PID_KD_X1000 = 300L;

int32_t MOTOR_POSITION_SPEED_LIMIT = 100L;      /* 位置环最大速度命令。 */
int32_t MOTOR_YAW_SPEED_LIMIT = 20L;            /* yaw环最大左右差速，实车安全限制。 */
int32_t MOTOR_POSITION_TOLERANCE_COUNTS = 5L;   /* 位置到达允许误差。 */
int32_t MOTOR_YAW_TOLERANCE_X10 = 10L;          /* yaw到达误差，10=1度。 */
uint16_t MOTOR_TARGET_STABLE_SAMPLES = 10U;     /* 连续满足次数，10=100ms。 */
uint32_t MOTOR_IMU_TIMEOUT_MS = 200U;             /* IMU数据超时保护，需大于姿态更新周期。 */

/* =============================== BMI088 =============================== */
uint16_t BMI088_GYRO_DEFAULT_CALIBRATION_SAMPLES = 300U; /* 上电零偏样本数。 */
uint16_t BMI088_GYRO_DEFAULT_UPDATE_PERIOD_MS = 2U;      /* 姿态更新周期。 */
float BMI088_FILTER_TIME_SEC = 0.50f;                    /* roll/pitch互补滤波。 */
float BMI088_ACCEL_MIN_MG_SQ = 250000.0f;                /* 融合加速度下限0.5g²。 */
float BMI088_ACCEL_MAX_MG_SQ = 2250000.0f;               /* 融合加速度上限1.5g²。 */
int16_t BMI088_Z_DEADBAND_X10 = 2;                       /* yaw角速度死区0.2°/s。 */
int16_t BMI088_YAW_STILL_ENTER_XY_X10 = 15;              /* 静止进入XY阈值。 */
int16_t BMI088_YAW_STILL_ENTER_Z_X10 = 8;                /* 静止进入Z阈值。 */
int16_t BMI088_YAW_STILL_EXIT_XY_X10 = 30;               /* 静止退出XY阈值。 */
int16_t BMI088_YAW_STILL_EXIT_Z_X10 = 15;                /* 静止退出Z阈值。 */
float BMI088_YAW_STILL_MIN_MG_SQ = 722500.0f;            /* 静止重力下限0.85g²。 */
float BMI088_YAW_STILL_MAX_MG_SQ = 1322500.0f;           /* 静止重力上限1.15g²。 */
uint32_t BMI088_YAW_STILL_CONFIRM_MS = 800U;              /* 静止确认时间。 */
float BMI088_YAW_BIAS_TRACK_TIME_SEC = 2.0f;              /* 在线零偏跟踪时间常数。 */
float BMI088_YAW_RATE_FILTER_TIME_SEC = 0.030f;           /* yaw角速度低通。 */
uint16_t BMI088_CAL_WARMUP_SAMPLES = 5U;                  /* 校准前丢弃样本。 */

/* ============================== MMC5983MA ============================= */
uint32_t MMC5983MA_SERVICE_PERIOD_MS = 20U; /* 磁场读取周期。 */
int32_t MMC5983MA_MIN_CAL_SPAN = 1000L;     /* 每轴校准最小变化量。 */
uint16_t MMC5983MA_MIN_FIELD_MG = 150U;     /* 合理磁场下限mG。 */
uint16_t MMC5983MA_MAX_FIELD_MG = 1000U;    /* 合理磁场上限mG。 */
int32_t MMC5983MA_X_SIGN = 1L;              /* X轴安装方向。 */
int32_t MMC5983MA_Y_SIGN = 1L;              /* Y轴安装方向。 */
int32_t MMC5983MA_Z_SIGN = 1L;              /* Z轴安装方向。 */
float MMC5983MA_HEADING_SIGN = 1.0f;        /* 航向角递增方向。 */
int16_t MMC5983MA_DECLINATION_X10 = 0;      /* 当地磁偏角，0.1度。 */

/* ============================= 舵机与提示器 =========================== */
float SERVO_MIN_ANGLE_DEG = 0.0f;       /* 机械安全最小角。 */
float SERVO_MAX_ANGLE_DEG = 180.0f;     /* 机械安全最大角。 */
float SERVO_DEFAULT_ANGLE_DEG = 90.0f;  /* 上电角度。 */
uint16_t SERVO_MIN_PULSE_US = 500U;     /* 最小角对应脉宽。 */
uint16_t SERVO_CENTER_PULSE_US = 1500U; /* 中位脉宽。 */
uint16_t SERVO_MAX_PULSE_US = 2500U;    /* 最大角对应脉宽。 */
uint8_t SERVO_DIRECTION_NORMAL = 1U;    /* 0可反转安装方向。 */
uint8_t SERVO_COMPARE_INVERTED = 0U;    /* PWM有效极性相反时设1。 */
uint8_t LED_ACTIVE_HIGH = 1U;           /* LED高电平点亮。 */
uint8_t BUZZER_ACTIVE_HIGH = 1U;        /* 蜂鸣器高电平鸣响。 */
uint32_t BUZZER_SHORT_BEEP_MS = 100U;   /* 短提示音。 */
uint32_t BUZZER_LONG_BEEP_MS = 500U;    /* 长提示音。 */

/* =============================== 人机输入 ============================= */
uint32_t BUTTON_DEBOUNCE_MS = 30U;      /* 机械按键消抖。 */
uint32_t BUTTON_LONG_PRESS_MS = 800U;   /* 长按判定。 */

/* ============================== 超声波测距 ============================ */
uint32_t CHAO_MEASURE_PERIOD_MS = 60U;   /* HC-SR04建议相邻触发不少于60ms。 */
uint32_t CHAO_ECHO_TIMEOUT_MS = 80U;     /* Combined捕获跨两次触发，须大于触发周期。 */
uint32_t CHAO_SOUND_SPEED_MM_S = 343000U;/* 20℃空气声速；温差大时可调整。 */
uint16_t CHAO_MIN_ECHO_US = 100U;        /* 小于约17mm通常视为毛刺。 */
uint16_t CHAO_MAX_ECHO_US = 30000U;      /* 大于约5.1m视为超范围。 */
uint8_t CHAO_FILTER_OLD_WEIGHT = 3U;     /* 新值权重1，旧值权重3。 */

/* ============================ 数字灰度循迹 ============================ */
uint8_t LINE_SENSOR_ACTIVE_LOW = 0U;    /* 实测：4/5压黑线为0x18，6/7为0x60。 */
uint32_t LINE_FOLLOW_PERIOD_MS = 5U;
int16_t LINE_STRAIGHT_SPEED = 37;
int16_t LINE_BASE_SPEED = 30;
int16_t LINE_MIN_SPEED = 6;
int16_t LINE_MAX_SPEED = 50;
int16_t LINE_LOST_FORWARD_SPEED = 8;
int16_t LINE_LOST_TURN_SPEED = 14;
int16_t LINE_INTERSECTION_SPEED = 24;
int16_t LINE_MAX_CORRECTION = 32;
int32_t LINE_CURVE_SLOWDOWN_X100 = 40L;
uint32_t LINE_LOST_COAST_MS = 60U;
uint32_t LINE_LOST_STOP_MS = 1500U;
uint32_t START_LINE_IGNORE_MS = 5000U;
int32_t LINE_KP_X100 = 50L;             /* 正确黑线掩码下的保守Kp=0.50。 */
int32_t LINE_KD_X100 = 120L;            /* 正确黑线掩码下的保守Kd=1.20。 */
int16_t LINE_RAMP_NORMAL = 1;
int16_t LINE_RAMP_FAST = 3;
int16_t LINE_BASE_RAMP_STEP = 1;
uint32_t LINE_BASE_RAMP_INTERVAL_MS = 10U;
int16_t LINE_CORRECTION_RAMP_STEP = 2;
int16_t LINE_CORRECTION_REVERSE_RAMP_STEP = 1;
int16_t LINE_DERIVATIVE_LIMIT_X10 = 80;
int16_t LINE_ERROR_RECOVERY_GAP_X10 = 80;

/* 1.5 m straights plus two r=0.5 m semicircles: 3.0 m + pi m. */
uint32_t TRACK_LAP_DISTANCE_CM = 614U;
int16_t TASK4_CRUISE_SPEED = 28;
uint32_t TASK4_PASS_B_DISTANCE_CM = 175U;
uint32_t TASK4_BASE_RAMP_INTERVAL_MS = 20U;
uint32_t TASK4_BRAKE_START_CM = 120U;
uint32_t TASK4_CRAWL_START_CM = 145U;
int16_t TASK2_CRUISE_SPEED = 41;
uint32_t TASK2_BRAKE_START_CM = 595U;
int32_t TASK2_POSITION_KP_X100 = 160L;
uint32_t TASK2_STOP_YAW_DEG = 325U;
uint32_t TASK56_STOP_YAW_DEG = 360U;
int16_t TASK56_CRUISE_SPEED = 25;
uint32_t TASK56_PASS_A_DISTANCE_CM = 8U;
int16_t APPROACH_SPEED = 18;
int16_t CRAWL_SPEED = 11;

/*
 * Launch S-curve and feedforward preparation.
 * Acceleration is expressed in car speed-percent per second.  The feedforward
 * angle is sent in 0.1 degree units; the present K230 firmware safely ignores
 * FC/FA/FE until its matching parser is enabled.
 */
uint32_t START_S_CURVE_MS = 900U;
uint32_t START_BALL_S_CURVE_MS = 1300U;
uint32_t START_FF_LEAD_MS = 80U;
int32_t START_FF_GAIN_X100 = 5L;
int32_t START_FF_DAMP_X100 = 0L;
int16_t START_FF_MAX_X10 = 30;
int16_t START_FF_SIGN = 1;
uint8_t START_FF_LINK_ENABLE = 1U;
uint8_t START_IMU_FORWARD_AXIS = 0U;   /* 0=X, 1=Y; diagnostics only. */
uint8_t START_IMU_FORWARD_INVERT = 0U;
uint32_t START_FF_LOG_MS = 2400U;


int32_t g_line_filter_alpha_normal = 5;
int32_t g_line_filter_alpha_fast   = 2;

/*
 * 直角弯参数：
 * 左转yaw增加，因此左直角目标=当前yaw+LINE_CORNER_ANGLE_X10；
 * 右直角目标=当前yaw-LINE_CORNER_ANGLE_X10。
 */
int16_t LINE_CORNER_ANGLE_X10 = 900;             /* 900=90.0°。 */
int16_t LINE_CORNER_YAW_TOLERANCE_X10 = 20;      /* 20=2.0°。 */
uint16_t LINE_CORNER_YAW_STABLE_SAMPLES = 10U;   /* 5ms周期下约50ms。 */
uint32_t LINE_CORNER_TURN_TIMEOUT_MS = 2500U;    /* yaw闭环转向超时。 */

/* yaw闭环结束后，不再保持角度，仅低速同方向原地寻找新线。 */
int16_t LINE_CORNER_FIND_SPEED = 8;
uint32_t LINE_CORNER_FIND_TIMEOUT_MS = 1200U;
uint16_t LINE_CORNER_FIND_CONFIRM_SAMPLES = 3U;  /* 中间通道连续确认。 */
uint32_t LINE_CORNER_COOLDOWN_MS = 300U;         /* 防止恢复后重复触发。 */
/* ============================ 相机循迹通用值 ========================== */
int16_t VISION_LINE_CENTER_X = 160;       /* 图像中心横坐标。 */
int16_t VISION_LINE_BASE_SPEED = 20;
int16_t VISION_LINE_MIN_SPEED = 0;
int16_t VISION_LINE_MAX_SPEED = 30;
int32_t VISION_LINE_KP_X1000 = 80L;       /* 相机偏差比例增益0.080。 */
uint16_t VISION_LINE_MAX_CENTER_X = 320U; /* 图像宽度/最大中心坐标。 */
uint32_t VISION_LINE_FRAME_TIMEOUT_MS = 300U; /* 丢帧停车超时。 */
