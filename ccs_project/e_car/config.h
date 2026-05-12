#ifndef CONFIG_H
#define CONFIG_H

#include <stdint.h>

/* ============================================================
 * 车体机械参数
 * ============================================================ */
#define WHEEL_DIAMETER_M       0.065f
#define WHEEL_BASE_M           0.150f
#define WHEEL_CIRCUMFERENCE_M  (3.14159265f * WHEEL_DIAMETER_M)
#define ENCODER_CPR            585   /* 13线 × 45减速比 (WHEELTEC套件) */

/* ============================================================
 * 速度/时间参数
 * ============================================================ */
#define LAP_CIRCUMFERENCE_M   4.0f
#define MAX_LAP_TIME_S        20.0f
#define MIN_SPEED_M_PER_S     0.20f
#define TARGET_SPEED_M_PER_S  0.30f
#define MAX_LAPS              5

/* ============================================================
 * 系统定时频率 (Hz)
 * ============================================================ */
#define SENSOR_SAMPLE_HZ      1000
#define SPEED_PID_HZ          1000
#define POSITION_PID_HZ       200
#define GIMBAL_PID_HZ         50
#define OPENMV_REQ_HZ         50
#define STATE_MACHINE_HZ      10
#define ODOMETRY_HZ           100

/* ============================================================
 * 巡线传感器
 * ============================================================ */
#define LINE_SENSOR_COUNT     7
#define LINE_SENSOR_SPACING_CM 1.2f
#define LINE_SENSOR_WHITE_THRESH 2000

/* ============================================================
 * 靶面参数
 * ============================================================ */
#define TARGET_DISTANCE_M     0.50f
#define TARGET_RADIUS_MAX_CM  10.0f
#define CIRCLE_DRAW_RADIUS_CM 6.0f
#define HIT_PRECISION_CM      2.0f

/* ============================================================
 * 舵机参数 (MG90S @ 50Hz PWM)
 * ============================================================ */
#define SERVO_PWM_FREQ_HZ     50
#define SERVO_PERIOD_US       20000
#define SERVO_MIN_US          500
#define SERVO_MAX_US          2500
#define SERVO_CENTER_US       1500
#define SERVO_US_PER_DEG      ((SERVO_MAX_US - SERVO_MIN_US) / 180.0f)

/* ============================================================
 * 云台角度限制
 * ============================================================ */
#define GIMBAL_X_ANGLE_MIN_DEG  30.0f
#define GIMBAL_X_ANGLE_MAX_DEG  150.0f
#define GIMBAL_Y_ANGLE_MIN_DEG  60.0f
#define GIMBAL_Y_ANGLE_MAX_DEG  120.0f

/* ============================================================
 * PID参数
 * ============================================================ */
/* 速度环 (增量式PI) */
#define SPEED_PID_KP          0.5f
#define SPEED_PID_KI          0.1f
#define SPEED_PID_I_MAX       30.0f
#define SPEED_PID_OUT_MAX     100.0f

/* 位置环 (巡线, 位置式PD) */
#define POSITION_PID_KP       1.0f
#define POSITION_PID_KD       0.3f
#define POSITION_PID_OUT_MAX  0.8f

/* 云台水平轴 */
#define GIMBAL_X_PID_KP       0.05f
#define GIMBAL_X_PID_KI       0.005f
#define GIMBAL_X_PID_KD       0.01f
#define GIMBAL_X_PID_I_MAX    5.0f
#define GIMBAL_X_PID_OUT_MAX  45.0f

/* 云台俯仰轴 */
#define GIMBAL_Y_PID_KP       0.05f
#define GIMBAL_Y_PID_KI       0.005f
#define GIMBAL_Y_PID_KD       0.01f
#define GIMBAL_Y_PID_I_MAX    5.0f
#define GIMBAL_Y_PID_OUT_MAX  45.0f

/* ============================================================
 * PWM 定时器参数
 * ============================================================ */
#define PWM_MOTOR_TOP          1600
#define PWM_SERVO_TOP          640000
#define PWM_TOP_VALUE          PWM_MOTOR_TOP

/* ============================================================
 * OpenMV 通信协议
 * ============================================================ */
#define OPENMV_FRAME_HEADER   0x2C
#define OPENMV_FRAME_FOOTER   0x5B
#define OPENMV_FRAME_LENGTH   7
#define OPENMV_UART_BAUDRATE  115200

/* ============================================================
 * 模式枚举
 * ============================================================ */
typedef enum {
    MODE_TRACK_ONLY       = 0,
    MODE_AIM_STATIC       = 1,
    MODE_TRACK_AIM_N1     = 2,
    MODE_TRACK_AIM_N2     = 3,
    MODE_TRACK_DRAW       = 4,
    MODE_COUNT
} system_mode_t;

#endif
