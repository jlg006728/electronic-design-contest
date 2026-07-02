#ifndef CONFIG_H
#define CONFIG_H

#include <stdint.h>

#define SYSCLK_HZ                32000000U

#define WHEEL_DIAMETER_M         0.065f
#define WHEEL_BASE_M             0.150f
#define PI_F                     3.14159265f
#define WHEEL_CIRCUMFERENCE_M    (PI_F * WHEEL_DIAMETER_M)

/*
 * Current encoder ISR counts both edges of channel A only and uses channel B
 * for direction. With 500-line GMR and 1:30 gearbox this is roughly
 * 500 * 30 * 2 = 30000 counts per wheel revolution. Recalibrate on the bench.
 */
#define ENCODER_COUNTS_PER_REV   30000.0f

#define LAP_CIRCUMFERENCE_M      4.0f
#define MAX_LAP_TIME_S           20.0f
#define TARGET_SPEED_M_PER_S     0.28f
#define TRACK_OPEN_LOOP_DUTY     34.0f
#define MAX_LAPS                 5U

#define SENSOR_SAMPLE_HZ         200U
#define POSITION_PID_HZ          100U
#define GIMBAL_PID_HZ            50U
#define OPENMV_REQ_HZ            25U
#define STATE_MACHINE_HZ         20U
#define ODOMETRY_HZ              100U

#define LINE_SENSOR_COUNT        7U
#define LINE_BLACK_IS_HIGH       1U
#define LINE_LOST_TURN_DUTY      22.0f

#define TARGET_DISTANCE_M        0.50f
#define TARGET_RADIUS_MAX_CM     10.0f
#define CIRCLE_DRAW_RADIUS_CM    6.0f
#define HIT_PRECISION_CM         2.0f

#define MOTOR_PWM_HZ             20000U
#define MOTOR_PWM_PERIOD         1600U

#define SERVO_PWM_HZ             50U
#define SERVO_PERIOD_US          20000U
#define SERVO_MIN_US             500U
#define SERVO_MAX_US             2500U
#define SERVO_CENTER_US          1500U
#define SERVO_MIN_DEG            0.0f
#define SERVO_MAX_DEG            180.0f

#define GIMBAL_X_CENTER_DEG      90.0f
#define GIMBAL_Y_CENTER_DEG      90.0f
#define GIMBAL_X_ANGLE_MIN_DEG   25.0f
#define GIMBAL_X_ANGLE_MAX_DEG   155.0f
#define GIMBAL_Y_ANGLE_MIN_DEG   45.0f
#define GIMBAL_Y_ANGLE_MAX_DEG   135.0f
#define GIMBAL_IMAGE_CENTER_X    160.0f
#define GIMBAL_IMAGE_CENTER_Y    120.0f

#define SPEED_PID_KP             18.0f
#define SPEED_PID_KI             2.0f
#define SPEED_PID_I_MAX          20.0f
#define SPEED_PID_OUT_MAX        80.0f

#define POSITION_PID_KP          13.0f
#define POSITION_PID_KD          5.0f
#define POSITION_PID_OUT_MAX     30.0f

#define GIMBAL_X_PID_KP          0.045f
#define GIMBAL_X_PID_KI          0.000f
#define GIMBAL_X_PID_KD          0.010f
#define GIMBAL_X_PID_I_MAX       5.0f
#define GIMBAL_X_PID_OUT_MAX     2.5f

#define GIMBAL_Y_PID_KP          0.045f
#define GIMBAL_Y_PID_KI          0.000f
#define GIMBAL_Y_PID_KD          0.010f
#define GIMBAL_Y_PID_I_MAX       5.0f
#define GIMBAL_Y_PID_OUT_MAX     2.5f

#define OPENMV_FRAME_HEADER      0x2CU
#define OPENMV_FRAME_FOOTER      0x5BU
#define OPENMV_FRAME_LENGTH      8U
#define OPENMV_UART_BAUDRATE     115200U
#define OPENMV_REQUEST_BYTE      'X'

typedef enum {
    MODE_TRACK_ONLY = 0,
    MODE_AIM_STATIC = 1,
    MODE_TRACK_AIM_N1 = 2,
    MODE_TRACK_AIM_N2 = 3,
    MODE_TRACK_DRAW = 4,
    MODE_COUNT
} system_mode_t;

#endif
