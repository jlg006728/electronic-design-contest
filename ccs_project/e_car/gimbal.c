#include "gimbal.h"
#include "pid.h"
#include <math.h>
#include "ti_msp_dl_config.h"

static pid_controller_t g_pid_x;
static pid_controller_t g_pid_y;
static float g_angle_x;
static float g_angle_y;

static uint32_t angle_to_ccr(float angle_deg)
{
    if (angle_deg < 0.0f)   angle_deg = 0.0f;
    if (angle_deg > 180.0f) angle_deg = 180.0f;
    float us = SERVO_MIN_US + angle_deg * SERVO_US_PER_DEG;
    if (us < SERVO_MIN_US) us = SERVO_MIN_US;
    if (us > SERVO_MAX_US) us = SERVO_MAX_US;
    return (uint32_t)(us * (float)PWM_SERVO_TOP / (float)SERVO_PERIOD_US);
}

void gimbal_init(void)
{
    g_angle_x = 90.0f;
    g_angle_y = 90.0f;

    pid_init(&g_pid_x, GIMBAL_X_PID_KP, GIMBAL_X_PID_KI, GIMBAL_X_PID_KD,
             GIMBAL_X_PID_I_MAX, GIMBAL_X_PID_OUT_MAX, 2.0f);
    pid_set_setpoint(&g_pid_x, 160.0f);

    pid_init(&g_pid_y, GIMBAL_Y_PID_KP, GIMBAL_Y_PID_KI, GIMBAL_Y_PID_KD,
             GIMBAL_Y_PID_I_MAX, GIMBAL_Y_PID_OUT_MAX, 2.0f);
    pid_set_setpoint(&g_pid_y, 120.0f);

    DL_Timer_setCaptureCompareValue(PWM_SERVO_TIMER,
        angle_to_ccr(90.0f), DL_TIMER_CC_0_INDEX);
    DL_Timer_setCaptureCompareValue(PWM_SERVO_TIMER,
        angle_to_ccr(90.0f), DL_TIMER_CC_1_INDEX);
    DL_TimerG_startCounter(PWM_SERVO_TIMER);

    gimbal_enable_laser(false);
}

void gimbal_set_angle_x(float angle_deg)
{
    g_angle_x = angle_deg;
    DL_Timer_setCaptureCompareValue(PWM_SERVO_TIMER,
        angle_to_ccr(angle_deg), DL_TIMER_CC_0_INDEX);
}

void gimbal_set_angle_y(float angle_deg)
{
    g_angle_y = angle_deg;
    DL_Timer_setCaptureCompareValue(PWM_SERVO_TIMER,
        angle_to_ccr(angle_deg), DL_TIMER_CC_1_INDEX);
}

void gimbal_aiming_update(const openmv_data_t *target)
{
    if (target == NULL || !target->detected) return;

    float correction_x = pid_update_positional(&g_pid_x, (float)target->cx);
    float correction_y = pid_update_positional(&g_pid_y, (float)target->cy);

    gimbal_set_angle_x(g_angle_x + correction_x);
    gimbal_set_angle_y(g_angle_y + correction_y);
}

void gimbal_circle_update(float car_progress_rad)
{
    float dx = CIRCLE_DRAW_RADIUS_CM * 2.0f * cosf(car_progress_rad);
    float dy = CIRCLE_DRAW_RADIUS_CM * 2.0f * sinf(car_progress_rad);
    gimbal_set_angle_x(90.0f + dx);
    gimbal_set_angle_y(90.0f + dy);
}

void gimbal_enable_laser(bool enable)
{
    if (enable) {
        DL_GPIO_setPins(GPIO_LASER_PORT, LASER_PIN);
    } else {
        DL_GPIO_clearPins(GPIO_LASER_PORT, LASER_PIN);
    }
}

void gimbal_reset(void)
{
    g_angle_x = 90.0f;
    g_angle_y = 90.0f;
    gimbal_set_angle_x(90.0f);
    gimbal_set_angle_y(90.0f);
    pid_reset(&g_pid_x);
    pid_reset(&g_pid_y);
    gimbal_enable_laser(false);
}
