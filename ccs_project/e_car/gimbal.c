#include "gimbal.h"
#include "board.h"
#include "pid.h"

static pid_controller_t g_pid_x;
static pid_controller_t g_pid_y;
static float g_angle_x;
static float g_angle_y;

static float clampf(float value, float min_value, float max_value)
{
    if (value < min_value) {
        return min_value;
    }
    if (value > max_value) {
        return max_value;
    }
    return value;
}

static uint16_t angle_to_us(float angle_deg)
{
    angle_deg = clampf(angle_deg, SERVO_MIN_DEG, SERVO_MAX_DEG);
    float span = (float)(SERVO_MAX_US - SERVO_MIN_US);
    return (uint16_t)((float)SERVO_MIN_US + span * angle_deg / 180.0f);
}

static float fast_sin(float x)
{
    while (x > PI_F) {
        x -= 2.0f * PI_F;
    }
    while (x < -PI_F) {
        x += 2.0f * PI_F;
    }
    float x2 = x * x;
    return x * (1.0f - x2 / 6.0f + (x2 * x2) / 120.0f);
}

static float fast_cos(float x)
{
    return fast_sin(x + (PI_F * 0.5f));
}

void gimbal_init(void)
{
    g_angle_x = GIMBAL_X_CENTER_DEG;
    g_angle_y = GIMBAL_Y_CENTER_DEG;

    pid_init(&g_pid_x, GIMBAL_X_PID_KP, GIMBAL_X_PID_KI, GIMBAL_X_PID_KD,
        GIMBAL_X_PID_I_MAX, GIMBAL_X_PID_OUT_MAX, 2.0f);
    pid_set_setpoint(&g_pid_x, GIMBAL_IMAGE_CENTER_X);

    pid_init(&g_pid_y, GIMBAL_Y_PID_KP, GIMBAL_Y_PID_KI, GIMBAL_Y_PID_KD,
        GIMBAL_Y_PID_I_MAX, GIMBAL_Y_PID_OUT_MAX, 2.0f);
    pid_set_setpoint(&g_pid_y, GIMBAL_IMAGE_CENTER_Y);

    gimbal_set_angle_x(g_angle_x);
    gimbal_set_angle_y(g_angle_y);
    gimbal_enable_laser(false);
}

void gimbal_set_angle_x(float angle_deg)
{
    g_angle_x = clampf(angle_deg, GIMBAL_X_ANGLE_MIN_DEG, GIMBAL_X_ANGLE_MAX_DEG);
    board_set_servo_us(0U, angle_to_us(g_angle_x));
}

void gimbal_set_angle_y(float angle_deg)
{
    g_angle_y = clampf(angle_deg, GIMBAL_Y_ANGLE_MIN_DEG, GIMBAL_Y_ANGLE_MAX_DEG);
    board_set_servo_us(1U, angle_to_us(g_angle_y));
}

void gimbal_aiming_update(const openmv_data_t *target)
{
    if (target == 0 || !target->detected) {
        return;
    }

    /*
     * Pixel correction signs depend on the physical gimbal build. If aiming
     * moves away from the red dot, swap the signs of these two increments.
     */
    float dx = pid_update_positional(&g_pid_x, (float)target->cx);
    float dy = pid_update_positional(&g_pid_y, (float)target->cy);

    gimbal_set_angle_x(g_angle_x + dx);
    gimbal_set_angle_y(g_angle_y - dy);
}

void gimbal_circle_update(float car_progress_rad)
{
    float servo_radius_deg = 8.0f;
    float x = GIMBAL_X_CENTER_DEG + servo_radius_deg * fast_cos(car_progress_rad);
    float y = GIMBAL_Y_CENTER_DEG + servo_radius_deg * fast_sin(car_progress_rad);
    gimbal_set_angle_x(x);
    gimbal_set_angle_y(y);
}

void gimbal_enable_laser(bool enable)
{
    board_laser_set(enable);
}

void gimbal_reset(void)
{
    pid_reset(&g_pid_x);
    pid_reset(&g_pid_y);
    gimbal_set_angle_x(GIMBAL_X_CENTER_DEG);
    gimbal_set_angle_y(GIMBAL_Y_CENTER_DEG);
    gimbal_enable_laser(false);
}

float gimbal_get_angle_x(void)
{
    return g_angle_x;
}

float gimbal_get_angle_y(void)
{
    return g_angle_y;
}
