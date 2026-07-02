#include "motor.h"
#include "board.h"
#include "encoder.h"
#include "pid.h"

static pid_controller_t g_speed_pid_l;
static pid_controller_t g_speed_pid_r;
static float g_target_l_mps;
static float g_target_r_mps;
static float g_pwm_l;
static float g_pwm_r;

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

static uint16_t duty_to_ticks(float duty)
{
    if (duty < 0.0f) {
        duty = -duty;
    }
    duty = clampf(duty, 0.0f, 100.0f);
    return (uint16_t)((duty * (float)MOTOR_PWM_PERIOD) / 100.0f);
}

static void write_motor_a(float duty)
{
    DL_GPIO_clearPins(MOTOR_DIR_PORT, MOTOR_AIN1_PIN | MOTOR_AIN2_PIN);
    if (duty > 0.0f) {
        DL_GPIO_setPins(MOTOR_DIR_PORT, MOTOR_AIN1_PIN);
    } else if (duty < 0.0f) {
        DL_GPIO_setPins(MOTOR_DIR_PORT, MOTOR_AIN2_PIN);
    }
}

static void write_motor_b(float duty)
{
    DL_GPIO_clearPins(MOTOR_DIR_PORT, MOTOR_BIN1_PIN | MOTOR_BIN2_PIN);
    if (duty > 0.0f) {
        DL_GPIO_setPins(MOTOR_DIR_PORT, MOTOR_BIN1_PIN);
    } else if (duty < 0.0f) {
        DL_GPIO_setPins(MOTOR_DIR_PORT, MOTOR_BIN2_PIN);
    }
}

static void apply_motor_output(float left_duty, float right_duty)
{
    g_pwm_l = clampf(left_duty, -100.0f, 100.0f);
    g_pwm_r = clampf(right_duty, -100.0f, 100.0f);

    write_motor_a(g_pwm_l);
    write_motor_b(g_pwm_r);
    board_set_motor_pwm(duty_to_ticks(g_pwm_l), duty_to_ticks(g_pwm_r));
}

void motor_init(void)
{
    board_set_motor_standby(true);
    pid_init(&g_speed_pid_l, SPEED_PID_KP, SPEED_PID_KI, 0.0f,
        SPEED_PID_I_MAX, SPEED_PID_OUT_MAX, 0.0f);
    pid_init(&g_speed_pid_r, SPEED_PID_KP, SPEED_PID_KI, 0.0f,
        SPEED_PID_I_MAX, SPEED_PID_OUT_MAX, 0.0f);
    g_target_l_mps = 0.0f;
    g_target_r_mps = 0.0f;
    g_pwm_l = 0.0f;
    g_pwm_r = 0.0f;
    motor_stop();
}

void motor_drive(float left_duty, float right_duty)
{
    apply_motor_output(left_duty, right_duty);
}

void motor_stop(void)
{
    apply_motor_output(0.0f, 0.0f);
}

void motor_brake(void)
{
    DL_GPIO_setPins(MOTOR_DIR_PORT,
        MOTOR_AIN1_PIN | MOTOR_AIN2_PIN | MOTOR_BIN1_PIN | MOTOR_BIN2_PIN);
    board_set_motor_pwm(MOTOR_PWM_PERIOD, MOTOR_PWM_PERIOD);
    g_pwm_l = 0.0f;
    g_pwm_r = 0.0f;
}

void motor_set_target(float left_mps, float right_mps)
{
    g_target_l_mps = left_mps;
    g_target_r_mps = right_mps;
    pid_set_setpoint(&g_speed_pid_l, left_mps);
    pid_set_setpoint(&g_speed_pid_r, right_mps);
}

void motor_update_speed_pid(uint32_t sys_time_ms)
{
    (void)sys_time_ms;

    if (g_target_l_mps == 0.0f && g_target_r_mps == 0.0f) {
        motor_stop();
        return;
    }

    pid_update_incremental(&g_speed_pid_l, encoder_left_get_speed(), &g_pwm_l);
    pid_update_incremental(&g_speed_pid_r, encoder_right_get_speed(), &g_pwm_r);
    apply_motor_output(g_pwm_l, g_pwm_r);
}
