#include "motor.h"
#include "encoder.h"
#include "pid.h"
#include <math.h>
#include "ti_msp_dl_config.h"

static pid_controller_t g_speed_pid_l;
static pid_controller_t g_speed_pid_r;
static float g_target_l_mps;
static float g_target_r_mps;
static uint32_t g_last_ms;
static float g_pwm_l;
static float g_pwm_r;

static uint32_t duty_to_ccr(float duty)
{
    if (duty > 100.0f) duty = 100.0f;
    if (duty < -100.0f) duty = -100.0f;
    return (uint32_t)(fabsf(duty) / 100.0f * (float)PWM_TOP_VALUE);
}

static void write_motor_a(float duty)
{
    uint32_t ccr = duty_to_ccr(duty);
    DL_GPIO_clearPins(GPIO_MOTOR_PORT, MOTOR_AIN1_PIN | MOTOR_AIN2_PIN);
    if (duty >= 0.0f) {
        DL_GPIO_setPins(GPIO_MOTOR_PORT, MOTOR_AIN1_PIN);
    } else {
        DL_GPIO_setPins(GPIO_MOTOR_PORT, MOTOR_AIN2_PIN);
    }
    DL_Timer_setCaptureCompareValue(PWM_MOTOR_TIMER, ccr, DL_TIMER_CC_0_INDEX);
}

static void write_motor_b(float duty)
{
    uint32_t ccr = duty_to_ccr(duty);
    DL_GPIO_clearPins(GPIO_MOTOR_PORT, MOTOR_BIN1_PIN | MOTOR_BIN2_PIN);
    if (duty >= 0.0f) {
        DL_GPIO_setPins(GPIO_MOTOR_PORT, MOTOR_BIN1_PIN);
    } else {
        DL_GPIO_setPins(GPIO_MOTOR_PORT, MOTOR_BIN2_PIN);
    }
    DL_Timer_setCaptureCompareValue(PWM_MOTOR_TIMER, ccr, DL_TIMER_CC_1_INDEX);
}

void motor_init(void)
{
    DL_GPIO_setPins(GPIO_MOTOR_PORT, MOTOR_STBY_PIN);

    pid_init(&g_speed_pid_l, SPEED_PID_KP, SPEED_PID_KI, 0.0f,
             SPEED_PID_I_MAX, SPEED_PID_OUT_MAX, 0.0f);
    pid_init(&g_speed_pid_r, SPEED_PID_KP, SPEED_PID_KI, 0.0f,
             SPEED_PID_I_MAX, SPEED_PID_OUT_MAX, 0.0f);

    g_target_l_mps = 0.0f;
    g_target_r_mps = 0.0f;
    g_pwm_l = 0.0f;
    g_pwm_r = 0.0f;
    g_last_ms = 0;

    DL_TimerG_startCounter(PWM_MOTOR_TIMER);
    motor_stop();
}

void motor_drive(float left_duty, float right_duty)
{
    g_pwm_l = left_duty;
    g_pwm_r = right_duty;
    write_motor_a(left_duty);
    write_motor_b(right_duty);
}

void motor_stop(void)
{
    DL_Timer_setCaptureCompareValue(PWM_MOTOR_TIMER, 0, DL_TIMER_CC_0_INDEX);
    DL_Timer_setCaptureCompareValue(PWM_MOTOR_TIMER, 0, DL_TIMER_CC_1_INDEX);
    DL_GPIO_clearPins(GPIO_MOTOR_PORT,
        MOTOR_AIN1_PIN | MOTOR_AIN2_PIN | MOTOR_BIN1_PIN | MOTOR_BIN2_PIN);
    g_pwm_l = 0.0f;
    g_pwm_r = 0.0f;
}

void motor_brake(void)
{
    DL_Timer_setCaptureCompareValue(PWM_MOTOR_TIMER,
        PWM_TOP_VALUE, DL_TIMER_CC_0_INDEX);
    DL_Timer_setCaptureCompareValue(PWM_MOTOR_TIMER,
        PWM_TOP_VALUE, DL_TIMER_CC_1_INDEX);
    DL_GPIO_setPins(GPIO_MOTOR_PORT,
        MOTOR_AIN1_PIN | MOTOR_AIN2_PIN | MOTOR_BIN1_PIN | MOTOR_BIN2_PIN);
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
    if (g_last_ms == 0) {
        g_last_ms = sys_time_ms;
        return;
    }

    float left_speed = encoder_left_get_speed();
    float right_speed = encoder_right_get_speed();

    pid_update_incremental(&g_speed_pid_l, left_speed, &g_pwm_l);
    pid_update_incremental(&g_speed_pid_r, right_speed, &g_pwm_r);

    write_motor_a(g_pwm_l);
    write_motor_b(g_pwm_r);
}
