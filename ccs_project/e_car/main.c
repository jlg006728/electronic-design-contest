#include "ti_msp_dl_config.h"
#include "board.h"
#include "config.h"
#include "encoder.h"
#include "gimbal.h"
#include "line_track.h"
#include "motor.h"
#include "openmv.h"
#include "state_machine.h"

static volatile uint32_t g_sys_tick_ms = 0U;

void SysTick_Handler(void)
{
    g_sys_tick_ms++;
}

uint32_t get_sys_time_ms(void)
{
    return g_sys_tick_ms;
}

static void run_line_follow_control(system_state_t state)
{
    if (state != STATE_TRACK_ONLY &&
        state != STATE_TRACK_AIM &&
        state != STATE_TRACK_DRAW) {
        motor_stop();
        return;
    }

    if (line_track_is_lost()) {
        float last_position = line_track_get_position();
        if (last_position >= 0.0f) {
            motor_drive(LINE_LOST_TURN_DUTY, -LINE_LOST_TURN_DUTY);
        } else {
            motor_drive(-LINE_LOST_TURN_DUTY, LINE_LOST_TURN_DUTY);
        }
        return;
    }

    float position = line_track_get_position();
    float correction = line_track_update_position_pid(position);
    motor_drive(TRACK_OPEN_LOOP_DUTY - correction, TRACK_OPEN_LOOP_DUTY + correction);
}

static void run_gimbal_control(system_state_t state)
{
    if (state == STATE_AIM_STATIC || state == STATE_TRACK_AIM) {
        openmv_request();
        if (openmv_data_ready()) {
            openmv_data_t target = openmv_get_data();
            gimbal_aiming_update(&target);
        }
    } else if (state == STATE_TRACK_DRAW) {
        float avg_dist = (encoder_left_get_distance() + encoder_right_get_distance()) * 0.5f;
        float progress = (avg_dist / LAP_CIRCUMFERENCE_M) * 2.0f * PI_F;
        while (progress > (2.0f * PI_F)) {
            progress -= 2.0f * PI_F;
        }
        gimbal_circle_update(progress);
    }
}

int main(void)
{
    SYSCFG_DL_init();
    board_init();

    SysTick_Config(SYSCLK_HZ / 1000U);

    encoder_left_init();
    encoder_right_init();
    motor_init();
    line_track_init();
    openmv_init();
    gimbal_init();
    sm_init();

    uint32_t last_line_ms = 0U;
    uint32_t last_control_ms = 0U;
    uint32_t last_gimbal_ms = 0U;
    uint32_t last_odom_ms = 0U;
    uint32_t last_state_ms = 0U;

    while (1) {
        uint32_t now = get_sys_time_ms();
        system_state_t state = sm_get_state();

        if ((now - last_line_ms) >= (1000U / SENSOR_SAMPLE_HZ)) {
            last_line_ms = now;
            line_track_update();
        }

        if ((now - last_control_ms) >= (1000U / POSITION_PID_HZ)) {
            last_control_ms = now;
            run_line_follow_control(state);
        }

        if ((now - last_gimbal_ms) >= (1000U / GIMBAL_PID_HZ)) {
            last_gimbal_ms = now;
            run_gimbal_control(state);
        }

        if ((now - last_odom_ms) >= (1000U / ODOMETRY_HZ)) {
            last_odom_ms = now;
            encoder_update(now);
        }

        if ((now - last_state_ms) >= (1000U / STATE_MACHINE_HZ)) {
            last_state_ms = now;
            sm_update(now);
        }
    }
}
