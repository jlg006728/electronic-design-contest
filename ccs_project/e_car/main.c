#include "ti_msp_dl_config.h"
#include "config.h"
#include "pid.h"
#include "encoder.h"
#include "motor.h"
#include "line_track.h"
#include "openmv.h"
#include "gimbal.h"
#include "state_machine.h"
#include <stdbool.h>
#include <math.h>

static volatile uint32_t g_sys_tick_ms = 0;

static void systick_init(void)
{
    /* 使用TIMG4作为系统时基, 1kHz中断 */
    SysTick_Config(80000000 / 1000);
}

void SysTick_Handler(void)
{
    g_sys_tick_ms++;
}

uint32_t get_sys_time_ms(void)
{
    return g_sys_tick_ms;
}

static void adc_sample_sensors(uint16_t *values)
{
    for (int i = 0; i < LINE_SENSOR_COUNT; i++) {
        DL_ADC12_startConversion(ADC12_INST);
        while (DL_ADC12_getPendingInterrupt(ADC12_INST) != DL_ADC12_IIDX_MEM0_RESULT_LOADED);
        values[i] = DL_ADC12_getMemResult(ADC12_INST, DL_ADC12_MEM_IDX_0);
    }
}

static void btn_isr_mode(void)
{
    sm_button_mode();
}

static void btn_isr_start(void)
{
    sm_button_start();
}

int main(void)
{
    SYSCFG_DL_init();

    /* --- 模块初始化 --- */
    systick_init();
    sm_init();
    encoder_left_init();
    encoder_right_init();
    motor_init();
    line_track_init();
    openmv_init();
    gimbal_init();

    /* --- 主循环: 定时任务调度 --- */
    uint32_t last_sensor_ms = 0;
    uint32_t last_position_ms = 0;
    uint32_t last_gimbal_ms = 0;
    uint32_t last_openmv_ms = 0;
    uint32_t last_state_ms = 0;
    uint32_t last_odom_ms = 0;

    while (1) {
        uint32_t now = get_sys_time_ms();
        system_state_t state = sm_get_state();

        /* 1kHz: 传感器采样 + 速度PID */
        if (now - last_sensor_ms >= (1000 / SENSOR_SAMPLE_HZ)) {
            last_sensor_ms = now;

            uint16_t sensor_values[LINE_SENSOR_COUNT];
            adc_sample_sensors(sensor_values);
            line_track_read(sensor_values);

            if (state != STATE_IDLE && state != STATE_STOP && state != STATE_AIM_STATIC) {
                motor_update_speed_pid(now);
            }
        }

        /* 200Hz: 位置PID */
        if (now - last_position_ms >= (1000 / POSITION_PID_HZ)) {
            last_position_ms = now;

            if (state == STATE_TRACK_ONLY ||
                state == STATE_TRACK_AIM ||
                state == STATE_TRACK_DRAW) {

                float pos = line_track_get_position();
                float turn_correction = line_track_update_position_pid(pos);

                float base_speed = TARGET_SPEED_M_PER_S;
                float left_target = base_speed + turn_correction * base_speed;
                float right_target = base_speed - turn_correction * base_speed;

                motor_set_target(left_target, right_target);
            }
        }

        /* 50Hz: 云台 + OpenMV */
        if (now - last_gimbal_ms >= (1000 / GIMBAL_PID_HZ)) {
            last_gimbal_ms = now;

            if (state == STATE_TRACK_AIM ||
                state == STATE_AIM_STATIC) {
                openmv_request();
            }

            if (openmv_data_ready()) {
                openmv_data_t data = openmv_get_data();
                if (state == STATE_TRACK_AIM ||
                    state == STATE_AIM_STATIC) {
                    gimbal_aiming_update(&data);
                }
            }

            if (state == STATE_TRACK_DRAW) {
                float avg_dist = (encoder_left_get_distance() +
                                  encoder_right_get_distance()) / 2.0f;
                float progress = (avg_dist / LAP_CIRCUMFERENCE_M) * 2.0f * 3.14159265f;
                while (progress > 2.0f * 3.14159265f) {
                    progress -= 2.0f * 3.14159265f;
                }
                gimbal_circle_update(progress);
            }
        }

        /* 100Hz: 里程计 */
        if (now - last_odom_ms >= (1000 / ODOMETRY_HZ)) {
            last_odom_ms = now;
            encoder_update(now);
        }

        /* 10Hz: 状态机 */
        if (now - last_state_ms >= (1000 / STATE_MACHINE_HZ)) {
            last_state_ms = now;
            sm_update(now);
        }
    }
}
