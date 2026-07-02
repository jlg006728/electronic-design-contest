#include "state_machine.h"
#include "board.h"
#include "encoder.h"
#include "gimbal.h"
#include "motor.h"

static system_state_t g_state;
static system_mode_t g_mode;
static uint8_t g_laps;
static uint8_t g_lap_count;
static uint32_t g_start_ms;
static uint32_t g_last_button_ms;

void sm_init(void)
{
    g_state = STATE_IDLE;
    g_mode = MODE_TRACK_ONLY;
    g_laps = 1U;
    g_lap_count = 0U;
    g_start_ms = 0U;
    g_last_button_ms = 0U;
    board_led_set(false);
}

system_state_t sm_get_state(void)
{
    return g_state;
}

system_mode_t sm_get_mode(void)
{
    return g_mode;
}

uint8_t sm_get_laps(void)
{
    return g_laps;
}

uint8_t sm_get_lap_count(void)
{
    return g_lap_count;
}

void sm_set_laps(uint8_t n)
{
    if (n >= 1U && n <= MAX_LAPS) {
        g_laps = n;
    }
}

static bool debounce_elapsed(void)
{
    extern uint32_t get_sys_time_ms(void);
    uint32_t now = get_sys_time_ms();
    if ((now - g_last_button_ms) < 200U) {
        return false;
    }
    g_last_button_ms = now;
    return true;
}

void sm_button_mode(void)
{
    if (!debounce_elapsed() || g_state != STATE_IDLE) {
        return;
    }

    g_mode = (system_mode_t)((uint8_t)g_mode + 1U);
    if (g_mode >= MODE_COUNT) {
        g_mode = MODE_TRACK_ONLY;
    }
}

void sm_button_start(void)
{
    if (!debounce_elapsed()) {
        return;
    }

    if (g_state == STATE_STOP) {
        motor_stop();
        gimbal_reset();
        sm_init();
        return;
    }
    if (g_state != STATE_IDLE) {
        g_state = STATE_STOP;
        motor_stop();
        gimbal_enable_laser(false);
        board_led_set(false);
        return;
    }

    encoder_reset_distance();
    g_lap_count = 0U;
    g_start_ms = 0U;

    switch (g_mode) {
    case MODE_TRACK_ONLY:
        g_state = STATE_TRACK_ONLY;
        g_laps = 1U;
        gimbal_enable_laser(false);
        break;
    case MODE_AIM_STATIC:
        g_state = STATE_AIM_STATIC;
        gimbal_enable_laser(true);
        break;
    case MODE_TRACK_AIM_N1:
        g_state = STATE_TRACK_AIM;
        g_laps = 1U;
        gimbal_enable_laser(true);
        break;
    case MODE_TRACK_AIM_N2:
        g_state = STATE_TRACK_AIM;
        g_laps = 2U;
        gimbal_enable_laser(true);
        break;
    case MODE_TRACK_DRAW:
        g_state = STATE_TRACK_DRAW;
        g_laps = 1U;
        gimbal_enable_laser(true);
        break;
    default:
        g_state = STATE_IDLE;
        return;
    }

    board_led_set(true);
}

static void update_lap_count(void)
{
    float avg_dist = (encoder_left_get_distance() + encoder_right_get_distance()) * 0.5f;
    uint8_t laps = (uint8_t)(avg_dist / LAP_CIRCUMFERENCE_M);
    if (laps > g_lap_count) {
        g_lap_count = laps;
    }
}

void sm_update(uint32_t sys_ms)
{
    if (g_state == STATE_IDLE) {
        if (((sys_ms / 250U) % ((uint32_t)MODE_COUNT + 1U)) == (uint32_t)g_mode) {
            board_led_set(true);
        } else {
            board_led_set(false);
        }
        return;
    }
    if (g_state == STATE_STOP) {
        board_led_set(false);
        return;
    }

    if (g_start_ms == 0U) {
        g_start_ms = sys_ms;
    }

    if (g_state == STATE_TRACK_ONLY ||
        g_state == STATE_TRACK_AIM ||
        g_state == STATE_TRACK_DRAW) {
        update_lap_count();
        float elapsed_s = (float)(sys_ms - g_start_ms) / 1000.0f;
        if (g_lap_count >= g_laps || elapsed_s > ((float)g_laps * MAX_LAP_TIME_S)) {
            g_state = STATE_STOP;
            motor_stop();
            gimbal_enable_laser(false);
            board_led_set(false);
        }
    }
}
