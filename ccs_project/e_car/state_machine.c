#include "state_machine.h"
#include "motor.h"
#include "encoder.h"
#include "gimbal.h"
#include "ti_msp_dl_config.h"

static system_state_t g_state;
static system_mode_t  g_mode;
static uint8_t        g_laps;
static uint8_t        g_lap_count;
static uint32_t       g_start_ms;
static bool           g_lap_edge;

void sm_init(void)
{
    g_state = STATE_IDLE;
    g_mode = MODE_TRACK_ONLY;
    g_laps = 1;
    g_lap_count = 0;
    g_start_ms = 0;
    g_lap_edge = false;
}

system_state_t sm_get_state(void)  { return g_state; }
system_mode_t  sm_get_mode(void)   { return g_mode; }
uint8_t        sm_get_laps(void)   { return g_laps; }

void sm_set_laps(uint8_t n)
{
    if (n >= 1 && n <= MAX_LAPS) g_laps = n;
}

void sm_button_mode(void)
{
    if (g_state != STATE_IDLE) return;
    g_mode = (system_mode_t)((uint8_t)g_mode + 1);
    if (g_mode >= MODE_COUNT) g_mode = MODE_TRACK_ONLY;

    for (int i = 0; i < (int)g_mode + 1; i++) {
        DL_GPIO_setPins(GPIO_LED_PORT, LED_PIN);
        for (volatile uint32_t d = 0; d < 80000; d++);
        DL_GPIO_clearPins(GPIO_LED_PORT, LED_PIN);
        for (volatile uint32_t d = 0; d < 120000; d++);
    }
}

void sm_button_start(void)
{
    if (g_state == STATE_STOP) {
        sm_init();
        return;
    }
    if (g_state != STATE_IDLE) return;

    encoder_reset_distance();

    switch (g_mode) {
    case MODE_TRACK_ONLY:
        g_state = STATE_TRACK_ONLY;
        gimbal_enable_laser(false);
        g_laps = 1;
        break;
    case MODE_AIM_STATIC:
        g_state = STATE_AIM_STATIC;
        gimbal_enable_laser(true);
        break;
    case MODE_TRACK_AIM_N1:
        g_state = STATE_TRACK_AIM;
        gimbal_enable_laser(true);
        g_laps = 1;
        break;
    case MODE_TRACK_AIM_N2:
        g_state = STATE_TRACK_AIM;
        gimbal_enable_laser(true);
        g_laps = 2;
        break;
    case MODE_TRACK_DRAW:
        g_state = STATE_TRACK_DRAW;
        gimbal_enable_laser(true);
        g_laps = 1;
        break;
    default:
        return;
    }

    g_lap_count = 0;
    g_start_ms = 0;
    g_lap_edge = false;
    DL_GPIO_setPins(GPIO_LED_PORT, LED_PIN);
}

static void detect_lap(void)
{
    float avg_dist = (encoder_left_get_distance() + encoder_right_get_distance()) / 2.0f;
    float lap_dist = LAP_CIRCUMFERENCE_M;

    if (avg_dist >= lap_dist * (float)(g_lap_count + 1) && !g_lap_edge) {
        g_lap_count++;
        g_lap_edge = true;
    }
    if (avg_dist < lap_dist * (float)g_lap_count - 0.1f) {
        g_lap_edge = false;
    }
}

void sm_update(uint32_t sys_ms)
{
    if (g_state == STATE_IDLE || g_state == STATE_STOP) return;

    if (g_state == STATE_TRACK_ONLY ||
        g_state == STATE_TRACK_AIM ||
        g_state == STATE_TRACK_DRAW) {

        if (g_start_ms == 0) g_start_ms = sys_ms;

        detect_lap();

        float elapsed = (float)(sys_ms - g_start_ms) / 1000.0f;
        if (g_lap_count >= g_laps ||
            elapsed > (float)g_laps * MAX_LAP_TIME_S) {
            g_state = STATE_STOP;
            motor_stop();
            gimbal_enable_laser(false);
            DL_GPIO_clearPins(GPIO_LED_PORT, LED_PIN);
        }
    }
}
