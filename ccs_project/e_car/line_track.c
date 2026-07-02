#include "line_track.h"
#include "board.h"
#include "pid.h"

static const float k_sensor_weight[LINE_SENSOR_COUNT] = {
    -3.0f, -2.0f, -1.0f, 0.0f, 1.0f, 2.0f, 3.0f
};

static pid_controller_t g_pos_pid;
static uint8_t g_line_mask;
static float g_position;
static bool g_lost;

void line_track_init(void)
{
    pid_init(&g_pos_pid, POSITION_PID_KP, 0.0f, POSITION_PID_KD,
        0.0f, POSITION_PID_OUT_MAX, 0.0f);
    pid_set_setpoint(&g_pos_pid, 0.0f);
    g_line_mask = 0U;
    g_position = 0.0f;
    g_lost = true;
}

void line_track_update(void)
{
    uint8_t mask = board_read_line_mask();
    float sum = 0.0f;
    float count = 0.0f;

    g_line_mask = mask;

    for (uint8_t i = 0U; i < LINE_SENSOR_COUNT; i++) {
        if ((mask & (uint8_t)(1U << i)) != 0U) {
            sum += k_sensor_weight[i];
            count += 1.0f;
        }
    }

    if (count > 0.0f) {
        g_position = sum / count;
        g_lost = false;
    } else {
        g_lost = true;
    }
}

uint8_t line_track_get_mask(void)
{
    return g_line_mask;
}

float line_track_get_position(void)
{
    return g_position;
}

bool line_track_is_lost(void)
{
    return g_lost;
}

float line_track_update_position_pid(float position_error)
{
    return pid_update_positional(&g_pos_pid, position_error);
}
