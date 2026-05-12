#include "line_track.h"
#include "pid.h"
#include "ti_msp_dl_config.h"

static pid_controller_t g_pos_pid;
static uint16_t g_sensor[LINE_SENSOR_COUNT];
static float g_position;

void line_track_init(void)
{
    pid_init(&g_pos_pid, POSITION_PID_KP, 0.0f, POSITION_PID_KD,
             0.0f, POSITION_PID_OUT_MAX, 0.01f);
    pid_set_setpoint(&g_pos_pid, 0.0f);

    for (int i = 0; i < LINE_SENSOR_COUNT; i++) {
        g_sensor[i] = 0;
    }
    g_position = 0.0f;
}

void line_track_read(uint16_t sensor_values[LINE_SENSOR_COUNT])
{
    uint32_t sum_num = 0;
    uint32_t sum_den = 0;

    for (int i = 0; i < LINE_SENSOR_COUNT; i++) {
        g_sensor[i] = sensor_values[i];
        if (sensor_values[i] > LINE_SENSOR_WHITE_THRESH) {
            sum_num += (uint32_t)sensor_values[i] * (i + 1);
            sum_den += sensor_values[i];
        }
    }

    if (sum_den > 0) {
        float center = (float)sum_num / (float)sum_den;
        g_position = center - (LINE_SENSOR_COUNT + 1.0f) / 2.0f;
    }
}

float line_track_get_position(void)
{
    return g_position;
}

float line_track_update_position_pid(float position_error)
{
    return pid_update_positional(&g_pos_pid, position_error);
}
