#include "encoder.h"
#include "board.h"

encoder_t g_encoder_left;
encoder_t g_encoder_right;

static void encoder_common_init(encoder_t *enc)
{
    enc->pulse_count = 0;
    enc->speed_rpm = 0.0f;
    enc->speed_m_per_s = 0.0f;
    enc->distance_m = 0.0f;
    enc->last_time_ms = 0U;
    enc->last_pulse_count = 0;
}

void encoder_left_init(void)
{
    encoder_common_init(&g_encoder_left);
}

void encoder_right_init(void)
{
    encoder_common_init(&g_encoder_right);
}

void encoder_left_isr(void)
{
    bool a = board_encoder_left_a_high();
    bool b = board_encoder_left_b_high();

    if (a != b) {
        g_encoder_left.pulse_count++;
    } else {
        g_encoder_left.pulse_count--;
    }
}

void encoder_right_isr(void)
{
    bool a = board_encoder_right_a_high();
    bool b = board_encoder_right_b_high();

    if (a != b) {
        g_encoder_right.pulse_count++;
    } else {
        g_encoder_right.pulse_count--;
    }
}

void encoder_update(uint32_t sys_time_ms)
{
    if (g_encoder_left.last_time_ms == 0U) {
        g_encoder_left.last_time_ms = sys_time_ms;
        g_encoder_right.last_time_ms = sys_time_ms;
        return;
    }

    float dt = (float)(sys_time_ms - g_encoder_left.last_time_ms) / 1000.0f;
    if (dt <= 0.0f) {
        return;
    }

    int32_t delta_l = g_encoder_left.pulse_count - g_encoder_left.last_pulse_count;
    int32_t delta_r = g_encoder_right.pulse_count - g_encoder_right.last_pulse_count;

    g_encoder_left.last_pulse_count = g_encoder_left.pulse_count;
    g_encoder_right.last_pulse_count = g_encoder_right.pulse_count;

    g_encoder_left.speed_rpm = (float)delta_l * 60.0f / (ENCODER_COUNTS_PER_REV * dt);
    g_encoder_right.speed_rpm = (float)delta_r * 60.0f / (ENCODER_COUNTS_PER_REV * dt);
    g_encoder_left.speed_m_per_s =
        (float)delta_l * WHEEL_CIRCUMFERENCE_M / (ENCODER_COUNTS_PER_REV * dt);
    g_encoder_right.speed_m_per_s =
        (float)delta_r * WHEEL_CIRCUMFERENCE_M / (ENCODER_COUNTS_PER_REV * dt);

    g_encoder_left.distance_m +=
        (float)delta_l * WHEEL_CIRCUMFERENCE_M / ENCODER_COUNTS_PER_REV;
    g_encoder_right.distance_m +=
        (float)delta_r * WHEEL_CIRCUMFERENCE_M / ENCODER_COUNTS_PER_REV;

    g_encoder_left.last_time_ms = sys_time_ms;
    g_encoder_right.last_time_ms = sys_time_ms;
}

float encoder_left_get_speed(void)
{
    return g_encoder_left.speed_m_per_s;
}

float encoder_right_get_speed(void)
{
    return g_encoder_right.speed_m_per_s;
}

float encoder_left_get_distance(void)
{
    return g_encoder_left.distance_m;
}

float encoder_right_get_distance(void)
{
    return g_encoder_right.distance_m;
}

void encoder_reset_distance(void)
{
    g_encoder_left.distance_m = 0.0f;
    g_encoder_right.distance_m = 0.0f;
    g_encoder_left.pulse_count = 0;
    g_encoder_right.pulse_count = 0;
    g_encoder_left.last_pulse_count = 0;
    g_encoder_right.last_pulse_count = 0;
}
