#ifndef SIMPLE_LINE_CONTROL_H
#define SIMPLE_LINE_CONTROL_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    int32_t left_base;
    int32_t right_base;
    int32_t pwm_min;
    int32_t pwm_max;
    int32_t kp_divisor;
    int32_t correction_limit;
    int32_t lost_search_correction;
    uint32_t lost_timeout_frames;
} simple_line_config_t;

typedef struct {
    uint32_t left_pwm;
    uint32_t right_pwm;
    int32_t correction;
    uint32_t lost_count;
    bool stop;
} simple_line_output_t;

static inline int32_t simple_line_clamp_int32(
    int32_t value,
    int32_t minimum,
    int32_t maximum)
{
    if (value < minimum) {
        return minimum;
    }
    if (value > maximum) {
        return maximum;
    }
    return value;
}

static inline uint32_t simple_line_clamp_pwm(
    int32_t value,
    const simple_line_config_t *config)
{
    if (value <= 0) {
        return 0U;
    }
    if (value < config->pwm_min) {
        return (uint32_t) config->pwm_min;
    }
    if (value > config->pwm_max) {
        return (uint32_t) config->pwm_max;
    }
    return (uint32_t) value;
}

static inline simple_line_output_t simple_line_step(
    const simple_line_config_t *config,
    bool line_valid,
    int32_t line_error,
    int32_t last_nonzero_error,
    uint32_t previous_lost_count)
{
    simple_line_output_t output = {0U, 0U, 0, 0U, false};

    if (line_valid) {
        output.correction = simple_line_clamp_int32(
            line_error / config->kp_divisor,
            -config->correction_limit,
            config->correction_limit);
    } else {
        output.lost_count = previous_lost_count + 1U;
        if (last_nonzero_error > 0) {
            output.correction = config->lost_search_correction;
        } else if (last_nonzero_error < 0) {
            output.correction = -config->lost_search_correction;
        }
    }

    output.stop = output.lost_count >= config->lost_timeout_frames;
    if (output.stop) {
        return output;
    }

    output.left_pwm = simple_line_clamp_pwm(
        config->left_base + output.correction,
        config);
    output.right_pwm = simple_line_clamp_pwm(
        config->right_base - output.correction,
        config);
    return output;
}

#endif
