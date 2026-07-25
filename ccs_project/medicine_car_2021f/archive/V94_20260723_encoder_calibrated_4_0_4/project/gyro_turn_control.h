#ifndef GYRO_TURN_CONTROL_H
#define GYRO_TURN_CONTROL_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    GYRO_TURN_PHASE_CENTER = 0,
    GYRO_TURN_PHASE_SETTLE,
    GYRO_TURN_PHASE_ROTATE,
    GYRO_TURN_PHASE_COMPLETE,
    GYRO_TURN_PHASE_FAULT
} gyro_turn_phase_t;

typedef enum {
    GYRO_TURN_MOTION_STOP = 0,
    GYRO_TURN_MOTION_FORWARD,
    GYRO_TURN_MOTION_SPIN_LEFT_FAST,
    GYRO_TURN_MOTION_SPIN_LEFT_SLOW,
    GYRO_TURN_MOTION_SPIN_RIGHT_FAST,
    GYRO_TURN_MOTION_SPIN_RIGHT_SLOW,
    GYRO_TURN_MOTION_BRAKE
} gyro_turn_motion_t;

typedef struct {
    uint16_t settle_ms;
    int32_t slow_zone_mdeg;
    int32_t tolerance_mdeg;
    int32_t stopped_rate_limit_mdps;
    uint8_t stable_frames_required;
    uint8_t max_sensor_failures;
} gyro_turn_config_t;

typedef struct {
    gyro_turn_phase_t phase;
    int32_t target_mdeg;
    uint16_t center_forward_ms;
    uint16_t phase_elapsed_ms;
    uint32_t total_elapsed_ms;
    uint32_t timeout_ms;
    uint8_t stable_frames;
    uint8_t sensor_failure_count;
} gyro_turn_state_t;

typedef struct {
    gyro_turn_motion_t motion;
    bool reset_yaw;
    bool complete;
    bool fault;
} gyro_turn_output_t;

static inline uint32_t gyro_turn_abs_i32(int32_t value)
{
    if (value >= 0) {
        return (uint32_t)value;
    }
    return (uint32_t)(-(int64_t)value);
}

static inline void gyro_turn_init(
    gyro_turn_state_t *state,
    int32_t target_mdeg,
    uint16_t center_forward_ms,
    uint32_t timeout_ms)
{
    *state = (gyro_turn_state_t){
        .phase = (center_forward_ms > 0U)
            ? GYRO_TURN_PHASE_CENTER
            : GYRO_TURN_PHASE_SETTLE,
        .target_mdeg = target_mdeg,
        .center_forward_ms = center_forward_ms,
        .timeout_ms = timeout_ms,
    };
}

static inline bool gyro_turn_advance_elapsed(
    gyro_turn_state_t *state,
    uint16_t delta_ms)
{
    if ((UINT32_MAX - state->total_elapsed_ms) < delta_ms) {
        state->total_elapsed_ms = UINT32_MAX;
    } else {
        state->total_elapsed_ms += delta_ms;
    }
    if (state->total_elapsed_ms > state->timeout_ms) {
        state->phase = GYRO_TURN_PHASE_FAULT;
        return false;
    }
    return true;
}

static inline gyro_turn_output_t gyro_turn_sensor_failure(
    const gyro_turn_config_t *config,
    gyro_turn_state_t *state,
    uint16_t delta_ms)
{
    gyro_turn_output_t output = {
        GYRO_TURN_MOTION_BRAKE, false, false, false};

    if (state->phase == GYRO_TURN_PHASE_FAULT) {
        output.fault = true;
        return output;
    }
    if (state->phase == GYRO_TURN_PHASE_COMPLETE) {
        output.motion = GYRO_TURN_MOTION_STOP;
        output.complete = true;
        return output;
    }

    if (state->sensor_failure_count < UINT8_MAX) {
        state->sensor_failure_count++;
    }
    if (!gyro_turn_advance_elapsed(state, delta_ms) ||
        (state->sensor_failure_count >= config->max_sensor_failures)) {
        state->phase = GYRO_TURN_PHASE_FAULT;
        output.motion = GYRO_TURN_MOTION_STOP;
        output.fault = true;
    }
    return output;
}

static inline gyro_turn_output_t gyro_turn_step(
    const gyro_turn_config_t *config,
    gyro_turn_state_t *state,
    int32_t yaw_mdeg,
    int32_t rate_mdps,
    uint16_t delta_ms)
{
    gyro_turn_output_t output = {GYRO_TURN_MOTION_STOP, false, false, false};
    int32_t error_mdeg;
    uint32_t abs_error_mdeg;

    if (state->phase == GYRO_TURN_PHASE_FAULT) {
        output.fault = true;
        return output;
    }
    if (state->phase == GYRO_TURN_PHASE_COMPLETE) {
        output.complete = true;
        return output;
    }
    if (!gyro_turn_advance_elapsed(state, delta_ms)) {
        output.fault = true;
        return output;
    }

    if (state->phase == GYRO_TURN_PHASE_CENTER) {
        state->phase_elapsed_ms =
            (uint16_t)(state->phase_elapsed_ms + delta_ms);
        if (state->phase_elapsed_ms < state->center_forward_ms) {
            output.motion = GYRO_TURN_MOTION_FORWARD;
            return output;
        }
        state->phase = GYRO_TURN_PHASE_SETTLE;
        state->phase_elapsed_ms = 0U;
        return output;
    }

    if (state->phase == GYRO_TURN_PHASE_SETTLE) {
        state->phase_elapsed_ms =
            (uint16_t)(state->phase_elapsed_ms + delta_ms);
        if (state->phase_elapsed_ms >= config->settle_ms) {
            state->phase = GYRO_TURN_PHASE_ROTATE;
            state->phase_elapsed_ms = 0U;
            output.reset_yaw = true;
        }
        return output;
    }

    error_mdeg = state->target_mdeg - yaw_mdeg;
    abs_error_mdeg = gyro_turn_abs_i32(error_mdeg);
    if (abs_error_mdeg <= (uint32_t)config->tolerance_mdeg) {
        output.motion = GYRO_TURN_MOTION_BRAKE;
        if (gyro_turn_abs_i32(rate_mdps) <=
            (uint32_t)config->stopped_rate_limit_mdps) {
            state->stable_frames++;
        } else {
            state->stable_frames = 0U;
        }
        if (state->stable_frames >= config->stable_frames_required) {
            state->phase = GYRO_TURN_PHASE_COMPLETE;
            output.complete = true;
        }
        return output;
    }

    state->stable_frames = 0U;
    if (error_mdeg > 0) {
        output.motion = (abs_error_mdeg <= (uint32_t)config->slow_zone_mdeg)
            ? GYRO_TURN_MOTION_SPIN_LEFT_SLOW
            : GYRO_TURN_MOTION_SPIN_LEFT_FAST;
    } else {
        output.motion = (abs_error_mdeg <= (uint32_t)config->slow_zone_mdeg)
            ? GYRO_TURN_MOTION_SPIN_RIGHT_SLOW
            : GYRO_TURN_MOTION_SPIN_RIGHT_FAST;
    }
    return output;
}

#endif
