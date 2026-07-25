#ifndef ROUTE_PROGRESS_GUARD_H
#define ROUTE_PROGRESS_GUARD_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    ROUTE_PROGRESS_FAULT_NONE = 0,
    ROUTE_PROGRESS_FAULT_DIRECTION,
    ROUTE_PROGRESS_FAULT_NO_MOTION,
    ROUTE_PROGRESS_FAULT_WHEEL_IMBALANCE,
    ROUTE_PROGRESS_FAULT_ENDPOINT_OVERRUN
} route_progress_fault_t;

typedef struct {
    uint32_t direction_tolerance_edges;
    uint32_t no_motion_timeout_frames;
    uint32_t imbalance_check_edges;
    uint8_t maximum_wheel_ratio;
    uint32_t endpoint_confirmation_grace_edges;
} route_progress_config_t;

typedef struct {
    int32_t left_start_count;
    int32_t right_start_count;
    uint32_t endpoint_min_edges;
    uint32_t endpoint_max_edges;
    uint32_t sample_frames;
    uint32_t left_forward_edges;
    uint32_t right_forward_edges;
    uint32_t average_forward_edges;
    uint32_t previous_left_forward_edges;
    uint32_t previous_right_forward_edges;
    uint32_t left_no_motion_frames;
    uint32_t right_no_motion_frames;
    uint32_t no_motion_frames;
    route_progress_fault_t fault;
} route_progress_state_t;

static inline uint32_t route_progress_positive_edges(int64_t signed_edges)
{
    if (signed_edges <= 0) {
        return 0U;
    }
    if ((uint64_t)signed_edges > UINT32_MAX) {
        return UINT32_MAX;
    }
    return (uint32_t)signed_edges;
}

static inline uint32_t route_progress_average(
    uint32_t left_edges,
    uint32_t right_edges)
{
    if (left_edges <= right_edges) {
        return left_edges + ((right_edges - left_edges) / 2U);
    }
    return right_edges + ((left_edges - right_edges) / 2U);
}

static inline void route_progress_begin(
    route_progress_state_t *state,
    int32_t left_start_count,
    int32_t right_start_count,
    uint32_t endpoint_min_edges,
    uint32_t endpoint_max_edges)
{
    *state = (route_progress_state_t){
        .left_start_count = left_start_count,
        .right_start_count = right_start_count,
        .endpoint_min_edges = endpoint_min_edges,
        .endpoint_max_edges = endpoint_max_edges,
        .fault = ROUTE_PROGRESS_FAULT_NONE,
    };
}

static inline route_progress_fault_t route_progress_step(
    const route_progress_config_t *config,
    route_progress_state_t *state,
    int32_t left_count,
    int32_t right_count)
{
    const int64_t left_forward_signed =
        (int64_t)state->left_start_count - (int64_t)left_count;
    const int64_t right_forward_signed =
        (int64_t)right_count - (int64_t)state->right_start_count;
    uint32_t smaller_edges;
    uint32_t larger_edges;
    uint32_t hard_stop_edges;

    if (state->fault != ROUTE_PROGRESS_FAULT_NONE) {
        return state->fault;
    }

    if (state->sample_frames < UINT32_MAX) {
        state->sample_frames++;
    }
    state->left_forward_edges =
        route_progress_positive_edges(left_forward_signed);
    state->right_forward_edges =
        route_progress_positive_edges(right_forward_signed);
    state->average_forward_edges = route_progress_average(
        state->left_forward_edges,
        state->right_forward_edges);

    if ((left_forward_signed <
         -(int64_t)config->direction_tolerance_edges) ||
        (right_forward_signed <
         -(int64_t)config->direction_tolerance_edges)) {
        state->fault = ROUTE_PROGRESS_FAULT_DIRECTION;
        return state->fault;
    }

    if (state->left_forward_edges >
        state->previous_left_forward_edges) {
        state->left_no_motion_frames = 0U;
    } else if (state->left_no_motion_frames < UINT32_MAX) {
        state->left_no_motion_frames++;
    }
    if (state->right_forward_edges >
        state->previous_right_forward_edges) {
        state->right_no_motion_frames = 0U;
    } else if (state->right_no_motion_frames < UINT32_MAX) {
        state->right_no_motion_frames++;
    }
    state->previous_left_forward_edges = state->left_forward_edges;
    state->previous_right_forward_edges = state->right_forward_edges;
    state->no_motion_frames =
        (state->left_no_motion_frames < state->right_no_motion_frames)
        ? state->left_no_motion_frames
        : state->right_no_motion_frames;
    if ((config->no_motion_timeout_frames > 0U) &&
        (state->left_no_motion_frames >=
         config->no_motion_timeout_frames) &&
        (state->right_no_motion_frames >=
         config->no_motion_timeout_frames)) {
        state->fault = ROUTE_PROGRESS_FAULT_NO_MOTION;
        return state->fault;
    }
    if ((config->no_motion_timeout_frames > 0U) &&
        ((state->left_no_motion_frames >=
          config->no_motion_timeout_frames) ||
         (state->right_no_motion_frames >=
          config->no_motion_timeout_frames))) {
        state->fault = ROUTE_PROGRESS_FAULT_WHEEL_IMBALANCE;
        return state->fault;
    }

    smaller_edges = state->left_forward_edges;
    larger_edges = state->right_forward_edges;
    if (smaller_edges > larger_edges) {
        const uint32_t temporary = smaller_edges;

        smaller_edges = larger_edges;
        larger_edges = temporary;
    }
    if ((config->maximum_wheel_ratio > 0U) &&
        (larger_edges >= config->imbalance_check_edges) &&
        (((uint64_t)smaller_edges * config->maximum_wheel_ratio) <=
         larger_edges)) {
        state->fault = ROUTE_PROGRESS_FAULT_WHEEL_IMBALANCE;
        return state->fault;
    }

    hard_stop_edges = state->endpoint_max_edges;
    if (hard_stop_edges >
        (UINT32_MAX - config->endpoint_confirmation_grace_edges)) {
        hard_stop_edges = UINT32_MAX;
    } else {
        hard_stop_edges += config->endpoint_confirmation_grace_edges;
    }
    if ((state->endpoint_max_edges > 0U) &&
        (state->average_forward_edges > hard_stop_edges)) {
        state->fault = ROUTE_PROGRESS_FAULT_ENDPOINT_OVERRUN;
    }
    return state->fault;
}

static inline bool route_progress_endpoint_ready(
    const route_progress_state_t *state)
{
    return (state->fault == ROUTE_PROGRESS_FAULT_NONE) &&
           (state->endpoint_min_edges > 0U) &&
           (state->endpoint_max_edges >= state->endpoint_min_edges) &&
           (state->average_forward_edges >= state->endpoint_min_edges) &&
           (state->average_forward_edges <= state->endpoint_max_edges);
}

#endif
