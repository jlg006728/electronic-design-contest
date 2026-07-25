#ifndef MEDICINE_MISSION_H
#define MEDICINE_MISSION_H

#include <stdbool.h>
#include <stdint.h>

#include "medicine_route.h"

typedef struct {
    uint16_t delta_ms;
    bool destination_reached;
    bool turn_complete;
    bool hardware_fault;
} medicine_mission_input_t;

typedef struct {
    uint8_t action_index;
    uint8_t visited_ward_mask;
    uint8_t ward_stop_count;
    uint32_t action_elapsed_ms;
    bool completed;
    bool fault;
} medicine_mission_state_t;

static inline void medicine_mission_init(medicine_mission_state_t *state)
{
    *state = (medicine_mission_state_t){0};
}

static inline const medicine_route_action_t *medicine_mission_current_action(
    const medicine_mission_state_t *state)
{
    return medicine_route_action_at(state->action_index);
}

static inline bool medicine_mission_tick_overrun(
    uint32_t work_elapsed_ms,
    uint16_t tick_period_ms)
{
    return work_elapsed_ms >= tick_period_ms;
}

static inline void medicine_mission_enter_action(
    medicine_mission_state_t *state,
    uint8_t action_index)
{
    const medicine_route_action_t *action;

    state->action_index = action_index;
    state->action_elapsed_ms = 0U;
    action = medicine_mission_current_action(state);
    if (action == NULL) {
        state->fault = true;
        return;
    }
    if (action->kind == MEDICINE_ACTION_WARD_STOP_2S) {
        uint8_t ward_bit;

        if ((action->ward == 0U) || (action->ward > 8U)) {
            state->fault = true;
            return;
        }
        ward_bit = (uint8_t)(1U << (action->ward - 1U));
        if ((state->visited_ward_mask & ward_bit) != 0U) {
            state->fault = true;
            return;
        }
        state->visited_ward_mask |= ward_bit;
        state->ward_stop_count++;
    }
}

static inline void medicine_mission_advance(medicine_mission_state_t *state)
{
    if (state->action_index >= (MEDICINE_ROUTE_ACTION_COUNT - 1U)) {
        state->fault = true;
        return;
    }
    medicine_mission_enter_action(state, (uint8_t)(state->action_index + 1U));
}

static inline void medicine_mission_step(
    medicine_mission_state_t *state,
    const medicine_mission_input_t *input)
{
    const medicine_route_action_t *action;

    if (state->completed || state->fault) {
        return;
    }
    if (input->hardware_fault) {
        state->fault = true;
        return;
    }

    action = medicine_mission_current_action(state);
    if (action == NULL) {
        state->fault = true;
        return;
    }

    if (medicine_route_is_follow_action(action->kind)) {
        if (input->destination_reached) {
            medicine_mission_advance(state);
        }
    } else if (medicine_route_is_turn_action(action->kind)) {
        if (input->turn_complete) {
            medicine_mission_advance(state);
        }
    } else if (action->kind == MEDICINE_ACTION_WARD_STOP_2S) {
        if (UINT32_MAX - state->action_elapsed_ms < input->delta_ms) {
            state->action_elapsed_ms = UINT32_MAX;
        } else {
            state->action_elapsed_ms += input->delta_ms;
        }
        if (state->action_elapsed_ms >= MEDICINE_WARD_STOP_MS) {
            medicine_mission_advance(state);
        }
    } else if (action->kind == MEDICINE_ACTION_COMPLETE) {
        state->completed = (state->visited_ward_mask == 0xFFU) &&
                           (state->ward_stop_count == 8U);
        state->fault = !state->completed;
    } else {
        state->fault = true;
    }
}

#endif
