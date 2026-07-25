#ifndef ROUTE_EVENT_DETECTOR_H
#define ROUTE_EVENT_DETECTOR_H

#include <stdbool.h>
#include <stdint.h>

#include "medicine_route.h"

typedef enum {
    ROUTE_EVENT_NONE = 0,
    ROUTE_EVENT_JUNCTION,
    ROUTE_EVENT_ENDPOINT
} route_event_t;

typedef struct {
    uint8_t wide_active_min;
    uint8_t wide_confirm_frames;
    uint8_t endpoint_confirm_frames;
    uint8_t endpoint_centered_frames;
    uint8_t rearm_normal_frames;
    uint16_t min_travel_frames;
} route_event_config_t;

typedef struct {
    uint16_t travel_frames;
    uint8_t normal_frames;
    uint8_t wide_frames;
    uint8_t lost_frames;
    uint8_t centered_frames;
    bool armed;
    bool endpoint_loss_active;
    bool endpoint_entry_centered;
    bool endpoint_entry_distance_ready;
} route_event_state_t;

#define ROUTE_EVENT_CONFIG_INITIALIZER \
    {                                  \
        .wide_active_min = 7U,         \
        .wide_confirm_frames = 2U,     \
        .endpoint_confirm_frames = 8U, \
        .endpoint_centered_frames = 5U, \
        .rearm_normal_frames = 5U,     \
        .min_travel_frames = 30U,      \
    }

static inline void route_event_init(route_event_state_t *state)
{
    *state = (route_event_state_t){0};
}

static inline uint8_t route_event_increment_u8(uint8_t value)
{
    return (value < UINT8_MAX) ? (uint8_t)(value + 1U) : UINT8_MAX;
}

static inline uint16_t route_event_increment_u16(uint16_t value)
{
    return (value < UINT16_MAX) ? (uint16_t)(value + 1U) : UINT16_MAX;
}

static inline route_event_t route_event_step(
    const route_event_config_t *config,
    route_event_state_t *state,
    route_destination_t destination,
    uint8_t active_count,
    bool line_centered,
    bool endpoint_distance_ready)
{
    const bool normal_line =
        (active_count > 0U) && (active_count < config->wide_active_min);

    if (normal_line) {
        state->travel_frames = route_event_increment_u16(state->travel_frames);
        state->normal_frames = route_event_increment_u8(state->normal_frames);
    } else {
        state->normal_frames = 0U;
    }

    if ((destination == ROUTE_DESTINATION_ENDPOINT) &&
        normal_line && line_centered) {
        state->centered_frames =
            route_event_increment_u8(state->centered_frames);
    } else if (active_count != 0U) {
        state->centered_frames = 0U;
    }

    if (!state->armed) {
        if ((state->travel_frames >= config->min_travel_frames) &&
            (state->normal_frames >= config->rearm_normal_frames)) {
            state->armed = true;
        }
        state->wide_frames = 0U;
        state->lost_frames = 0U;
        state->endpoint_loss_active = false;
        state->endpoint_entry_centered = false;
        state->endpoint_entry_distance_ready = false;
        return ROUTE_EVENT_NONE;
    }

    if (destination == ROUTE_DESTINATION_JUNCTION) {
        state->lost_frames = 0U;
        state->centered_frames = 0U;
        state->endpoint_loss_active = false;
        state->endpoint_entry_centered = false;
        state->endpoint_entry_distance_ready = false;
        if (active_count >= config->wide_active_min) {
            state->wide_frames = route_event_increment_u8(state->wide_frames);
        } else {
            state->wide_frames = 0U;
        }
        if (state->wide_frames >= config->wide_confirm_frames) {
            state->armed = false;
            return ROUTE_EVENT_JUNCTION;
        }
    } else {
        state->wide_frames = 0U;
        if (active_count == 0U) {
            if (!state->endpoint_loss_active) {
                state->endpoint_loss_active = true;
                state->endpoint_entry_centered =
                    state->centered_frames >=
                    config->endpoint_centered_frames;
                state->endpoint_entry_distance_ready =
                    endpoint_distance_ready;
                state->centered_frames = 0U;
            }
            if (state->endpoint_entry_distance_ready &&
                state->endpoint_entry_centered) {
                state->lost_frames =
                    route_event_increment_u8(state->lost_frames);
            } else {
                state->lost_frames = 0U;
            }
        } else {
            state->lost_frames = 0U;
            state->endpoint_loss_active = false;
            state->endpoint_entry_centered = false;
            state->endpoint_entry_distance_ready = false;
        }
        if (state->lost_frames >= config->endpoint_confirm_frames) {
            state->armed = false;
            return ROUTE_EVENT_ENDPOINT;
        }
    }

    return ROUTE_EVENT_NONE;
}

#endif
