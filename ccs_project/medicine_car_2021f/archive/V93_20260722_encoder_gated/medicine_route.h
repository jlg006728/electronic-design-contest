#ifndef MEDICINE_ROUTE_H
#define MEDICINE_ROUTE_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef enum {
    MEDICINE_ACTION_FOLLOW_TO_JUNCTION = 0,
    MEDICINE_ACTION_FOLLOW_TO_ENDPOINT,
    MEDICINE_ACTION_TURN_LEFT_90,
    MEDICINE_ACTION_TURN_RIGHT_90,
    MEDICINE_ACTION_TURN_AROUND_180,
    MEDICINE_ACTION_WARD_STOP_2S,
    MEDICINE_ACTION_COMPLETE
} medicine_action_kind_t;

typedef enum {
    ROUTE_DESTINATION_JUNCTION = 0,
    ROUTE_DESTINATION_ENDPOINT
} route_destination_t;

typedef struct {
    medicine_action_kind_t kind;
    uint8_t ward;
} medicine_route_action_t;

#define MEDICINE_ROUTE_ACTION_COUNT 53U
#define MEDICINE_WARD_STOP_MS       2000U
#define MEDICINE_ENDPOINT_MINIMUM_CM 20U
#define MEDICINE_WARD_ENDPOINT_MAXIMUM_CM 70U
#define MEDICINE_HOME_ENDPOINT_MAXIMUM_CM 100U

static const medicine_route_action_t MEDICINE_ROUTE[MEDICINE_ROUTE_ACTION_COUNT] = {
    {MEDICINE_ACTION_FOLLOW_TO_JUNCTION, 0U},
    {MEDICINE_ACTION_TURN_LEFT_90, 0U},
    {MEDICINE_ACTION_FOLLOW_TO_ENDPOINT, 1U},
    {MEDICINE_ACTION_WARD_STOP_2S, 1U},
    {MEDICINE_ACTION_TURN_AROUND_180, 0U},
    {MEDICINE_ACTION_FOLLOW_TO_JUNCTION, 0U},
    {MEDICINE_ACTION_FOLLOW_TO_ENDPOINT, 2U},
    {MEDICINE_ACTION_WARD_STOP_2S, 2U},
    {MEDICINE_ACTION_TURN_AROUND_180, 0U},
    {MEDICINE_ACTION_FOLLOW_TO_JUNCTION, 0U},
    {MEDICINE_ACTION_TURN_RIGHT_90, 0U},
    {MEDICINE_ACTION_FOLLOW_TO_JUNCTION, 0U},
    {MEDICINE_ACTION_TURN_LEFT_90, 0U},
    {MEDICINE_ACTION_FOLLOW_TO_ENDPOINT, 3U},
    {MEDICINE_ACTION_WARD_STOP_2S, 3U},
    {MEDICINE_ACTION_TURN_AROUND_180, 0U},
    {MEDICINE_ACTION_FOLLOW_TO_JUNCTION, 0U},
    {MEDICINE_ACTION_FOLLOW_TO_ENDPOINT, 4U},
    {MEDICINE_ACTION_WARD_STOP_2S, 4U},
    {MEDICINE_ACTION_TURN_AROUND_180, 0U},
    {MEDICINE_ACTION_FOLLOW_TO_JUNCTION, 0U},
    {MEDICINE_ACTION_TURN_RIGHT_90, 0U},
    {MEDICINE_ACTION_FOLLOW_TO_JUNCTION, 0U},
    {MEDICINE_ACTION_TURN_LEFT_90, 0U},
    {MEDICINE_ACTION_FOLLOW_TO_JUNCTION, 0U},
    {MEDICINE_ACTION_TURN_LEFT_90, 0U},
    {MEDICINE_ACTION_FOLLOW_TO_ENDPOINT, 7U},
    {MEDICINE_ACTION_WARD_STOP_2S, 7U},
    {MEDICINE_ACTION_TURN_AROUND_180, 0U},
    {MEDICINE_ACTION_FOLLOW_TO_JUNCTION, 0U},
    {MEDICINE_ACTION_FOLLOW_TO_ENDPOINT, 5U},
    {MEDICINE_ACTION_WARD_STOP_2S, 5U},
    {MEDICINE_ACTION_TURN_AROUND_180, 0U},
    {MEDICINE_ACTION_FOLLOW_TO_JUNCTION, 0U},
    {MEDICINE_ACTION_TURN_LEFT_90, 0U},
    {MEDICINE_ACTION_FOLLOW_TO_JUNCTION, 0U},
    {MEDICINE_ACTION_FOLLOW_TO_JUNCTION, 0U},
    {MEDICINE_ACTION_TURN_RIGHT_90, 0U},
    {MEDICINE_ACTION_FOLLOW_TO_ENDPOINT, 6U},
    {MEDICINE_ACTION_WARD_STOP_2S, 6U},
    {MEDICINE_ACTION_TURN_AROUND_180, 0U},
    {MEDICINE_ACTION_FOLLOW_TO_JUNCTION, 0U},
    {MEDICINE_ACTION_FOLLOW_TO_ENDPOINT, 8U},
    {MEDICINE_ACTION_WARD_STOP_2S, 8U},
    {MEDICINE_ACTION_TURN_AROUND_180, 0U},
    {MEDICINE_ACTION_FOLLOW_TO_JUNCTION, 0U},
    {MEDICINE_ACTION_TURN_RIGHT_90, 0U},
    {MEDICINE_ACTION_FOLLOW_TO_JUNCTION, 0U},
    {MEDICINE_ACTION_TURN_LEFT_90, 0U},
    {MEDICINE_ACTION_FOLLOW_TO_JUNCTION, 0U},
    {MEDICINE_ACTION_FOLLOW_TO_JUNCTION, 0U},
    {MEDICINE_ACTION_FOLLOW_TO_ENDPOINT, 0U},
    {MEDICINE_ACTION_COMPLETE, 0U},
};

static inline const medicine_route_action_t *medicine_route_action_at(
    uint8_t index)
{
    if (index >= MEDICINE_ROUTE_ACTION_COUNT) {
        return NULL;
    }
    return &MEDICINE_ROUTE[index];
}

static inline bool medicine_route_is_follow_action(
    medicine_action_kind_t kind)
{
    return (kind == MEDICINE_ACTION_FOLLOW_TO_JUNCTION) ||
           (kind == MEDICINE_ACTION_FOLLOW_TO_ENDPOINT);
}

static inline bool medicine_route_is_turn_action(
    medicine_action_kind_t kind)
{
    return (kind == MEDICINE_ACTION_TURN_LEFT_90) ||
           (kind == MEDICINE_ACTION_TURN_RIGHT_90) ||
           (kind == MEDICINE_ACTION_TURN_AROUND_180);
}

static inline route_destination_t medicine_route_destination(
    medicine_action_kind_t kind)
{
    return (kind == MEDICINE_ACTION_FOLLOW_TO_JUNCTION)
        ? ROUTE_DESTINATION_JUNCTION
        : ROUTE_DESTINATION_ENDPOINT;
}

static inline int32_t medicine_route_turn_target_mdeg(
    medicine_action_kind_t kind)
{
    if (kind == MEDICINE_ACTION_TURN_LEFT_90) {
        return 90000;
    }
    if (kind == MEDICINE_ACTION_TURN_RIGHT_90) {
        return -90000;
    }
    if (kind == MEDICINE_ACTION_TURN_AROUND_180) {
        return 180000;
    }
    return 0;
}

static inline uint16_t medicine_route_endpoint_minimum_cm(
    const medicine_route_action_t *action)
{
    if ((action == NULL) ||
        (action->kind != MEDICINE_ACTION_FOLLOW_TO_ENDPOINT)) {
        return 0U;
    }
    return MEDICINE_ENDPOINT_MINIMUM_CM;
}

static inline uint16_t medicine_route_endpoint_maximum_cm(
    const medicine_route_action_t *action)
{
    if ((action == NULL) ||
        (action->kind != MEDICINE_ACTION_FOLLOW_TO_ENDPOINT)) {
        return 0U;
    }
    return (action->ward == 0U)
        ? MEDICINE_HOME_ENDPOINT_MAXIMUM_CM
        : MEDICINE_WARD_ENDPOINT_MAXIMUM_CM;
}

#endif
