#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "gyro_turn_control.h"
#include "medicine_mission.h"
#include "medicine_route.h"
#include "route_event_detector.h"
#include "route_progress_guard.h"

static const route_event_config_t EVENT_CONFIG =
    ROUTE_EVENT_CONFIG_INITIALIZER;

static const route_progress_config_t PROGRESS_CONFIG = {
    .direction_tolerance_edges = 100U,
    .no_motion_timeout_frames = 50U,
    .imbalance_check_edges = 1000U,
    .maximum_wheel_ratio = 4U,
    .endpoint_confirmation_grace_edges = 200U,
};

static const gyro_turn_config_t TURN_CONFIG = {
    .settle_ms = 20U,
    .slow_zone_mdeg = 20000,
    .tolerance_mdeg = 2500,
    .stopped_rate_limit_mdps = 4000,
    .stable_frames_required = 3U,
    .max_sensor_failures = 3U,
};

static void test_route_table_matches_full_course(void)
{
    static const uint8_t expected_wards[8] = {1U, 2U, 3U, 4U, 7U, 5U, 6U, 8U};
    static const medicine_action_kind_t expected_actions[53] = {
        MEDICINE_ACTION_FOLLOW_TO_JUNCTION,
        MEDICINE_ACTION_TURN_LEFT_90,
        MEDICINE_ACTION_FOLLOW_TO_ENDPOINT,
        MEDICINE_ACTION_WARD_STOP_2S,
        MEDICINE_ACTION_TURN_AROUND_180,
        MEDICINE_ACTION_FOLLOW_TO_JUNCTION,
        MEDICINE_ACTION_FOLLOW_TO_ENDPOINT,
        MEDICINE_ACTION_WARD_STOP_2S,
        MEDICINE_ACTION_TURN_AROUND_180,
        MEDICINE_ACTION_FOLLOW_TO_JUNCTION,
        MEDICINE_ACTION_TURN_RIGHT_90,
        MEDICINE_ACTION_FOLLOW_TO_JUNCTION,
        MEDICINE_ACTION_TURN_LEFT_90,
        MEDICINE_ACTION_FOLLOW_TO_ENDPOINT,
        MEDICINE_ACTION_WARD_STOP_2S,
        MEDICINE_ACTION_TURN_AROUND_180,
        MEDICINE_ACTION_FOLLOW_TO_JUNCTION,
        MEDICINE_ACTION_FOLLOW_TO_ENDPOINT,
        MEDICINE_ACTION_WARD_STOP_2S,
        MEDICINE_ACTION_TURN_AROUND_180,
        MEDICINE_ACTION_FOLLOW_TO_JUNCTION,
        MEDICINE_ACTION_TURN_RIGHT_90,
        MEDICINE_ACTION_FOLLOW_TO_JUNCTION,
        MEDICINE_ACTION_TURN_LEFT_90,
        MEDICINE_ACTION_FOLLOW_TO_JUNCTION,
        MEDICINE_ACTION_TURN_LEFT_90,
        MEDICINE_ACTION_FOLLOW_TO_ENDPOINT,
        MEDICINE_ACTION_WARD_STOP_2S,
        MEDICINE_ACTION_TURN_AROUND_180,
        MEDICINE_ACTION_FOLLOW_TO_JUNCTION,
        MEDICINE_ACTION_FOLLOW_TO_ENDPOINT,
        MEDICINE_ACTION_WARD_STOP_2S,
        MEDICINE_ACTION_TURN_AROUND_180,
        MEDICINE_ACTION_FOLLOW_TO_JUNCTION,
        MEDICINE_ACTION_TURN_LEFT_90,
        MEDICINE_ACTION_FOLLOW_TO_JUNCTION,
        MEDICINE_ACTION_FOLLOW_TO_JUNCTION,
        MEDICINE_ACTION_TURN_RIGHT_90,
        MEDICINE_ACTION_FOLLOW_TO_ENDPOINT,
        MEDICINE_ACTION_WARD_STOP_2S,
        MEDICINE_ACTION_TURN_AROUND_180,
        MEDICINE_ACTION_FOLLOW_TO_JUNCTION,
        MEDICINE_ACTION_FOLLOW_TO_ENDPOINT,
        MEDICINE_ACTION_WARD_STOP_2S,
        MEDICINE_ACTION_TURN_AROUND_180,
        MEDICINE_ACTION_FOLLOW_TO_JUNCTION,
        MEDICINE_ACTION_TURN_RIGHT_90,
        MEDICINE_ACTION_FOLLOW_TO_JUNCTION,
        MEDICINE_ACTION_TURN_LEFT_90,
        MEDICINE_ACTION_FOLLOW_TO_JUNCTION,
        MEDICINE_ACTION_FOLLOW_TO_JUNCTION,
        MEDICINE_ACTION_FOLLOW_TO_ENDPOINT,
        MEDICINE_ACTION_COMPLETE,
    };
    uint8_t ward_index = 0U;
    uint8_t left_turns = 0U;
    uint8_t right_turns = 0U;
    uint8_t turn_arounds = 0U;
    uint8_t action_index;

    assert(MEDICINE_ROUTE_ACTION_COUNT == 53U);
    for (action_index = 0U;
         action_index < MEDICINE_ROUTE_ACTION_COUNT;
         action_index++) {
        const medicine_route_action_t *action =
            medicine_route_action_at(action_index);

        assert(action != NULL);
        assert(action->kind == expected_actions[action_index]);
        if (action->kind == MEDICINE_ACTION_WARD_STOP_2S) {
            assert(ward_index < 8U);
            assert(action->ward == expected_wards[ward_index]);
            ward_index++;
        } else if (action->kind == MEDICINE_ACTION_TURN_LEFT_90) {
            left_turns++;
        } else if (action->kind == MEDICINE_ACTION_TURN_RIGHT_90) {
            right_turns++;
        } else if (action->kind == MEDICINE_ACTION_TURN_AROUND_180) {
            turn_arounds++;
        }
    }

    assert(ward_index == 8U);
    assert(left_turns == 6U);
    assert(right_turns == 4U);
    assert(turn_arounds == 8U);
    assert(medicine_route_action_at(MEDICINE_ROUTE_ACTION_COUNT) == NULL);
    assert(medicine_route_action_at(MEDICINE_ROUTE_ACTION_COUNT - 1U)->kind ==
           MEDICINE_ACTION_COMPLETE);
    assert(medicine_route_turn_target_mdeg(MEDICINE_ACTION_TURN_LEFT_90) ==
           90000);
    assert(medicine_route_turn_target_mdeg(MEDICINE_ACTION_TURN_RIGHT_90) ==
           -90000);
    assert(medicine_route_turn_target_mdeg(MEDICINE_ACTION_TURN_AROUND_180) ==
           180000);
}

static void test_every_endpoint_has_a_distance_window(void)
{
    typedef struct {
        uint8_t action_index;
        uint8_t ward;
        uint16_t minimum_cm;
        uint16_t maximum_cm;
    } expected_endpoint_t;
    static const expected_endpoint_t expected_endpoints[9] = {
        {2U, 1U, 20U, 70U},
        {6U, 2U, 20U, 70U},
        {13U, 3U, 20U, 70U},
        {17U, 4U, 20U, 70U},
        {26U, 7U, 20U, 70U},
        {30U, 5U, 20U, 70U},
        {38U, 6U, 20U, 70U},
        {42U, 8U, 20U, 70U},
        {51U, 0U, 20U, 100U},
    };
    uint8_t action_index;
    uint8_t endpoint_index = 0U;

    for (action_index = 0U;
         action_index < MEDICINE_ROUTE_ACTION_COUNT;
         action_index++) {
        const medicine_route_action_t *action =
            medicine_route_action_at(action_index);
        const uint16_t minimum_cm =
            medicine_route_endpoint_minimum_cm(action);
        const uint16_t maximum_cm =
            medicine_route_endpoint_maximum_cm(action);

        assert(action != NULL);
        if (action->kind == MEDICINE_ACTION_FOLLOW_TO_ENDPOINT) {
            const expected_endpoint_t *expected;

            assert(endpoint_index < 9U);
            expected = &expected_endpoints[endpoint_index];
            assert(action_index == expected->action_index);
            assert(action->ward == expected->ward);
            assert(action->endpoint_minimum_cm == minimum_cm);
            assert(action->endpoint_maximum_cm == maximum_cm);
            assert(minimum_cm == expected->minimum_cm);
            assert(maximum_cm == expected->maximum_cm);
            endpoint_index++;
        } else {
            assert(action->endpoint_minimum_cm == 0U);
            assert(action->endpoint_maximum_cm == 0U);
            assert(minimum_cm == 0U);
            assert(maximum_cm == 0U);
        }
    }
    assert(endpoint_index == 9U);
}

static void test_mission_faults_are_fail_closed(void)
{
    medicine_mission_state_t mission;
    medicine_mission_input_t input = {.hardware_fault = true};
    uint8_t prior_action;

    medicine_mission_init(&mission);
    medicine_mission_step(&mission, &input);
    assert(mission.fault);
    assert(!mission.completed);
    prior_action = mission.action_index;
    medicine_mission_step(&mission, &input);
    assert(mission.action_index == prior_action);

    medicine_mission_init(&mission);
    mission.action_index = MEDICINE_ROUTE_ACTION_COUNT;
    input = (medicine_mission_input_t){0};
    medicine_mission_step(&mission, &input);
    assert(mission.fault);

    medicine_mission_init(&mission);
    medicine_mission_enter_action(&mission, MEDICINE_ROUTE_ACTION_COUNT);
    assert(mission.fault);

    medicine_mission_init(&mission);
    mission.visited_ward_mask = 0x01U;
    medicine_mission_enter_action(&mission, 3U);
    assert(mission.fault);

    medicine_mission_init(&mission);
    mission.action_index = MEDICINE_ROUTE_ACTION_COUNT - 1U;
    medicine_mission_advance(&mission);
    assert(mission.fault);

    medicine_mission_init(&mission);
    mission.action_index = 3U;
    mission.action_elapsed_ms = UINT32_MAX - 1U;
    input = (medicine_mission_input_t){.delta_ms = 10U};
    medicine_mission_step(&mission, &input);
    assert(mission.action_index == 4U);

    medicine_mission_init(&mission);
    mission.action_index = MEDICINE_ROUTE_ACTION_COUNT - 1U;
    medicine_mission_step(&mission, &input);
    assert(mission.fault);
    assert(!mission.completed);
}

static void test_control_tick_overrun_boundary(void)
{
    assert(!medicine_mission_tick_overrun(0U, 10U));
    assert(!medicine_mission_tick_overrun(9U, 10U));
    assert(medicine_mission_tick_overrun(10U, 10U));
    assert(medicine_mission_tick_overrun(11U, 10U));
}

static void test_mission_visits_every_ward_and_returns_home(void)
{
    medicine_mission_state_t mission;
    uint32_t guard = 0U;

    medicine_mission_init(&mission);
    while (!mission.completed && !mission.fault && (guard < 500U)) {
        const medicine_route_action_t *action =
            medicine_mission_current_action(&mission);
        medicine_mission_input_t input = {0};

        assert(action != NULL);
        if (medicine_route_is_follow_action(action->kind)) {
            input.destination_reached = true;
        } else if (medicine_route_is_turn_action(action->kind)) {
            input.turn_complete = true;
        } else if (action->kind == MEDICINE_ACTION_WARD_STOP_2S) {
            input.delta_ms = 500U;
        }
        medicine_mission_step(&mission, &input);
        guard++;
    }

    assert(guard < 500U);
    assert(!mission.fault);
    assert(mission.completed);
    assert(mission.visited_ward_mask == 0xFFU);
    assert(mission.ward_stop_count == 8U);
    assert(mission.action_index == MEDICINE_ROUTE_ACTION_COUNT - 1U);
}

static void test_ward_stop_is_exactly_two_seconds(void)
{
    medicine_mission_state_t mission;
    medicine_mission_input_t input = {0};
    uint8_t stop_action_index;

    medicine_mission_init(&mission);
    for (;;) {
        const medicine_route_action_t *action =
            medicine_mission_current_action(&mission);

        assert(action != NULL);
        if (action->kind == MEDICINE_ACTION_WARD_STOP_2S) {
            break;
        }
        input = (medicine_mission_input_t){0};
        input.destination_reached = medicine_route_is_follow_action(action->kind);
        input.turn_complete = medicine_route_is_turn_action(action->kind);
        medicine_mission_step(&mission, &input);
    }

    stop_action_index = mission.action_index;
    input = (medicine_mission_input_t){.delta_ms = 1990U};
    medicine_mission_step(&mission, &input);
    assert(mission.action_index == stop_action_index);
    assert(mission.action_elapsed_ms == 1990U);
    assert(mission.ward_stop_count == 1U);

    input.delta_ms = 10U;
    medicine_mission_step(&mission, &input);
    assert(mission.action_index == (uint8_t)(stop_action_index + 1U));
    assert(mission.action_elapsed_ms == 0U);
}

static route_event_t step_event(
    route_event_state_t *state,
    route_destination_t destination,
    uint8_t active_count)
{
    const bool line_centered =
        (active_count > 0U) &&
        (active_count < EVENT_CONFIG.wide_active_min);

    return route_event_step(
        &EVENT_CONFIG,
        state,
        destination,
        active_count,
        line_centered,
        true);
}

static void feed_normal_frames(
    route_event_state_t *state,
    route_destination_t destination,
    uint32_t count)
{
    uint32_t index;

    for (index = 0U; index < count; index++) {
        assert(step_event(
                   state, destination, 2U) ==
               ROUTE_EVENT_NONE);
    }
}

static void test_junction_requires_rearm_and_does_not_double_trigger(void)
{
    route_event_state_t detector;
    uint32_t index;

    route_event_init(&detector);
    for (index = 0U; index < 10U; index++) {
        assert(step_event(
                   &detector,
                   ROUTE_DESTINATION_JUNCTION,
                   8U) == ROUTE_EVENT_NONE);
    }
    feed_normal_frames(
        &detector,
        ROUTE_DESTINATION_JUNCTION,
        EVENT_CONFIG.min_travel_frames);
    assert(detector.armed);
    assert(step_event(
               &detector,
               ROUTE_DESTINATION_JUNCTION,
               7U) == ROUTE_EVENT_NONE);
    assert(step_event(
               &detector,
               ROUTE_DESTINATION_JUNCTION,
               8U) == ROUTE_EVENT_JUNCTION);
    assert(!detector.armed);
    assert(step_event(
               &detector,
               ROUTE_DESTINATION_JUNCTION,
               8U) == ROUTE_EVENT_NONE);
}

static void test_endpoint_requires_confirmed_loss_after_real_travel(void)
{
    route_event_state_t detector;
    uint32_t index;

    route_event_init(&detector);
    for (index = 0U; index < 12U; index++) {
        assert(step_event(
                   &detector,
                   ROUTE_DESTINATION_ENDPOINT,
                   0U) == ROUTE_EVENT_NONE);
    }
    feed_normal_frames(
        &detector,
        ROUTE_DESTINATION_ENDPOINT,
        EVENT_CONFIG.min_travel_frames);
    assert(detector.armed);
    for (index = 1U; index < EVENT_CONFIG.endpoint_confirm_frames; index++) {
        assert(step_event(
                   &detector,
                   ROUTE_DESTINATION_ENDPOINT,
                   0U) == ROUTE_EVENT_NONE);
    }
    assert(step_event(
               &detector,
               ROUTE_DESTINATION_ENDPOINT,
               0U) == ROUTE_EVENT_ENDPOINT);
}

static void test_midcourse_loss_never_advances_endpoint(void)
{
    route_event_state_t detector;
    uint32_t index;

    route_event_init(&detector);
    feed_normal_frames(
        &detector,
        ROUTE_DESTINATION_ENDPOINT,
        EVENT_CONFIG.min_travel_frames);
    assert(detector.armed);

    assert(route_event_step(
               &EVENT_CONFIG,
               &detector,
               ROUTE_DESTINATION_ENDPOINT,
               0U,
               false,
               false) == ROUTE_EVENT_NONE);
    for (index = 0U; index <
         (uint32_t)(EVENT_CONFIG.endpoint_confirm_frames + 5U); index++) {
        assert(route_event_step(
                   &EVENT_CONFIG,
                   &detector,
                   ROUTE_DESTINATION_ENDPOINT,
                   0U,
                   false,
                   true) == ROUTE_EVENT_NONE);
    }
    assert(detector.armed);
    assert(detector.lost_frames == 0U);
    assert(!detector.endpoint_entry_distance_ready);

    feed_normal_frames(
        &detector,
        ROUTE_DESTINATION_ENDPOINT,
        EVENT_CONFIG.endpoint_centered_frames);
    for (index = 1U; index < EVENT_CONFIG.endpoint_confirm_frames; index++) {
        assert(route_event_step(
                   &EVENT_CONFIG,
                   &detector,
                   ROUTE_DESTINATION_ENDPOINT,
                   0U,
                   false,
                   true) == ROUTE_EVENT_NONE);
    }
    assert(route_event_step(
               &EVENT_CONFIG,
               &detector,
               ROUTE_DESTINATION_ENDPOINT,
               0U,
               false,
               true) == ROUTE_EVENT_ENDPOINT);
}

static void test_uncentered_loss_cannot_be_an_endpoint(void)
{
    route_event_state_t detector;
    uint32_t index;

    route_event_init(&detector);
    feed_normal_frames(
        &detector,
        ROUTE_DESTINATION_ENDPOINT,
        EVENT_CONFIG.min_travel_frames);
    assert(route_event_step(
               &EVENT_CONFIG,
               &detector,
               ROUTE_DESTINATION_ENDPOINT,
               1U,
               false,
               true) == ROUTE_EVENT_NONE);

    for (index = 0U; index <
         (uint32_t)(EVENT_CONFIG.endpoint_confirm_frames + 2U); index++) {
        assert(route_event_step(
                   &EVENT_CONFIG,
                   &detector,
                   ROUTE_DESTINATION_ENDPOINT,
                   0U,
                   false,
                   true) == ROUTE_EVENT_NONE);
    }
    assert(detector.lost_frames == 0U);
}

static void test_endpoint_distance_window_uses_both_wheels(void)
{
    route_progress_state_t progress;

    route_progress_begin(&progress, 10000, 20000, 1000U, 3000U);
    assert(route_progress_step(
               &PROGRESS_CONFIG,
               &progress,
               8800,
               21300) == ROUTE_PROGRESS_FAULT_NONE);
    assert(progress.left_forward_edges == 1200U);
    assert(progress.right_forward_edges == 1300U);
    assert(progress.average_forward_edges == 1250U);
    assert(route_progress_endpoint_ready(&progress));
}

static void test_encoder_faults_are_fail_closed(void)
{
    route_progress_state_t progress;
    uint32_t index;

    route_progress_begin(&progress, 0, 0, 1000U, 3000U);
    assert(route_progress_step(
               &PROGRESS_CONFIG,
               &progress,
               101,
               0) == ROUTE_PROGRESS_FAULT_DIRECTION);

    route_progress_begin(&progress, 0, 0, 1000U, 3000U);
    for (index = 1U; index < PROGRESS_CONFIG.no_motion_timeout_frames; index++) {
        assert(route_progress_step(
                   &PROGRESS_CONFIG,
                   &progress,
                   0,
                   0) == ROUTE_PROGRESS_FAULT_NONE);
    }
    assert(route_progress_step(
               &PROGRESS_CONFIG,
               &progress,
               0,
               0) == ROUTE_PROGRESS_FAULT_NO_MOTION);

    route_progress_begin(&progress, 0, 0, 1000U, 3000U);
    assert(route_progress_step(
               &PROGRESS_CONFIG,
               &progress,
               -500,
               500) == ROUTE_PROGRESS_FAULT_NONE);
    for (index = 1U; index < PROGRESS_CONFIG.no_motion_timeout_frames; index++) {
        assert(route_progress_step(
                   &PROGRESS_CONFIG,
                   &progress,
                   -500,
                   500) == ROUTE_PROGRESS_FAULT_NONE);
    }
    assert(route_progress_step(
               &PROGRESS_CONFIG,
               &progress,
               -500,
               500) == ROUTE_PROGRESS_FAULT_NO_MOTION);

    route_progress_begin(&progress, 0, 0, 1000U, 3000U);
    assert(route_progress_step(
               &PROGRESS_CONFIG,
               &progress,
               -500,
               500) == ROUTE_PROGRESS_FAULT_NONE);
    for (index = 1U; index < PROGRESS_CONFIG.no_motion_timeout_frames; index++) {
        assert(route_progress_step(
                   &PROGRESS_CONFIG,
                   &progress,
                   -500,
                   (int32_t)(500U + (index * 10U))) ==
               ROUTE_PROGRESS_FAULT_NONE);
    }
    assert(route_progress_step(
               &PROGRESS_CONFIG,
               &progress,
               -500,
               1000) == ROUTE_PROGRESS_FAULT_WHEEL_IMBALANCE);

    route_progress_begin(&progress, 0, 0, 1000U, 3000U);
    assert(route_progress_step(
               &PROGRESS_CONFIG,
               &progress,
               -1200,
               50) == ROUTE_PROGRESS_FAULT_WHEEL_IMBALANCE);

    route_progress_begin(&progress, 0, 0, 1000U, 10000U);
    assert(route_progress_step(
               &PROGRESS_CONFIG,
               &progress,
               -4000,
               1000) == ROUTE_PROGRESS_FAULT_WHEEL_IMBALANCE);

    route_progress_begin(&progress, 0, 0, 1000U, 3000U);
    assert(route_progress_step(
               &PROGRESS_CONFIG,
               &progress,
               -2950,
               2950) == ROUTE_PROGRESS_FAULT_NONE);
    assert(route_progress_endpoint_ready(&progress));
    assert(route_progress_step(
               &PROGRESS_CONFIG,
               &progress,
               -3100,
               3100) == ROUTE_PROGRESS_FAULT_NONE);
    assert(!route_progress_endpoint_ready(&progress));
    assert(route_progress_step(
               &PROGRESS_CONFIG,
               &progress,
               -3201,
               3201) == ROUTE_PROGRESS_FAULT_ENDPOINT_OVERRUN);
}

static void test_event_type_cannot_be_confused(void)
{
    route_event_state_t endpoint_detector;
    route_event_state_t junction_detector;
    uint32_t index;

    route_event_init(&endpoint_detector);
    route_event_init(&junction_detector);
    feed_normal_frames(
        &endpoint_detector,
        ROUTE_DESTINATION_ENDPOINT,
        EVENT_CONFIG.min_travel_frames);
    feed_normal_frames(
        &junction_detector,
        ROUTE_DESTINATION_JUNCTION,
        EVENT_CONFIG.min_travel_frames);

    for (index = 0U; index < 10U; index++) {
        assert(step_event(
                   &endpoint_detector,
                   ROUTE_DESTINATION_ENDPOINT,
                   8U) == ROUTE_EVENT_NONE);
        assert(step_event(
                   &junction_detector,
                   ROUTE_DESTINATION_JUNCTION,
                   0U) == ROUTE_EVENT_NONE);
    }
}

static gyro_turn_output_t step_turn(
    gyro_turn_state_t *state,
    int32_t yaw_mdeg,
    int32_t rate_mdps,
    uint16_t delta_ms)
{
    return gyro_turn_step(
        &TURN_CONFIG, state, yaw_mdeg, rate_mdps, delta_ms);
}

static void test_left_turn_centers_axle_then_closes_on_gyro(void)
{
    gyro_turn_state_t turn;
    gyro_turn_output_t output;

    gyro_turn_init(&turn, 90000, 20U, 2000U);
    output = step_turn(&turn, 0, 0, 10U);
    assert(output.motion == GYRO_TURN_MOTION_FORWARD);
    output = step_turn(&turn, 0, 0, 10U);
    assert(output.motion == GYRO_TURN_MOTION_STOP);
    assert(turn.phase == GYRO_TURN_PHASE_SETTLE);

    output = step_turn(&turn, 0, 0, 10U);
    assert(output.motion == GYRO_TURN_MOTION_STOP);
    output = step_turn(&turn, 0, 0, 10U);
    assert(output.reset_yaw);
    assert(turn.phase == GYRO_TURN_PHASE_ROTATE);

    output = step_turn(&turn, 0, 0, 10U);
    assert(output.motion == GYRO_TURN_MOTION_SPIN_LEFT_FAST);
    output = step_turn(&turn, 75000, 12000, 10U);
    assert(output.motion == GYRO_TURN_MOTION_SPIN_LEFT_SLOW);
    output = step_turn(&turn, 89000, 1000, 10U);
    assert(output.motion == GYRO_TURN_MOTION_BRAKE);
    assert(!output.complete);
    output = step_turn(&turn, 89500, 800, 10U);
    assert(output.motion == GYRO_TURN_MOTION_BRAKE);
    output = step_turn(&turn, 90000, 500, 10U);
    assert(output.complete);
    assert(turn.phase == GYRO_TURN_PHASE_COMPLETE);
}

static void test_turn_corrects_overshoot_and_supports_right_turn(void)
{
    gyro_turn_state_t turn;
    gyro_turn_output_t output;

    gyro_turn_init(&turn, 90000, 0U, 2000U);
    step_turn(&turn, 0, 0, 20U);
    output = step_turn(&turn, 95000, 0, 10U);
    assert(output.motion == GYRO_TURN_MOTION_SPIN_RIGHT_SLOW);

    gyro_turn_init(&turn, -90000, 0U, 2000U);
    step_turn(&turn, 0, 0, 20U);
    output = step_turn(&turn, 0, 0, 10U);
    assert(output.motion == GYRO_TURN_MOTION_SPIN_RIGHT_FAST);
}

static void test_turn_timeout_is_a_fault(void)
{
    gyro_turn_state_t turn;
    gyro_turn_output_t output;

    gyro_turn_init(&turn, 180000, 0U, 30U);
    output = step_turn(&turn, 0, 0, 31U);
    assert(output.fault);
    assert(turn.phase == GYRO_TURN_PHASE_FAULT);
}

static void test_intermittent_gyro_failures_are_fail_closed(void)
{
    gyro_turn_state_t turn;
    gyro_turn_output_t output;

    gyro_turn_init(&turn, 90000, 0U, 2000U);
    output = gyro_turn_sensor_failure(&TURN_CONFIG, &turn, 10U);
    assert(!output.fault);
    step_turn(&turn, 0, 0, 10U);
    output = gyro_turn_sensor_failure(&TURN_CONFIG, &turn, 10U);
    assert(!output.fault);
    step_turn(&turn, 0, 0, 10U);
    output = gyro_turn_sensor_failure(&TURN_CONFIG, &turn, 10U);
    assert(output.fault);
    assert(turn.sensor_failure_count == 3U);
    assert(turn.total_elapsed_ms == 50U);
    assert(turn.phase == GYRO_TURN_PHASE_FAULT);
}

static void test_full_course_sensor_and_gyro_simulation(void)
{
    medicine_mission_state_t mission;
    route_event_state_t detector;
    route_progress_state_t progress;
    gyro_turn_state_t turn;
    gyro_turn_motion_t previous_motion = GYRO_TURN_MOTION_STOP;
    uint8_t active_action_index = UINT8_MAX;
    uint16_t follow_frame = 0U;
    int32_t simulated_yaw_mdeg = 0;
    int32_t simulated_left_encoder = 0;
    int32_t simulated_right_encoder = 0;
    uint32_t junction_events = 0U;
    uint32_t endpoint_events = 0U;
    uint32_t completed_turns = 0U;
    uint32_t guard = 0U;

    medicine_mission_init(&mission);
    while (!mission.completed && !mission.fault && (guard < 10000U)) {
        const medicine_route_action_t *action =
            medicine_mission_current_action(&mission);
        medicine_mission_input_t input = {0};

        assert(action != NULL);
        if (active_action_index != mission.action_index) {
            active_action_index = mission.action_index;
            follow_frame = 0U;
            simulated_yaw_mdeg = 0;
            previous_motion = GYRO_TURN_MOTION_STOP;
            if (medicine_route_is_follow_action(action->kind)) {
                const uint32_t simulated_edges_per_cm = 100U;

                route_event_init(&detector);
                route_progress_begin(
                    &progress,
                    simulated_left_encoder,
                    simulated_right_encoder,
                    (uint32_t)medicine_route_endpoint_minimum_cm(action) *
                        simulated_edges_per_cm,
                    (uint32_t)medicine_route_endpoint_maximum_cm(action) *
                        simulated_edges_per_cm);
            } else if (medicine_route_is_turn_action(action->kind)) {
                gyro_turn_init(
                    &turn,
                    medicine_route_turn_target_mdeg(action->kind),
                    (action->kind == MEDICINE_ACTION_TURN_AROUND_180)
                        ? 0U
                        : 20U,
                    5000U);
            }
        }

        if (medicine_route_is_follow_action(action->kind)) {
            const route_destination_t destination =
                medicine_route_destination(action->kind);
            route_event_t event;
            route_progress_fault_t progress_fault;
            uint8_t active_count;
            bool line_centered;

            if (follow_frame < 3U) {
                active_count = 8U;
            } else if (follow_frame <
                       (uint16_t)(3U + EVENT_CONFIG.min_travel_frames)) {
                active_count = 2U;
            } else {
                active_count =
                    (destination == ROUTE_DESTINATION_JUNCTION) ? 8U : 0U;
            }
            simulated_left_encoder -= 100;
            simulated_right_encoder += 100;
            progress_fault = route_progress_step(
                &PROGRESS_CONFIG,
                &progress,
                simulated_left_encoder,
                simulated_right_encoder);
            assert(progress_fault == ROUTE_PROGRESS_FAULT_NONE);
            line_centered = active_count == 2U;
            event = route_event_step(
                &EVENT_CONFIG,
                &detector,
                destination,
                active_count,
                line_centered,
                route_progress_endpoint_ready(&progress));
            if (event != ROUTE_EVENT_NONE) {
                input.destination_reached = true;
                if (event == ROUTE_EVENT_JUNCTION) {
                    junction_events++;
                } else {
                    endpoint_events++;
                }
            }
            follow_frame++;
        } else if (medicine_route_is_turn_action(action->kind)) {
            gyro_turn_output_t output;
            int32_t simulated_rate_mdps = 0;

            if (previous_motion == GYRO_TURN_MOTION_SPIN_LEFT_FAST) {
                simulated_yaw_mdeg += 5000;
                simulated_rate_mdps = 500000;
            } else if (previous_motion == GYRO_TURN_MOTION_SPIN_LEFT_SLOW) {
                simulated_yaw_mdeg += 1000;
                simulated_rate_mdps = 100000;
            } else if (previous_motion == GYRO_TURN_MOTION_SPIN_RIGHT_FAST) {
                simulated_yaw_mdeg -= 5000;
                simulated_rate_mdps = -500000;
            } else if (previous_motion == GYRO_TURN_MOTION_SPIN_RIGHT_SLOW) {
                simulated_yaw_mdeg -= 1000;
                simulated_rate_mdps = -100000;
            }

            output = gyro_turn_step(
                &TURN_CONFIG,
                &turn,
                simulated_yaw_mdeg,
                simulated_rate_mdps,
                10U);
            assert(!output.fault);
            if (output.reset_yaw) {
                simulated_yaw_mdeg = 0;
            }
            previous_motion = output.motion;
            if (output.complete) {
                input.turn_complete = true;
                completed_turns++;
            }
        } else if (action->kind == MEDICINE_ACTION_WARD_STOP_2S) {
            input.delta_ms = 10U;
        }

        medicine_mission_step(&mission, &input);
        guard++;
    }

    assert(guard < 10000U);
    assert(!mission.fault);
    assert(mission.completed);
    assert(mission.visited_ward_mask == 0xFFU);
    assert(mission.ward_stop_count == 8U);
    assert(junction_events == 17U);
    assert(endpoint_events == 9U);
    assert(completed_turns == 18U);
}

int main(void)
{
    test_route_table_matches_full_course();
    test_every_endpoint_has_a_distance_window();
    test_mission_visits_every_ward_and_returns_home();
    test_ward_stop_is_exactly_two_seconds();
    test_junction_requires_rearm_and_does_not_double_trigger();
    test_endpoint_requires_confirmed_loss_after_real_travel();
    test_midcourse_loss_never_advances_endpoint();
    test_uncentered_loss_cannot_be_an_endpoint();
    test_endpoint_distance_window_uses_both_wheels();
    test_encoder_faults_are_fail_closed();
    test_event_type_cannot_be_confused();
    test_left_turn_centers_axle_then_closes_on_gyro();
    test_turn_corrects_overshoot_and_supports_right_turn();
    test_turn_timeout_is_a_fault();
    test_intermittent_gyro_failures_are_fail_closed();
    test_mission_faults_are_fail_closed();
    test_control_tick_overrun_boundary();
    test_full_course_sensor_and_gyro_simulation();
    puts("medicine_route_host_test: 18/18 passed");
    return 0;
}
