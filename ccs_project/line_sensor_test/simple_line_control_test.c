#include <assert.h>
#include <stdio.h>

#include "simple_line_control.h"
#include "simple_line_profile.h"

static const simple_line_config_t TEST_CONFIG =
    SIMPLE_LINE_PROFILE_CONFIG_INITIALIZER;

static void test_center_keeps_base_speed(void)
{
    const simple_line_output_t output = simple_line_step(
        &TEST_CONFIG, true, 0, 0, 0U);

    assert(output.correction == 0);
    assert(output.left_pwm == 380U);
    assert(output.right_pwm == 383U);
    assert(!output.stop);
}

static void test_positive_error_turns_right(void)
{
    const simple_line_output_t output = simple_line_step(
        &TEST_CONFIG, true, 1800, 1800, 0U);

    assert(output.correction == 69);
    assert(output.left_pwm == 449U);
    assert(output.right_pwm == 314U);
}

static void test_negative_error_turns_left(void)
{
    const simple_line_output_t output = simple_line_step(
        &TEST_CONFIG, true, -1800, -1800, 0U);

    assert(output.correction == -69);
    assert(output.left_pwm == 311U);
    assert(output.right_pwm == 452U);
}

static void test_center_sensors_correct_before_outer_sensors(void)
{
    const simple_line_output_t x5 = simple_line_step(
        &TEST_CONFIG, true, 500, 500, 0U);
    const simple_line_output_t x6 = simple_line_step(
        &TEST_CONFIG, true, 1500, 1500, 0U);
    const simple_line_output_t x7 = simple_line_step(
        &TEST_CONFIG, true, 2500, 2500, 0U);
    const simple_line_output_t x8 = simple_line_step(
        &TEST_CONFIG, true, 3500, 3500, 0U);

    assert(x5.correction == 19);
    assert(x6.correction == 57);
    assert(x7.correction == 96);
    assert(x8.correction == 110);
    assert(x5.correction < x6.correction);
    assert(x6.correction < x7.correction);
    assert(x7.correction < x8.correction);
}

static void test_white_high_raw_mask_is_normalized_to_line_mask(void)
{
    assert(simple_line_normalize_mask(0xC3U, 0xFFU) == 0x3CU);
    assert(simple_line_normalize_mask(0xE7U, 0xFFU) == 0x18U);
}

static void test_sensor_mask_maps_to_physical_line_error(void)
{
    uint8_t active_count = 0U;

    assert(simple_line_error_from_mask(0x01U, &active_count) == -3500);
    assert(active_count == 1U);
    assert(simple_line_error_from_mask(0x02U, &active_count) == -2500);
    assert(simple_line_error_from_mask(0x04U, &active_count) == -1500);
    assert(simple_line_error_from_mask(0x08U, &active_count) == -500);
    assert(simple_line_error_from_mask(0x10U, &active_count) == 500);
    assert(simple_line_error_from_mask(0x20U, &active_count) == 1500);
    assert(simple_line_error_from_mask(0x40U, &active_count) == 2500);
    assert(simple_line_error_from_mask(0x80U, &active_count) == 3500);
    assert(simple_line_error_from_mask(0x18U, &active_count) == 0);
    assert(active_count == 2U);
    assert(simple_line_error_from_mask(0x3CU, &active_count) == 0);
    assert(active_count == 4U);
    assert(simple_line_error_from_mask(0x00U, &active_count) == 0);
    assert(active_count == 0U);
}

static void test_lost_line_searches_using_last_direction(void)
{
    const simple_line_output_t output = simple_line_step(
        &TEST_CONFIG, false, 0, 1800, 0U);

    assert(output.lost_count == 1U);
    assert(output.correction == 70);
    assert(output.left_pwm == 450U);
    assert(output.right_pwm == 313U);
    assert(!output.stop);
}

static void test_lost_timeout_stops(void)
{
    const simple_line_output_t output = simple_line_step(
        &TEST_CONFIG, false, 0, -1800, 199U);

    assert(output.lost_count == 200U);
    assert(output.stop);
    assert(output.left_pwm == 0U);
    assert(output.right_pwm == 0U);
}

static void test_correction_is_limited(void)
{
    const simple_line_output_t positive = simple_line_step(
        &TEST_CONFIG, true, 10000, 10000, 0U);
    const simple_line_output_t negative = simple_line_step(
        &TEST_CONFIG, true, -10000, -10000, 0U);

    assert(positive.correction == 110);
    assert(positive.left_pwm == 490U);
    assert(positive.right_pwm == 273U);
    assert(negative.correction == -110);
    assert(negative.left_pwm == 270U);
    assert(negative.right_pwm == 493U);
}

int main(void)
{
    test_center_keeps_base_speed();
    test_positive_error_turns_right();
    test_negative_error_turns_left();
    test_center_sensors_correct_before_outer_sensors();
    test_white_high_raw_mask_is_normalized_to_line_mask();
    test_sensor_mask_maps_to_physical_line_error();
    test_lost_line_searches_using_last_direction();
    test_lost_timeout_stops();
    test_correction_is_limited();
    puts("simple_line_control_test: 9/9 passed");
    return 0;
}
