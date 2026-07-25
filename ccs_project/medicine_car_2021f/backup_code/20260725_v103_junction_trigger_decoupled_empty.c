/*
 * V103: decoupled junction trigger and normal-line thresholds.
 *
 * Wiring:
 *   PB5 -> AD0, PB6 -> AD1, PB7 -> AD2, PA7 <- OUT.
 *   PA12 -> PWMA, PA13 -> PWMB.
 *   PB0/PB1 -> AIN1/AIN2, PB2/PB3 -> BIN1/BIN2, PB4 -> STBY.
 *   PB12/PB13 <- left encoder A/B, PB14/PB15 <- right encoder A/B.
 *
 * Route: ward 1 -> 2 -> 3 -> 4 -> 7 -> 5 -> 6 -> 8 -> pharmacy.
 * Every ward stop lasts 2 seconds, followed by an in-place 180-degree turn.
 * Junction turns use the MPU Z gyro. OpenMV remains disabled.
 */

#include <stdbool.h>
#include <stdint.h>

#include "encoder_calibration_profile.h"
#include "gyro_turn_control.h"
#include "medicine_mission.h"
#include "medicine_route.h"
#include "route_event_detector.h"
#include "route_progress_guard.h"
#include "simple_line_control.h"
#include "simple_line_profile.h"
#include "ti_msp_dl_config.h"

#define NOINLINE                    __attribute__((noinline))

#define FIRMWARE_VERSION            103U
#define SINGLE_TURN_TEST_ENABLE     1U
#define LINE_SENSOR_ERROR_SIGN      (-1)
#define CPU_CLOCK_HZ                32000000U

#define GRAY_CHANNEL_COUNT          8U
#define GRAY_SAMPLE_COUNT           9U
#define GRAY_HIGH_MAJORITY          5U
#define GRAY_ADDRESS_SETTLE_US      40U
#define GRAY_SAMPLE_INTERVAL_US     10U
#define GRAY_BASELINE_SCAN_COUNT    50U
#define GRAY_FIXED_WHITE_BASELINE_MASK 0xFFU
#define MISSION_TICK_MS             10U
#define GRAY_MAIN_LOOP_MS           MISSION_TICK_MS
#define GRAY_POWERUP_DELAY_MS       300U
#define DELAY_LOOPS_PER_US          8U

#define PHASE_INIT                  0U
#define PHASE_CALIBRATION           1U
#define PHASE_MISSION_FOLLOW        2U
#define PHASE_COMPLETE              3U
#define PHASE_STOP                  4U
#define PHASE_GYRO_TURN             5U
#define PHASE_WARD_STOP             6U

#define STOP_REASON_NONE            0U
#define STOP_REASON_TEST_COMPLETE   1U
#define STOP_REASON_BASELINE_INVALID 2U
#define STOP_REASON_LINE_LOST       3U
#define STOP_REASON_MPU_INIT        4U
#define STOP_REASON_MPU_CALIBRATION 5U
#define STOP_REASON_MPU_READ        6U
#define STOP_REASON_TURN_TIMEOUT    7U
#define STOP_REASON_MISSION_FAULT   8U
#define STOP_REASON_MISSION_TIMEOUT 9U
#define STOP_REASON_MANUAL          10U
#define STOP_REASON_CONTROL_OVERRUN 11U
#define STOP_REASON_ENCODER_FAULT   12U
#define STOP_REASON_ENDPOINT_MISSED 13U

#define PWM_TIMER                   TIMG0
#define PWM_PERIOD                  1600U
#define PWM_LEFT_BASE_TICKS         SIMPLE_LINE_PROFILE_LEFT_BASE
#define PWM_RIGHT_BASE_TICKS        SIMPLE_LINE_PROFILE_RIGHT_BASE
#define LINE_MAX_ACTIVE_SENSORS     6U
#define ENDPOINT_CENTER_ERROR_MAX   500
#define FIRST_JUNCTION_MIN_TRAVEL_FRAMES 180U
#define JUNCTION_TRIGGER_ACTIVE_MIN 4U
#define LINE_WIDE_ACTIVE_MIN        7U
#define JUNCTION_WIDE_CONFIRM_FRAMES 1U

#define START_READY_HOLD_MS         2000U
#define MISSION_TIMEOUT_MS          240000U

#define GRAY_TO_TURN_CENTER_MM      175U
#define JUNCTION_CENTER_TARGET_EDGES \
    (((uint32_t)GRAY_TO_TURN_CENTER_MM * ENCODER_RISING_EDGES_PER_CM + 5U) / 10U)
#define JUNCTION_CENTER_MAX_EDGES \
    (JUNCTION_CENTER_TARGET_EDGES + ENCODER_RISING_EDGES_PER_CM)
#define JUNCTION_CENTER_LEFT_PWM    300U
#define JUNCTION_CENTER_RIGHT_PWM   303U
#define JUNCTION_CENTER_HEADING_TRIM_DIVISOR 250
#define JUNCTION_CENTER_ENCODER_TRIM_DIVISOR 64
#define JUNCTION_CENTER_HEADING_TRIM_MAX 60
#define ENDPOINT_CONFIRM_LEFT_PWM   300U
#define ENDPOINT_CONFIRM_RIGHT_PWM  303U

#define TURN_SETTLE_MS              100U
#define TURN_SLOW_ZONE_MDEG         20000
#define TURN_TOLERANCE_MDEG         2500
#define TURN_STOP_RATE_MDPS         4000
#define TURN_STABLE_FRAMES          5U
#define TURN_90_TIMEOUT_MS          4000U
#define TURN_180_TIMEOUT_MS         6500U
#define TURN_FAST_PWM               360U
#define TURN_SLOW_PWM               270U

#define MPU_I2C                     I2C0
#define MPU_I2C_ADDRESS             0x68U
#define MPU_I2C_TIMEOUT             200000U
#define MPU_I2C_TIMER_PERIOD_100KHZ 31U

#define MPU_REG_SMPLRT_DIV          0x19U
#define MPU_REG_CONFIG              0x1AU
#define MPU_REG_GYRO_CONFIG         0x1BU
#define MPU_REG_GYRO_XOUT_H         0x43U
#define MPU_REG_PWR_MGMT_1          0x6BU
#define MPU_REG_WHO_AM_I            0x75U

#define MPU6050_WHO_AM_I            0x68U
#define MPU6500_WHO_AM_I            0x70U
#define MPU_GYRO_FS_500DPS          0x08U
#define MPU_CALIBRATION_SAMPLES     500U
#define MPU_MAX_CONSECUTIVE_FAILURES 3U
#define MPU_GYRO_STATIONARY_LIMIT_MDPS 3000
#define MPU_LEFT_POSITIVE_SIGN      1

#define WARD_BIAS_SETTLE_MS         500U
#define WARD_BIAS_MIN_SAMPLES       50U

#define ENCODER_DIRECTION_TOLERANCE_EDGES \
    (2U * ENCODER_RISING_EDGES_PER_CM)
#define ENCODER_NO_MOTION_FRAMES    50U
#define ENCODER_IMBALANCE_CHECK_EDGES \
    (5U * ENCODER_RISING_EDGES_PER_CM)
#define ENCODER_MAX_WHEEL_RATIO     4U
#define ENCODER_ENDPOINT_CONFIRM_GRACE_EDGES \
    (5U * ENCODER_RISING_EDGES_PER_CM)

#define MOTOR_PORT                  GPIOB
#define AIN1_PIN                    DL_GPIO_PIN_0
#define AIN2_PIN                    DL_GPIO_PIN_1
#define BIN1_PIN                    DL_GPIO_PIN_2
#define BIN2_PIN                    DL_GPIO_PIN_3
#define STBY_PIN                    DL_GPIO_PIN_4

#define PWM_PORT                    GPIOA
#define PWMA_PIN                    DL_GPIO_PIN_12
#define PWMB_PIN                    DL_GPIO_PIN_13

#define GRAY_ADDRESS_PORT           GPIOB
#define GRAY_AD0_PIN                DL_GPIO_PIN_5
#define GRAY_AD1_PIN                DL_GPIO_PIN_6
#define GRAY_AD2_PIN                DL_GPIO_PIN_7
#define GRAY_ADDRESS_MASK           \
    (GRAY_AD0_PIN | GRAY_AD1_PIN | GRAY_AD2_PIN)

#define GRAY_OUT_PORT               GPIOA
#define GRAY_OUT_PIN                DL_GPIO_PIN_7

#define SERVO_PORT                  GPIOA
#define SERVO_X_PIN                 DL_GPIO_PIN_14
#define SERVO_Y_PIN                 DL_GPIO_PIN_17

#define LED_PORT                    GPIOA
#define LED_PIN                     DL_GPIO_PIN_15

#define BUZZER_PORT                 GPIOB
#define BUZZER_PIN                  DL_GPIO_PIN_11

#define ENCODER_PORT                GPIOB
#define ENCODER_LEFT_A_PIN          DL_GPIO_PIN_12
#define ENCODER_LEFT_B_PIN          DL_GPIO_PIN_13
#define ENCODER_RIGHT_A_PIN         DL_GPIO_PIN_14
#define ENCODER_RIGHT_B_PIN         DL_GPIO_PIN_15
#define ENCODER_INTERRUPT_MASK      \
    (ENCODER_LEFT_A_PIN | ENCODER_RIGHT_A_PIN)

static const simple_line_config_t LINE_CONTROL_CONFIG =
    SIMPLE_LINE_PROFILE_CONFIG_INITIALIZER;

static const route_event_config_t ROUTE_EVENT_CONFIG = {
    .wide_active_min = JUNCTION_TRIGGER_ACTIVE_MIN,
    .wide_confirm_frames = JUNCTION_WIDE_CONFIRM_FRAMES,
    .endpoint_confirm_frames = 8U,
    .endpoint_centered_frames = 5U,
    .rearm_normal_frames = 5U,
    .min_travel_frames = 30U,
};

static const route_event_config_t FIRST_JUNCTION_ROUTE_EVENT_CONFIG = {
    .wide_active_min = JUNCTION_TRIGGER_ACTIVE_MIN,
    .wide_confirm_frames = JUNCTION_WIDE_CONFIRM_FRAMES,
    .endpoint_confirm_frames = 8U,
    .endpoint_centered_frames = 5U,
    .rearm_normal_frames = 5U,
    .min_travel_frames = FIRST_JUNCTION_MIN_TRAVEL_FRAMES,
};

static const route_progress_config_t ROUTE_PROGRESS_CONFIG = {
    .direction_tolerance_edges = ENCODER_DIRECTION_TOLERANCE_EDGES,
    .no_motion_timeout_frames = ENCODER_NO_MOTION_FRAMES,
    .imbalance_check_edges = ENCODER_IMBALANCE_CHECK_EDGES,
    .maximum_wheel_ratio = ENCODER_MAX_WHEEL_RATIO,
    .endpoint_confirmation_grace_edges =
        ENCODER_ENDPOINT_CONFIRM_GRACE_EDGES,
};

static const gyro_turn_config_t GYRO_TURN_CONFIG = {
    .settle_ms = TURN_SETTLE_MS,
    .slow_zone_mdeg = TURN_SLOW_ZONE_MDEG,
    .tolerance_mdeg = TURN_TOLERANCE_MDEG,
    .stopped_rate_limit_mdps = TURN_STOP_RATE_MDPS,
    .stable_frames_required = TURN_STABLE_FRAMES,
    .max_sensor_failures = MPU_MAX_CONSECUTIVE_FAILURES,
};

volatile uint32_t g_firmware_version = FIRMWARE_VERSION;
volatile uint8_t g_motor_disabled = 0U;
volatile uint8_t g_buzzer_idle_level = 1U;

volatile uint8_t g_phase = PHASE_INIT;
volatile uint8_t g_stop_reason = STOP_REASON_NONE;
volatile uint32_t g_run_frame = 0U;
volatile uint32_t g_system_ms = 0U;
volatile uint32_t g_mission_elapsed_ms = 0U;
volatile uint32_t g_mission_tick_work_ms = 0U;
volatile uint32_t g_mission_tick_overrun_count = 0U;

volatile uint8_t g_route_action_index = 0U;
volatile uint8_t g_route_action_kind = MEDICINE_ACTION_FOLLOW_TO_JUNCTION;
volatile uint8_t g_route_action_ward = 0U;
volatile uint8_t g_visited_ward_mask = 0U;
volatile uint8_t g_ward_stop_count = 0U;
volatile uint32_t g_route_action_elapsed_ms = 0U;
volatile uint8_t g_route_complete = 0U;
volatile uint8_t g_route_fault = 0U;

volatile uint8_t g_route_event = ROUTE_EVENT_NONE;
volatile uint8_t g_route_event_armed = 0U;
volatile uint16_t g_route_travel_frames = 0U;
volatile uint8_t g_route_normal_frames = 0U;
volatile uint8_t g_route_wide_frames = 0U;
volatile uint8_t g_route_lost_frames = 0U;
volatile uint8_t g_route_centered_frames = 0U;
volatile uint8_t g_route_endpoint_loss_active = 0U;
volatile uint8_t g_route_endpoint_entry_centered = 0U;
volatile uint8_t g_route_endpoint_entry_distance_ready = 0U;
volatile uint32_t g_junction_event_count = 0U;
volatile uint32_t g_endpoint_event_count = 0U;

volatile uint8_t g_gray_test_state = 0U;
volatile uint8_t g_gray_selected_channel = 0U;
volatile uint8_t g_gray_live_out = 0U;
volatile uint8_t g_gray_raw_high_mask = 0U;
volatile uint8_t g_gray_unstable_mask = 0U;
volatile uint8_t g_gray_white_baseline_mask = 0U;
volatile uint8_t g_gray_changed_from_white_mask = 0U;
volatile uint8_t g_gray_previous_change_mask = 0U;

volatile uint8_t g_gray_x1 = 0U;
volatile uint8_t g_gray_x2 = 0U;
volatile uint8_t g_gray_x3 = 0U;
volatile uint8_t g_gray_x4 = 0U;
volatile uint8_t g_gray_x5 = 0U;
volatile uint8_t g_gray_x6 = 0U;
volatile uint8_t g_gray_x7 = 0U;
volatile uint8_t g_gray_x8 = 0U;

volatile uint8_t g_gray_line_x1 = 0U;
volatile uint8_t g_gray_line_x2 = 0U;
volatile uint8_t g_gray_line_x3 = 0U;
volatile uint8_t g_gray_line_x4 = 0U;
volatile uint8_t g_gray_line_x5 = 0U;
volatile uint8_t g_gray_line_x6 = 0U;
volatile uint8_t g_gray_line_x7 = 0U;
volatile uint8_t g_gray_line_x8 = 0U;

volatile uint8_t g_gray_x1_high_count = 0U;
volatile uint8_t g_gray_x2_high_count = 0U;
volatile uint8_t g_gray_x3_high_count = 0U;
volatile uint8_t g_gray_x4_high_count = 0U;
volatile uint8_t g_gray_x5_high_count = 0U;
volatile uint8_t g_gray_x6_high_count = 0U;
volatile uint8_t g_gray_x7_high_count = 0U;
volatile uint8_t g_gray_x8_high_count = 0U;

volatile uint8_t g_gray_baseline_ready = 0U;
volatile uint8_t g_gray_baseline_uniform = 0U;
volatile uint8_t g_gray_white_level_code = 2U;
volatile uint8_t g_gray_changed_count = 0U;
volatile uint8_t g_gray_line_valid = 0U;
volatile uint8_t g_gray_line_mask = 0U;
volatile uint8_t g_gray_line_abnormal = 0U;
volatile int16_t g_gray_line_error = 0;
volatile int16_t g_line_last_error = 0;
volatile int16_t g_line_last_nonzero_error = 0;
volatile int32_t g_line_correction = 0;
volatile uint32_t g_line_lost_count = 0U;

volatile uint32_t g_left_pwm_ticks = 0U;
volatile uint32_t g_right_pwm_ticks = 0U;
volatile uint32_t g_left_compare_ticks = PWM_PERIOD;
volatile uint32_t g_right_compare_ticks = PWM_PERIOD;

volatile int32_t g_encoder_left_count = 0;
volatile int32_t g_encoder_right_count = 0;
volatile uint32_t g_encoder_left_edges = 0U;
volatile uint32_t g_encoder_right_edges = 0U;
volatile uint8_t g_encoder_left_a_level = 0U;
volatile uint8_t g_encoder_left_b_level = 0U;
volatile uint8_t g_encoder_right_a_level = 0U;
volatile uint8_t g_encoder_right_b_level = 0U;
volatile int32_t g_route_left_start_count = 0;
volatile int32_t g_route_right_start_count = 0;
volatile uint32_t g_route_left_forward_edges = 0U;
volatile uint32_t g_route_right_forward_edges = 0U;
volatile uint32_t g_route_average_forward_edges = 0U;
volatile uint32_t g_route_encoder_no_motion_frames = 0U;
volatile uint32_t g_route_left_no_motion_frames = 0U;
volatile uint32_t g_route_right_no_motion_frames = 0U;
volatile uint32_t g_route_endpoint_min_edges = 0U;
volatile uint32_t g_route_endpoint_max_edges = 0U;
volatile uint8_t g_route_endpoint_distance_ready = 0U;
volatile uint8_t g_route_progress_fault = ROUTE_PROGRESS_FAULT_NONE;

volatile uint16_t g_gray_baseline_scan_count = 0U;
volatile uint16_t g_gray_baseline_votes[GRAY_CHANNEL_COUNT] = {0U};
volatile uint8_t g_gray_channel_high_count[GRAY_CHANNEL_COUNT] = {0U};
volatile uint32_t g_gray_scan_count = 0U;
volatile uint32_t g_gray_change_event_count = 0U;
volatile uint8_t g_led_state = 0U;

volatile uint8_t g_mpu_state = 0U;
volatile uint8_t g_mpu_error = 0U;
volatile uint8_t g_mpu_link_ok = 0U;
volatile uint8_t g_mpu_who_am_i = 0U;
volatile uint8_t g_mpu_device_type = 0U;
volatile uint32_t g_i2c_error_count = 0U;
volatile uint32_t g_mpu_sample_count = 0U;
volatile uint32_t g_mpu_read_fail_count = 0U;
volatile uint32_t g_mpu_read_fail_total = 0U;
volatile uint32_t g_calibration_sample_count = 0U;

volatile int16_t g_gyro_x_raw = 0;
volatile int16_t g_gyro_y_raw = 0;
volatile int16_t g_gyro_z_raw = 0;
volatile int32_t g_gyro_z_bias_raw = 0;
volatile int32_t g_gyro_z_corrected_raw = 0;
volatile int32_t g_gyro_z_mdps = 0;
volatile int32_t g_gyro_z_left_positive_mdps = 0;

volatile uint8_t g_turn_phase = GYRO_TURN_PHASE_SETTLE;
volatile uint8_t g_turn_motion = GYRO_TURN_MOTION_STOP;
volatile int32_t g_turn_target_mdeg = 0;
volatile int32_t g_turn_yaw_mdeg = 0;
volatile int32_t g_turn_error_mdeg = 0;
volatile uint32_t g_turn_elapsed_ms = 0U;
volatile uint32_t g_turn_count = 0U;
volatile uint8_t g_single_turn_test_complete = 0U;
volatile uint32_t g_turn_read_pause_count = 0U;
volatile uint8_t g_turn_sensor_failure_count = 0U;
volatile uint8_t g_turn_center_enabled = 0U;
volatile uint8_t g_turn_center_complete = 0U;
volatile uint8_t g_turn_center_fault = ROUTE_PROGRESS_FAULT_NONE;
volatile uint32_t g_turn_center_target_edges =
    JUNCTION_CENTER_TARGET_EDGES;
volatile uint32_t g_turn_center_left_edges = 0U;
volatile uint32_t g_turn_center_right_edges = 0U;
volatile uint32_t g_turn_center_average_edges = 0U;
volatile uint8_t g_turn_center_event_latched = 0U;
volatile int32_t g_turn_center_event_left_count = 0;
volatile int32_t g_turn_center_event_right_count = 0;
volatile int32_t g_turn_center_heading_mdeg = 0;
volatile int32_t g_turn_center_heading_trim = 0;

volatile int32_t g_ward_bias_sum_raw = 0;
volatile uint32_t g_ward_bias_sample_count = 0U;
volatile uint32_t g_ward_bias_update_count = 0U;

static medicine_mission_state_t g_mission;
static route_event_state_t g_route_detector;
static route_progress_state_t g_route_progress;
static route_progress_state_t g_turn_center_progress;
static gyro_turn_state_t g_turn_controller;
static uint8_t g_synced_action_index = UINT8_MAX;
static medicine_action_kind_t g_synced_action_kind = MEDICINE_ACTION_COMPLETE;

static NOINLINE void delay_cycles_rough(uint32_t cycles)
{
    volatile uint32_t index;

    for (index = 0U; index < cycles; index++) {
        __asm volatile("nop");
    }
}

static NOINLINE void delay_us_rough(uint32_t microseconds)
{
    delay_cycles_rough(microseconds * DELAY_LOOPS_PER_US);
}

static NOINLINE void delay_ms_rough(uint32_t milliseconds)
{
    uint32_t index;

    for (index = 0U; index < milliseconds; index++) {
        delay_us_rough(1000U);
    }
}

void SysTick_Handler(void)
{
    g_system_ms++;
}

static NOINLINE void system_tick_init(void)
{
    SysTick->CTRL = 0U;
    SysTick->LOAD = (CPU_CLOCK_HZ / 1000U) - 1U;
    SysTick->VAL = 0U;
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk |
                    SysTick_CTRL_TICKINT_Msk |
                    SysTick_CTRL_ENABLE_Msk;
}

static NOINLINE void wait_for_mission_tick(uint32_t tick_start_ms)
{
    while ((uint32_t)(g_system_ms - tick_start_ms) < MISSION_TICK_MS) {
        __asm volatile("nop");
    }
}

static NOINLINE void init_output(
    uint32_t pincm,
    GPIO_Regs *port,
    uint32_t pin,
    bool high)
{
    DL_GPIO_initDigitalOutput(pincm);
    if (high) {
        DL_GPIO_setPins(port, pin);
    } else {
        DL_GPIO_clearPins(port, pin);
    }
    DL_GPIO_enableOutput(port, pin);
}

static NOINLINE void init_input(uint32_t pincm)
{
    DL_GPIO_initDigitalInputFeatures(
        pincm,
        DL_GPIO_INVERSION_DISABLE,
        DL_GPIO_RESISTOR_NONE,
        DL_GPIO_HYSTERESIS_ENABLE,
        DL_GPIO_WAKEUP_DISABLE);
}

static NOINLINE void encoder_update_levels(void)
{
    const uint32_t pins = DL_GPIO_readPins(
        ENCODER_PORT,
        ENCODER_LEFT_A_PIN | ENCODER_LEFT_B_PIN |
        ENCODER_RIGHT_A_PIN | ENCODER_RIGHT_B_PIN);

    g_encoder_left_a_level =
        ((pins & ENCODER_LEFT_A_PIN) != 0U) ? 1U : 0U;
    g_encoder_left_b_level =
        ((pins & ENCODER_LEFT_B_PIN) != 0U) ? 1U : 0U;
    g_encoder_right_a_level =
        ((pins & ENCODER_RIGHT_A_PIN) != 0U) ? 1U : 0U;
    g_encoder_right_b_level =
        ((pins & ENCODER_RIGHT_B_PIN) != 0U) ? 1U : 0U;
}

static NOINLINE void encoder_init(void)
{
    init_input(IOMUX_PINCM29);
    init_input(IOMUX_PINCM30);
    init_input(IOMUX_PINCM31);
    init_input(IOMUX_PINCM32);
    encoder_update_levels();

    DL_GPIO_setLowerPinsPolarity(
        ENCODER_PORT,
        DL_GPIO_PIN_12_EDGE_RISE | DL_GPIO_PIN_14_EDGE_RISE);
    DL_GPIO_clearInterruptStatus(
        ENCODER_PORT, ENCODER_INTERRUPT_MASK);
    DL_GPIO_enableInterrupt(ENCODER_PORT, ENCODER_INTERRUPT_MASK);
    NVIC_SetPriority(GPIOB_INT_IRQn, 3U);
    NVIC_EnableIRQ(GPIOB_INT_IRQn);
}

static NOINLINE void encoder_snapshot_counts(
    int32_t *left_count,
    int32_t *right_count)
{
    __disable_irq();
    *left_count = g_encoder_left_count;
    *right_count = g_encoder_right_count;
    __enable_irq();
}

static NOINLINE void led_set(bool on)
{
    if (on) {
        DL_GPIO_setPins(LED_PORT, LED_PIN);
        g_led_state = 1U;
    } else {
        DL_GPIO_clearPins(LED_PORT, LED_PIN);
        g_led_state = 0U;
    }
}

static NOINLINE void safe_outputs_init(void)
{
    init_output(IOMUX_PINCM12, MOTOR_PORT, AIN1_PIN, false);
    init_output(IOMUX_PINCM13, MOTOR_PORT, AIN2_PIN, false);
    init_output(IOMUX_PINCM15, MOTOR_PORT, BIN1_PIN, false);
    init_output(IOMUX_PINCM16, MOTOR_PORT, BIN2_PIN, false);
    init_output(IOMUX_PINCM17, MOTOR_PORT, STBY_PIN, false);

    init_output(IOMUX_PINCM34, PWM_PORT, PWMA_PIN, false);
    init_output(IOMUX_PINCM35, PWM_PORT, PWMB_PIN, false);
    init_output(IOMUX_PINCM36, SERVO_PORT, SERVO_X_PIN, false);
    init_output(IOMUX_PINCM39, SERVO_PORT, SERVO_Y_PIN, false);
    init_output(IOMUX_PINCM28, BUZZER_PORT, BUZZER_PIN, true);
    init_output(IOMUX_PINCM37, LED_PORT, LED_PIN, false);
}

static NOINLINE void pwm_init(void)
{
    DL_TimerG_ClockConfig clock_config = {
        .clockSel = DL_TIMER_CLOCK_BUSCLK,
        .divideRatio = DL_TIMER_CLOCK_DIVIDE_1,
        .prescale = 0U,
    };
    DL_TimerG_PWMConfig pwm_config = {
        .pwmMode = DL_TIMER_PWM_MODE_EDGE_ALIGN,
        .period = PWM_PERIOD,
        .isTimerWithFourCC = false,
        .startTimer = DL_TIMER_STOP,
    };

    DL_TimerG_reset(PWM_TIMER);
    DL_TimerG_enablePower(PWM_TIMER);
    delay_cycles(16U);

    DL_GPIO_initPeripheralOutputFunction(
        IOMUX_PINCM34, IOMUX_PINCM34_PF_TIMG0_CCP0);
    DL_GPIO_enableOutput(PWM_PORT, PWMA_PIN);
    DL_GPIO_initPeripheralOutputFunction(
        IOMUX_PINCM35, IOMUX_PINCM35_PF_TIMG0_CCP1);
    DL_GPIO_enableOutput(PWM_PORT, PWMB_PIN);

    DL_TimerG_setClockConfig(PWM_TIMER, &clock_config);
    DL_TimerG_initPWMMode(PWM_TIMER, &pwm_config);

    DL_TimerG_setCaptureCompareOutCtl(
        PWM_TIMER,
        DL_TIMER_CC_OCTL_INIT_VAL_LOW,
        DL_TIMER_CC_OCTL_INV_OUT_DISABLED,
        DL_TIMER_CC_OCTL_SRC_FUNCVAL,
        DL_TIMERG_CAPTURE_COMPARE_0_INDEX);
    DL_TimerG_setCaptureCompareOutCtl(
        PWM_TIMER,
        DL_TIMER_CC_OCTL_INIT_VAL_LOW,
        DL_TIMER_CC_OCTL_INV_OUT_DISABLED,
        DL_TIMER_CC_OCTL_SRC_FUNCVAL,
        DL_TIMERG_CAPTURE_COMPARE_1_INDEX);
    DL_TimerG_setCaptCompUpdateMethod(
        PWM_TIMER,
        DL_TIMER_CC_UPDATE_METHOD_IMMEDIATE,
        DL_TIMERG_CAPTURE_COMPARE_0_INDEX);
    DL_TimerG_setCaptCompUpdateMethod(
        PWM_TIMER,
        DL_TIMER_CC_UPDATE_METHOD_IMMEDIATE,
        DL_TIMERG_CAPTURE_COMPARE_1_INDEX);

    DL_TimerG_enableClock(PWM_TIMER);
    DL_TimerG_setCCPDirection(
        PWM_TIMER, DL_TIMER_CC0_OUTPUT | DL_TIMER_CC1_OUTPUT);
    DL_TimerG_startCounter(PWM_TIMER);
}

static NOINLINE void set_pwm(uint32_t left_ticks, uint32_t right_ticks)
{
    g_left_pwm_ticks = left_ticks;
    g_right_pwm_ticks = right_ticks;
    g_left_compare_ticks = PWM_PERIOD - left_ticks;
    g_right_compare_ticks = PWM_PERIOD - right_ticks;

    DL_TimerG_setCaptureCompareValue(
        PWM_TIMER, g_left_compare_ticks, DL_TIMER_CC_0_INDEX);
    DL_TimerG_setCaptureCompareValue(
        PWM_TIMER, g_right_compare_ticks, DL_TIMER_CC_1_INDEX);
}

static NOINLINE void motors_forward(void)
{
    DL_GPIO_clearPins(
        MOTOR_PORT, AIN1_PIN | AIN2_PIN | BIN1_PIN | BIN2_PIN);
    DL_GPIO_setPins(MOTOR_PORT, AIN1_PIN | BIN1_PIN | STBY_PIN);
}

static NOINLINE void motors_spin_left(void)
{
    DL_GPIO_clearPins(
        MOTOR_PORT, AIN1_PIN | AIN2_PIN | BIN1_PIN | BIN2_PIN);
    DL_GPIO_setPins(MOTOR_PORT, AIN2_PIN | BIN1_PIN | STBY_PIN);
}

static NOINLINE void motors_spin_right(void)
{
    DL_GPIO_clearPins(
        MOTOR_PORT, AIN1_PIN | AIN2_PIN | BIN1_PIN | BIN2_PIN);
    DL_GPIO_setPins(MOTOR_PORT, AIN1_PIN | BIN2_PIN | STBY_PIN);
}

static NOINLINE void motors_brake(void)
{
    set_pwm(PWM_PERIOD, PWM_PERIOD);
    DL_GPIO_setPins(
        MOTOR_PORT,
        AIN1_PIN | AIN2_PIN | BIN1_PIN | BIN2_PIN | STBY_PIN);
}

static NOINLINE void motors_stop(void)
{
    set_pwm(0U, 0U);
    DL_GPIO_clearPins(
        MOTOR_PORT,
        AIN1_PIN | AIN2_PIN | BIN1_PIN | BIN2_PIN | STBY_PIN);
}

static NOINLINE void i2c0_init_100khz(void)
{
    static const DL_I2C_ClockConfig clock_config = {
        .clockSel = DL_I2C_CLOCK_BUSCLK,
        .divideRatio = DL_I2C_CLOCK_DIVIDE_1,
    };

    DL_I2C_reset(MPU_I2C);
    DL_I2C_enablePower(MPU_I2C);
    delay_cycles(16U);
    DL_GPIO_initPeripheralInputFunctionFeatures(
        IOMUX_PINCM1,
        IOMUX_PINCM1_PF_I2C0_SDA,
        DL_GPIO_INVERSION_DISABLE,
        DL_GPIO_RESISTOR_NONE,
        DL_GPIO_HYSTERESIS_DISABLE,
        DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initPeripheralInputFunctionFeatures(
        IOMUX_PINCM2,
        IOMUX_PINCM2_PF_I2C0_SCL,
        DL_GPIO_INVERSION_DISABLE,
        DL_GPIO_RESISTOR_NONE,
        DL_GPIO_HYSTERESIS_DISABLE,
        DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_enableHiZ(IOMUX_PINCM1);
    DL_GPIO_enableHiZ(IOMUX_PINCM2);
    DL_I2C_setClockConfig(MPU_I2C, &clock_config);
    DL_I2C_disableAnalogGlitchFilter(MPU_I2C);
    DL_I2C_resetControllerTransfer(MPU_I2C);
    DL_I2C_setTimerPeriod(MPU_I2C, MPU_I2C_TIMER_PERIOD_100KHZ);
    DL_I2C_setControllerTXFIFOThreshold(
        MPU_I2C, DL_I2C_TX_FIFO_LEVEL_BYTES_1);
    DL_I2C_setControllerRXFIFOThreshold(
        MPU_I2C, DL_I2C_RX_FIFO_LEVEL_BYTES_1);
    DL_I2C_enableControllerClockStretching(MPU_I2C);
    DL_I2C_enableController(MPU_I2C);
}

static NOINLINE bool i2c_wait_idle(void)
{
    uint32_t timeout = MPU_I2C_TIMEOUT;

    while (timeout > 0U) {
        if ((DL_I2C_getControllerStatus(MPU_I2C) &
             DL_I2C_CONTROLLER_STATUS_IDLE) != 0U) {
            return true;
        }
        timeout--;
    }
    return false;
}

static NOINLINE bool i2c_wait_not_busy(void)
{
    uint32_t timeout = MPU_I2C_TIMEOUT;

    while (timeout > 0U) {
        if ((DL_I2C_getControllerStatus(MPU_I2C) &
             DL_I2C_CONTROLLER_STATUS_BUSY) == 0U) {
            return true;
        }
        timeout--;
    }
    return false;
}

static NOINLINE void i2c_recover_after_error(void)
{
    g_i2c_error_count++;
    DL_I2C_resetControllerTransfer(MPU_I2C);
    DL_I2C_flushControllerTXFIFO(MPU_I2C);
    DL_I2C_flushControllerRXFIFO(MPU_I2C);
}

static NOINLINE bool i2c_write_bytes(
    uint8_t address,
    const uint8_t *data,
    uint8_t length)
{
    if ((length == 0U) || (length > 8U) || !i2c_wait_idle()) {
        i2c_recover_after_error();
        return false;
    }
    DL_I2C_flushControllerTXFIFO(MPU_I2C);
    if (DL_I2C_fillControllerTXFIFO(MPU_I2C, data, length) != length) {
        i2c_recover_after_error();
        return false;
    }
    DL_I2C_startControllerTransfer(
        MPU_I2C,
        address,
        DL_I2C_CONTROLLER_DIRECTION_TX,
        length);
    delay_cycles(3U);
    if (!i2c_wait_not_busy() ||
        ((DL_I2C_getControllerStatus(MPU_I2C) &
          DL_I2C_CONTROLLER_STATUS_ERROR) != 0U)) {
        i2c_recover_after_error();
        return false;
    }
    return i2c_wait_idle();
}

static NOINLINE bool i2c_read_bytes(
    uint8_t address,
    uint8_t *data,
    uint8_t length)
{
    uint8_t index;

    if ((length == 0U) || (length > 8U) || !i2c_wait_idle()) {
        i2c_recover_after_error();
        return false;
    }
    DL_I2C_flushControllerRXFIFO(MPU_I2C);
    DL_I2C_startControllerTransfer(
        MPU_I2C,
        address,
        DL_I2C_CONTROLLER_DIRECTION_RX,
        length);
    delay_cycles(3U);
    for (index = 0U; index < length; index++) {
        uint32_t timeout = MPU_I2C_TIMEOUT;

        while (DL_I2C_isControllerRXFIFOEmpty(MPU_I2C) &&
               (timeout > 0U)) {
            timeout--;
        }
        if (timeout == 0U) {
            i2c_recover_after_error();
            return false;
        }
        data[index] = DL_I2C_receiveControllerData(MPU_I2C);
    }
    if (!i2c_wait_not_busy() ||
        ((DL_I2C_getControllerStatus(MPU_I2C) &
          DL_I2C_CONTROLLER_STATUS_ERROR) != 0U)) {
        i2c_recover_after_error();
        return false;
    }
    return true;
}

static NOINLINE bool mpu_write_register(uint8_t reg, uint8_t value)
{
    const uint8_t packet[2] = {reg, value};

    return i2c_write_bytes(MPU_I2C_ADDRESS, packet, 2U);
}

static NOINLINE bool mpu_read_registers(
    uint8_t first_reg,
    uint8_t *data,
    uint8_t length)
{
    if (!i2c_write_bytes(MPU_I2C_ADDRESS, &first_reg, 1U)) {
        return false;
    }
    delay_cycles(100U);
    return i2c_read_bytes(MPU_I2C_ADDRESS, data, length);
}

static NOINLINE bool mpu_read_gyro_raw(void)
{
    uint8_t data[6];

    if (!mpu_read_registers(MPU_REG_GYRO_XOUT_H, data, 6U)) {
        return false;
    }
    g_gyro_x_raw = (int16_t)(((uint16_t)data[0] << 8) | data[1]);
    g_gyro_y_raw = (int16_t)(((uint16_t)data[2] << 8) | data[3]);
    g_gyro_z_raw = (int16_t)(((uint16_t)data[4] << 8) | data[5]);
    return true;
}

static NOINLINE bool mpu_init(void)
{
    uint8_t who_am_i = 0U;

    if (!mpu_read_registers(MPU_REG_WHO_AM_I, &who_am_i, 1U)) {
        g_mpu_error = 1U;
        return false;
    }
    g_mpu_who_am_i = who_am_i;
    if (who_am_i == MPU6050_WHO_AM_I) {
        g_mpu_device_type = 1U;
    } else if (who_am_i == MPU6500_WHO_AM_I) {
        g_mpu_device_type = 2U;
    } else {
        g_mpu_error = 2U;
        return false;
    }

    g_mpu_state = 1U;
    if (!mpu_write_register(MPU_REG_PWR_MGMT_1, 0x80U)) {
        g_mpu_error = 3U;
        return false;
    }
    delay_ms_rough(100U);
    if (!mpu_write_register(MPU_REG_PWR_MGMT_1, 0x01U) ||
        !mpu_write_register(MPU_REG_SMPLRT_DIV, 0x04U) ||
        !mpu_write_register(MPU_REG_CONFIG, 0x03U) ||
        !mpu_write_register(MPU_REG_GYRO_CONFIG, MPU_GYRO_FS_500DPS)) {
        g_mpu_error = 3U;
        return false;
    }
    delay_ms_rough(50U);
    g_mpu_link_ok = 1U;
    g_mpu_error = 0U;
    return true;
}

static NOINLINE bool mpu_calibrate_gyro_z(void)
{
    int64_t sum = 0;
    uint32_t index;

    g_mpu_state = 2U;
    g_gyro_z_bias_raw = 0;
    g_calibration_sample_count = 0U;
    for (index = 0U; index < MPU_CALIBRATION_SAMPLES; index++) {
        if (!mpu_read_gyro_raw()) {
            g_mpu_error = 4U;
            return false;
        }
        sum += g_gyro_z_raw;
        g_calibration_sample_count = index + 1U;
        led_set(((index / 50U) & 0x01U) != 0U);
        delay_ms_rough(4U);
    }
    g_gyro_z_bias_raw = (int32_t)(sum / MPU_CALIBRATION_SAMPLES);
    g_mpu_state = 3U;
    g_mpu_error = 0U;
    return true;
}

static NOINLINE bool mpu_sample_update(void)
{
    if (!mpu_read_gyro_raw()) {
        g_mpu_read_fail_count++;
        g_mpu_read_fail_total++;
        g_mpu_error = 4U;
        if (g_mpu_read_fail_count >= MPU_MAX_CONSECUTIVE_FAILURES) {
            g_mpu_link_ok = 0U;
        }
        return false;
    }

    g_mpu_read_fail_count = 0U;
    g_mpu_sample_count++;
    g_mpu_error = 0U;
    g_gyro_z_corrected_raw =
        (int32_t)g_gyro_z_raw - g_gyro_z_bias_raw;
    g_gyro_z_mdps = (int32_t)(
        ((int64_t)g_gyro_z_corrected_raw * 2000) / 131);
    g_gyro_z_left_positive_mdps =
        g_gyro_z_mdps * MPU_LEFT_POSITIVE_SIGN;
    return true;
}

static NOINLINE void gray_sensor_init(void)
{
    init_output(IOMUX_PINCM18, GRAY_ADDRESS_PORT, GRAY_AD0_PIN, false);
    init_output(IOMUX_PINCM23, GRAY_ADDRESS_PORT, GRAY_AD1_PIN, false);
    init_output(IOMUX_PINCM24, GRAY_ADDRESS_PORT, GRAY_AD2_PIN, false);
    init_input(IOMUX_PINCM14);
}

static NOINLINE void gray_select_channel(uint8_t channel)
{
    uint32_t set_mask = 0U;

    DL_GPIO_clearPins(GRAY_ADDRESS_PORT, GRAY_ADDRESS_MASK);
    if ((channel & 0x01U) != 0U) {
        set_mask |= GRAY_AD0_PIN;
    }
    if ((channel & 0x02U) != 0U) {
        set_mask |= GRAY_AD1_PIN;
    }
    if ((channel & 0x04U) != 0U) {
        set_mask |= GRAY_AD2_PIN;
    }
    if (set_mask != 0U) {
        DL_GPIO_setPins(GRAY_ADDRESS_PORT, set_mask);
    }

    g_gray_selected_channel = (uint8_t) (channel + 1U);
    delay_us_rough(GRAY_ADDRESS_SETTLE_US);
}

static NOINLINE uint8_t gray_read_selected_channel(void)
{
    uint8_t sample;
    uint8_t high_count = 0U;

    for (sample = 0U; sample < GRAY_SAMPLE_COUNT; sample++) {
        const uint8_t level =
            ((DL_GPIO_readPins(GRAY_OUT_PORT, GRAY_OUT_PIN) &
              GRAY_OUT_PIN) != 0U)
            ? 1U
            : 0U;

        g_gray_live_out = level;
        high_count = (uint8_t) (high_count + level);
        delay_us_rough(GRAY_SAMPLE_INTERVAL_US);
    }
    return high_count;
}

static NOINLINE void gray_publish_channel_values(void)
{
    g_gray_x1 = ((g_gray_raw_high_mask & 0x01U) != 0U) ? 1U : 0U;
    g_gray_x2 = ((g_gray_raw_high_mask & 0x02U) != 0U) ? 1U : 0U;
    g_gray_x3 = ((g_gray_raw_high_mask & 0x04U) != 0U) ? 1U : 0U;
    g_gray_x4 = ((g_gray_raw_high_mask & 0x08U) != 0U) ? 1U : 0U;
    g_gray_x5 = ((g_gray_raw_high_mask & 0x10U) != 0U) ? 1U : 0U;
    g_gray_x6 = ((g_gray_raw_high_mask & 0x20U) != 0U) ? 1U : 0U;
    g_gray_x7 = ((g_gray_raw_high_mask & 0x40U) != 0U) ? 1U : 0U;
    g_gray_x8 = ((g_gray_raw_high_mask & 0x80U) != 0U) ? 1U : 0U;

    g_gray_x1_high_count = g_gray_channel_high_count[0];
    g_gray_x2_high_count = g_gray_channel_high_count[1];
    g_gray_x3_high_count = g_gray_channel_high_count[2];
    g_gray_x4_high_count = g_gray_channel_high_count[3];
    g_gray_x5_high_count = g_gray_channel_high_count[4];
    g_gray_x6_high_count = g_gray_channel_high_count[5];
    g_gray_x7_high_count = g_gray_channel_high_count[6];
    g_gray_x8_high_count = g_gray_channel_high_count[7];
}

static NOINLINE void gray_publish_line_values(uint8_t line_mask)
{
    g_gray_line_x1 = ((line_mask & 0x01U) != 0U) ? 1U : 0U;
    g_gray_line_x2 = ((line_mask & 0x02U) != 0U) ? 1U : 0U;
    g_gray_line_x3 = ((line_mask & 0x04U) != 0U) ? 1U : 0U;
    g_gray_line_x4 = ((line_mask & 0x08U) != 0U) ? 1U : 0U;
    g_gray_line_x5 = ((line_mask & 0x10U) != 0U) ? 1U : 0U;
    g_gray_line_x6 = ((line_mask & 0x20U) != 0U) ? 1U : 0U;
    g_gray_line_x7 = ((line_mask & 0x40U) != 0U) ? 1U : 0U;
    g_gray_line_x8 = ((line_mask & 0x80U) != 0U) ? 1U : 0U;
}

static NOINLINE void gray_scan_all_channels(void)
{
    uint8_t channel;
    uint8_t high_mask = 0U;
    uint8_t unstable_mask = 0U;

    for (channel = 0U; channel < GRAY_CHANNEL_COUNT; channel++) {
        uint8_t high_count;

        gray_select_channel(channel);
        high_count = gray_read_selected_channel();
        g_gray_channel_high_count[channel] = high_count;

        if (high_count >= GRAY_HIGH_MAJORITY) {
            high_mask |= (uint8_t) (1U << channel);
        }
        if ((high_count != 0U) && (high_count != GRAY_SAMPLE_COUNT)) {
            unstable_mask |= (uint8_t) (1U << channel);
        }
    }

    g_gray_raw_high_mask = high_mask;
    g_gray_unstable_mask = unstable_mask;
    gray_publish_channel_values();
    g_gray_scan_count++;
}

static NOINLINE void gray_update_line_position(uint8_t changed_mask)
{
    uint8_t active_count = 0U;
    const int16_t line_error = simple_line_error_from_mask(
        changed_mask,
        &active_count);

    g_gray_changed_count = active_count;
    g_gray_line_abnormal =
        (active_count > LINE_MAX_ACTIVE_SENSORS) ? 1U : 0U;

    if ((active_count == 0U) ||
        (active_count > LINE_MAX_ACTIVE_SENSORS)) {
        g_gray_line_valid = 0U;
        g_gray_line_mask = 0U;
        g_gray_line_error = g_line_last_error;
        return;
    }

    g_gray_line_valid = 1U;
    g_gray_line_mask = changed_mask;
    g_gray_line_error = line_error;
    g_line_last_error = g_gray_line_error;
    if (g_gray_line_error != 0) {
        g_line_last_nonzero_error = g_gray_line_error;
    }
}

static NOINLINE void gray_use_fixed_white_baseline(void)
{
    /*
     * The official start places the car at the pharmacy entrance with the
     * front sensor already over the red center line. This board has been
     * verified as white=1 and red=0, so learning the background at boot
     * would incorrectly learn the starting line as part of the baseline.
     */
    g_gray_baseline_scan_count = 0U;
    g_gray_white_baseline_mask = GRAY_FIXED_WHITE_BASELINE_MASK;
    g_gray_baseline_uniform = 1U;
    g_gray_white_level_code = 1U;
    g_gray_baseline_ready = 1U;
    g_gray_test_state = 2U;
}

static NOINLINE void gray_update_baseline_and_line(void)
{
    uint8_t channel;

    if (g_gray_baseline_ready == 0U) {
        for (channel = 0U; channel < GRAY_CHANNEL_COUNT; channel++) {
            if ((g_gray_raw_high_mask & (uint8_t) (1U << channel)) != 0U) {
                g_gray_baseline_votes[channel]++;
            }
        }

        g_gray_baseline_scan_count++;
        if (g_gray_baseline_scan_count >= GRAY_BASELINE_SCAN_COUNT) {
            uint8_t baseline_mask = 0U;

            for (channel = 0U; channel < GRAY_CHANNEL_COUNT; channel++) {
                if (g_gray_baseline_votes[channel] >=
                    ((GRAY_BASELINE_SCAN_COUNT + 1U) / 2U)) {
                    baseline_mask |= (uint8_t) (1U << channel);
                }
            }

            g_gray_white_baseline_mask = baseline_mask;
            g_gray_baseline_uniform =
                ((baseline_mask == 0x00U) || (baseline_mask == 0xFFU))
                ? 1U
                : 0U;
            g_gray_white_level_code =
                (baseline_mask == 0x00U) ? 0U :
                (baseline_mask == 0xFFU) ? 1U : 2U;
            g_gray_baseline_ready = 1U;
            g_gray_test_state =
                (g_gray_baseline_uniform != 0U) ? 2U : 3U;
        }
        return;
    }

    g_gray_changed_from_white_mask = simple_line_normalize_mask(
        g_gray_raw_high_mask,
        g_gray_white_baseline_mask);
    gray_publish_line_values(g_gray_changed_from_white_mask);
    if (g_gray_changed_from_white_mask != g_gray_previous_change_mask) {
        g_gray_change_event_count++;
        g_gray_previous_change_mask = g_gray_changed_from_white_mask;
    }
    gray_update_line_position(g_gray_changed_from_white_mask);
}

static NOINLINE bool simple_line_follow_update(void)
{
    const simple_line_output_t output = simple_line_step(
        &LINE_CONTROL_CONFIG,
        g_gray_line_valid != 0U,
        LINE_SENSOR_ERROR_SIGN * (int32_t) g_gray_line_error,
        LINE_SENSOR_ERROR_SIGN * (int32_t) g_line_last_nonzero_error,
        g_line_lost_count);

    g_line_correction = output.correction;
    g_line_lost_count = output.lost_count;

    if (g_motor_disabled != 0U) {
        g_phase = PHASE_STOP;
        g_stop_reason = STOP_REASON_MANUAL;
        motors_stop();
        return false;
    }
    if (output.stop) {
        g_phase = PHASE_STOP;
        g_stop_reason = STOP_REASON_LINE_LOST;
        motors_stop();
        return false;
    }

    set_pwm(output.left_pwm, output.right_pwm);
    return true;
}

static NOINLINE void gray_update_led(void)
{
    if (g_phase == PHASE_CALIBRATION) {
        led_set(((g_gray_scan_count / 10U) & 0x01U) != 0U);
    } else if (g_phase == PHASE_MISSION_FOLLOW) {
        if (g_gray_line_valid != 0U) {
            led_set(true);
        } else {
            led_set(((g_run_frame / 5U) & 0x01U) != 0U);
        }
    } else {
        led_set(false);
    }
}

static NOINLINE void fatal_error_loop(uint8_t stop_reason)
{
    g_phase = PHASE_STOP;
    g_stop_reason = stop_reason;
    motors_stop();

    while (1) {
        led_set(true);
        delay_ms_rough(100U);
        led_set(false);
        delay_ms_rough(100U);
    }
}

static NOINLINE void medicine_publish_state(void)
{
    const medicine_route_action_t *action =
        medicine_mission_current_action(&g_mission);

    g_route_action_index = g_mission.action_index;
    g_visited_ward_mask = g_mission.visited_ward_mask;
    g_ward_stop_count = g_mission.ward_stop_count;
    g_route_action_elapsed_ms = g_mission.action_elapsed_ms;
    g_route_complete = g_mission.completed ? 1U : 0U;
    g_route_fault = g_mission.fault ? 1U : 0U;
    if (action != NULL) {
        g_route_action_kind = (uint8_t)action->kind;
        g_route_action_ward = action->ward;
    }
}

static NOINLINE void route_publish_detector(void)
{
    g_route_event_armed = g_route_detector.armed ? 1U : 0U;
    g_route_travel_frames = g_route_detector.travel_frames;
    g_route_normal_frames = g_route_detector.normal_frames;
    g_route_wide_frames = g_route_detector.wide_frames;
    g_route_lost_frames = g_route_detector.lost_frames;
    g_route_centered_frames = g_route_detector.centered_frames;
    g_route_endpoint_loss_active =
        g_route_detector.endpoint_loss_active ? 1U : 0U;
    g_route_endpoint_entry_centered =
        g_route_detector.endpoint_entry_centered ? 1U : 0U;
    g_route_endpoint_entry_distance_ready =
        g_route_detector.endpoint_entry_distance_ready ? 1U : 0U;
}

static NOINLINE uint32_t route_centimeters_to_encoder_edges(
    uint16_t centimeters)
{
    return (uint32_t)centimeters * ENCODER_RISING_EDGES_PER_CM;
}

static NOINLINE void route_publish_progress(void)
{
    g_route_left_start_count = g_route_progress.left_start_count;
    g_route_right_start_count = g_route_progress.right_start_count;
    g_route_left_forward_edges = g_route_progress.left_forward_edges;
    g_route_right_forward_edges = g_route_progress.right_forward_edges;
    g_route_average_forward_edges =
        g_route_progress.average_forward_edges;
    g_route_encoder_no_motion_frames =
        g_route_progress.no_motion_frames;
    g_route_left_no_motion_frames =
        g_route_progress.left_no_motion_frames;
    g_route_right_no_motion_frames =
        g_route_progress.right_no_motion_frames;
    g_route_endpoint_min_edges = g_route_progress.endpoint_min_edges;
    g_route_endpoint_max_edges = g_route_progress.endpoint_max_edges;
    g_route_endpoint_distance_ready =
        route_progress_endpoint_ready(&g_route_progress) ? 1U : 0U;
    g_route_progress_fault = (uint8_t)g_route_progress.fault;
}

static NOINLINE void route_progress_begin_action(
    const medicine_route_action_t *action)
{
    int32_t left_count;
    int32_t right_count;
    const uint32_t endpoint_min_edges =
        route_centimeters_to_encoder_edges(
            medicine_route_endpoint_minimum_cm(action));
    const uint32_t endpoint_max_edges =
        route_centimeters_to_encoder_edges(
            medicine_route_endpoint_maximum_cm(action));

    encoder_snapshot_counts(&left_count, &right_count);
    route_progress_begin(
        &g_route_progress,
        left_count,
        right_count,
        endpoint_min_edges,
        endpoint_max_edges);
    route_publish_progress();
}

static NOINLINE route_progress_fault_t route_progress_update_action(void)
{
    int32_t left_count;
    int32_t right_count;
    route_progress_fault_t fault;

    encoder_snapshot_counts(&left_count, &right_count);
    fault = route_progress_step(
        &ROUTE_PROGRESS_CONFIG,
        &g_route_progress,
        left_count,
        right_count);
    route_publish_progress();
    return fault;
}

static NOINLINE void ward_bias_begin(void)
{
    g_ward_bias_sum_raw = 0;
    g_ward_bias_sample_count = 0U;
}

static NOINLINE void ward_bias_collect(void)
{
    if ((g_mission.action_elapsed_ms >= WARD_BIAS_SETTLE_MS) &&
        (gyro_turn_abs_i32(g_gyro_z_mdps) <=
         MPU_GYRO_STATIONARY_LIMIT_MDPS)) {
        g_ward_bias_sum_raw += g_gyro_z_raw;
        g_ward_bias_sample_count++;
    }
}

static NOINLINE void ward_bias_finish(void)
{
    if (g_ward_bias_sample_count >= WARD_BIAS_MIN_SAMPLES) {
        g_gyro_z_bias_raw =
            g_ward_bias_sum_raw / (int32_t)g_ward_bias_sample_count;
        g_ward_bias_update_count++;
    }
}

static NOINLINE void medicine_fail(uint8_t stop_reason)
{
    const medicine_mission_input_t input = {
        .hardware_fault = true,
    };

    medicine_mission_step(&g_mission, &input);
    g_phase = PHASE_STOP;
    g_stop_reason = stop_reason;
    motors_stop();
    medicine_publish_state();
}

static NOINLINE void turn_center_latch_junction_event(void)
{
    int32_t left_count;
    int32_t right_count;

    encoder_snapshot_counts(&left_count, &right_count);
    g_turn_center_event_left_count = left_count;
    g_turn_center_event_right_count = right_count;
    g_turn_center_event_latched = 1U;
}

static NOINLINE bool medicine_action_needs_turn_centering(
    medicine_action_kind_t kind)
{
    return (kind == MEDICINE_ACTION_TURN_LEFT_90) ||
           (kind == MEDICINE_ACTION_TURN_RIGHT_90);
}

static NOINLINE void turn_center_publish_progress(void)
{
    g_turn_center_left_edges =
        g_turn_center_progress.left_forward_edges;
    g_turn_center_right_edges =
        g_turn_center_progress.right_forward_edges;
    g_turn_center_average_edges =
        g_turn_center_progress.average_forward_edges;
    g_turn_center_fault = (uint8_t)g_turn_center_progress.fault;
}

static NOINLINE void turn_center_disable(void)
{
    g_turn_center_progress = (route_progress_state_t){0};
    g_turn_center_enabled = 0U;
    g_turn_center_complete = 0U;
    g_turn_center_fault = ROUTE_PROGRESS_FAULT_NONE;
    g_turn_center_target_edges = JUNCTION_CENTER_TARGET_EDGES;
    g_turn_center_left_edges = 0U;
    g_turn_center_right_edges = 0U;
    g_turn_center_average_edges = 0U;
    g_turn_center_event_latched = 0U;
    g_turn_center_event_left_count = 0;
    g_turn_center_event_right_count = 0;
    g_turn_center_heading_mdeg = 0;
    g_turn_center_heading_trim = 0;
}

static NOINLINE void turn_center_begin(void)
{
    int32_t left_count;
    int32_t right_count;

    if (g_turn_center_event_latched != 0U) {
        left_count = g_turn_center_event_left_count;
        right_count = g_turn_center_event_right_count;
    } else {
        encoder_snapshot_counts(&left_count, &right_count);
    }
    g_turn_center_event_latched = 0U;
    route_progress_begin(
        &g_turn_center_progress,
        left_count,
        right_count,
        JUNCTION_CENTER_TARGET_EDGES,
        JUNCTION_CENTER_MAX_EDGES);
    g_turn_center_enabled = 1U;
    g_turn_center_complete = 0U;
    g_turn_center_target_edges = JUNCTION_CENTER_TARGET_EDGES;
    g_turn_center_heading_mdeg = 0;
    g_turn_center_heading_trim = 0;
    turn_center_publish_progress();
}

static NOINLINE bool turn_center_step(void)
{
    int32_t left_count;
    int32_t right_count;
    route_progress_fault_t fault;

    encoder_snapshot_counts(&left_count, &right_count);
    fault = route_progress_step(
        &ROUTE_PROGRESS_CONFIG,
        &g_turn_center_progress,
        left_count,
        right_count);
    turn_center_publish_progress();

    if (fault != ROUTE_PROGRESS_FAULT_NONE) {
        medicine_fail(STOP_REASON_ENCODER_FAULT);
        return false;
    }
    if (g_turn_center_average_edges >= JUNCTION_CENTER_TARGET_EDGES) {
        g_turn_center_enabled = 0U;
        g_turn_center_complete = 1U;
        g_turn_center_heading_mdeg = g_turn_yaw_mdeg;
        g_turn_controller.phase = GYRO_TURN_PHASE_SETTLE;
        g_turn_controller.phase_elapsed_ms = 0U;
        motors_stop();
    }
    return true;
}

static NOINLINE int32_t turn_center_heading_trim(void)
{
    const int64_t encoder_error =
        (int64_t)g_turn_center_right_edges -
        (int64_t)g_turn_center_left_edges;
    int32_t trim =
        g_turn_yaw_mdeg / JUNCTION_CENTER_HEADING_TRIM_DIVISOR;

    trim += (int32_t)(
        encoder_error / JUNCTION_CENTER_ENCODER_TRIM_DIVISOR);
    if (trim > JUNCTION_CENTER_HEADING_TRIM_MAX) {
        trim = JUNCTION_CENTER_HEADING_TRIM_MAX;
    } else if (trim < -JUNCTION_CENTER_HEADING_TRIM_MAX) {
        trim = -JUNCTION_CENTER_HEADING_TRIM_MAX;
    }
    return trim;
}

static NOINLINE void apply_turn_center_forward(void)
{
    const int32_t trim = turn_center_heading_trim();
    const int32_t left_pwm =
        (int32_t)JUNCTION_CENTER_LEFT_PWM + trim;
    const int32_t right_pwm =
        (int32_t)JUNCTION_CENTER_RIGHT_PWM - trim;

    g_turn_center_heading_mdeg = g_turn_yaw_mdeg;
    g_turn_center_heading_trim = trim;
    set_pwm((uint32_t)left_pwm, (uint32_t)right_pwm);
    motors_forward();
}

static NOINLINE void medicine_sync_action(void)
{
    const medicine_route_action_t *action;

    if (g_synced_action_index == g_mission.action_index) {
        return;
    }
    if (g_synced_action_kind == MEDICINE_ACTION_WARD_STOP_2S) {
        ward_bias_finish();
    }

    action = medicine_mission_current_action(&g_mission);
    if (action == NULL) {
        medicine_fail(STOP_REASON_MISSION_FAULT);
        return;
    }

    g_synced_action_index = g_mission.action_index;
    g_synced_action_kind = action->kind;
    g_route_event = ROUTE_EVENT_NONE;

    if (medicine_route_is_follow_action(action->kind)) {
        route_event_init(&g_route_detector);
        route_publish_detector();
        route_progress_begin_action(action);
        g_turn_center_event_latched = 0U;
        g_line_lost_count = 0U;
        g_line_last_error = 0;
        g_line_last_nonzero_error = 0;
        g_line_correction = 0;
        g_phase = PHASE_MISSION_FOLLOW;
        set_pwm(PWM_LEFT_BASE_TICKS, PWM_RIGHT_BASE_TICKS);
        motors_forward();
    } else if (medicine_route_is_turn_action(action->kind)) {
        const int32_t target_mdeg =
            medicine_route_turn_target_mdeg(action->kind);
        const bool needs_turn_centering =
            medicine_action_needs_turn_centering(action->kind);
        const uint16_t center_forward_ms =
            needs_turn_centering
            ? UINT16_MAX
            : 0U;
        const uint32_t timeout_ms =
            (action->kind == MEDICINE_ACTION_TURN_AROUND_180)
            ? TURN_180_TIMEOUT_MS
            : TURN_90_TIMEOUT_MS;

        gyro_turn_init(
            &g_turn_controller,
            target_mdeg,
            center_forward_ms,
            timeout_ms);
        if (needs_turn_centering) {
            turn_center_begin();
        } else {
            turn_center_disable();
        }
        g_turn_target_mdeg = target_mdeg;
        g_turn_yaw_mdeg = 0;
        g_turn_error_mdeg = target_mdeg;
        g_turn_phase = (uint8_t)g_turn_controller.phase;
        g_turn_motion = GYRO_TURN_MOTION_STOP;
        g_turn_elapsed_ms = 0U;
        g_turn_sensor_failure_count = 0U;
        g_turn_count++;
        g_phase = PHASE_GYRO_TURN;
        motors_stop();
    } else if (action->kind == MEDICINE_ACTION_WARD_STOP_2S) {
        ward_bias_begin();
        g_phase = PHASE_WARD_STOP;
        motors_stop();
    } else if (action->kind == MEDICINE_ACTION_COMPLETE) {
        g_phase = PHASE_COMPLETE;
        motors_stop();
    } else {
        medicine_fail(STOP_REASON_MISSION_FAULT);
    }
    medicine_publish_state();
}

static NOINLINE bool medicine_follow_step(
    const medicine_route_action_t *action)
{
    const route_destination_t destination =
        medicine_route_destination(action->kind);
    const route_event_config_t *event_config =
        (g_route_action_index == 0U)
        ? &FIRST_JUNCTION_ROUTE_EVENT_CONFIG
        : &ROUTE_EVENT_CONFIG;
    route_event_config_t detector_config = *event_config;
    const route_progress_fault_t progress_fault =
        route_progress_update_action();
    const bool line_centered =
        (g_gray_line_valid != 0U) &&
        (g_gray_line_error >= -ENDPOINT_CENTER_ERROR_MAX) &&
        (g_gray_line_error <= ENDPOINT_CENTER_ERROR_MAX);
    route_event_t event;

    if (progress_fault != ROUTE_PROGRESS_FAULT_NONE) {
        const uint8_t stop_reason =
            (progress_fault == ROUTE_PROGRESS_FAULT_ENDPOINT_OVERRUN)
            ? STOP_REASON_ENDPOINT_MISSED
            : STOP_REASON_ENCODER_FAULT;

        medicine_fail(stop_reason);
        return false;
    }

    if ((destination != ROUTE_DESTINATION_JUNCTION) ||
        !g_route_detector.armed) {
        detector_config.wide_active_min = LINE_WIDE_ACTIVE_MIN;
    }

    event = route_event_step(
        &detector_config,
        &g_route_detector,
        destination,
        g_gray_changed_count,
        line_centered,
        g_route_endpoint_distance_ready != 0U);

    route_publish_detector();
    if (event != ROUTE_EVENT_NONE) {
        const medicine_mission_input_t input = {
            .destination_reached = true,
        };

        g_route_event = (uint8_t)event;
        if (event == ROUTE_EVENT_JUNCTION) {
            turn_center_latch_junction_event();
            g_junction_event_count++;
            motors_brake();
        } else {
            g_turn_center_event_latched = 0U;
            g_endpoint_event_count++;
            motors_stop();
        }
        medicine_mission_step(&g_mission, &input);
        medicine_publish_state();
        return !g_mission.fault;
    }

    if (g_gray_changed_count >= LINE_WIDE_ACTIVE_MIN) {
        g_line_correction = 0;
        g_line_lost_count = 0U;
        set_pwm(PWM_LEFT_BASE_TICKS, PWM_RIGHT_BASE_TICKS);
        motors_forward();
        return true;
    }

    if ((destination == ROUTE_DESTINATION_ENDPOINT) &&
        g_route_detector.armed &&
        (g_route_detector.lost_frames > 0U)) {
        g_line_correction = 0;
        g_line_lost_count = 0U;
        set_pwm(ENDPOINT_CONFIRM_LEFT_PWM, ENDPOINT_CONFIRM_RIGHT_PWM);
        motors_forward();
        return true;
    }

    if (!simple_line_follow_update()) {
        return false;
    }
    motors_forward();
    return true;
}

static NOINLINE void apply_turn_motion(gyro_turn_motion_t motion)
{
    switch (motion) {
        case GYRO_TURN_MOTION_FORWARD:
            set_pwm(JUNCTION_CENTER_LEFT_PWM, JUNCTION_CENTER_RIGHT_PWM);
            motors_forward();
            break;
        case GYRO_TURN_MOTION_SPIN_LEFT_FAST:
            set_pwm(TURN_FAST_PWM, TURN_FAST_PWM);
            motors_spin_left();
            break;
        case GYRO_TURN_MOTION_SPIN_LEFT_SLOW:
            set_pwm(TURN_SLOW_PWM, TURN_SLOW_PWM);
            motors_spin_left();
            break;
        case GYRO_TURN_MOTION_SPIN_RIGHT_FAST:
            set_pwm(TURN_FAST_PWM, TURN_FAST_PWM);
            motors_spin_right();
            break;
        case GYRO_TURN_MOTION_SPIN_RIGHT_SLOW:
            set_pwm(TURN_SLOW_PWM, TURN_SLOW_PWM);
            motors_spin_right();
            break;
        case GYRO_TURN_MOTION_BRAKE:
            motors_brake();
            break;
        case GYRO_TURN_MOTION_STOP:
        default:
            motors_stop();
            break;
    }
}

static NOINLINE bool medicine_turn_step(bool mpu_sample_ok)
{
    gyro_turn_output_t output;

    if (!mpu_sample_ok) {
        g_turn_read_pause_count++;
        output = gyro_turn_sensor_failure(
            &GYRO_TURN_CONFIG,
            &g_turn_controller,
            MISSION_TICK_MS);
        g_turn_phase = (uint8_t)g_turn_controller.phase;
        g_turn_motion = (uint8_t)output.motion;
        g_turn_elapsed_ms = g_turn_controller.total_elapsed_ms;
        g_turn_sensor_failure_count =
            g_turn_controller.sensor_failure_count;
        apply_turn_motion(output.motion);
        if (output.fault) {
            medicine_fail(STOP_REASON_MPU_READ);
            return false;
        }
        return true;
    }

    if ((g_turn_controller.phase == GYRO_TURN_PHASE_CENTER) ||
        (g_turn_controller.phase == GYRO_TURN_PHASE_ROTATE)) {
        g_turn_yaw_mdeg += (int32_t)(
            ((int64_t)g_gyro_z_left_positive_mdps * MISSION_TICK_MS) /
            1000);
    }
    if ((g_turn_center_enabled != 0U) &&
        (g_turn_controller.phase == GYRO_TURN_PHASE_CENTER)) {
        if (!turn_center_step()) {
            return false;
        }
    }
    output = gyro_turn_step(
        &GYRO_TURN_CONFIG,
        &g_turn_controller,
        g_turn_yaw_mdeg,
        g_gyro_z_left_positive_mdps,
        MISSION_TICK_MS);
    if (output.reset_yaw) {
        g_turn_yaw_mdeg = 0;
    }

    g_turn_phase = (uint8_t)g_turn_controller.phase;
    g_turn_motion = (uint8_t)output.motion;
    g_turn_target_mdeg = g_turn_controller.target_mdeg;
    g_turn_error_mdeg = g_turn_target_mdeg - g_turn_yaw_mdeg;
    g_turn_elapsed_ms = g_turn_controller.total_elapsed_ms;
    g_turn_sensor_failure_count =
        g_turn_controller.sensor_failure_count;
    if (output.motion == GYRO_TURN_MOTION_FORWARD) {
        apply_turn_center_forward();
    } else {
        apply_turn_motion(output.motion);
    }

    if (output.fault) {
        medicine_fail(STOP_REASON_TURN_TIMEOUT);
        return false;
    }
    if (output.complete) {
        const medicine_mission_input_t input = {
            .turn_complete = true,
        };

        motors_stop();
        medicine_mission_step(&g_mission, &input);
        if ((SINGLE_TURN_TEST_ENABLE != 0U) &&
            (g_turn_count == 1U) &&
            !g_mission.fault) {
            g_single_turn_test_complete = 1U;
        }
        medicine_publish_state();
    }
    return !g_mission.fault;
}

static NOINLINE bool medicine_ward_stop_step(bool mpu_sample_ok)
{
    const medicine_mission_input_t input = {
        .delta_ms = MISSION_TICK_MS,
    };

    motors_stop();
    if (!mpu_sample_ok &&
        (g_mpu_read_fail_count >= MPU_MAX_CONSECUTIVE_FAILURES)) {
        medicine_fail(STOP_REASON_MPU_READ);
        return false;
    }
    if (mpu_sample_ok) {
        ward_bias_collect();
    }
    medicine_mission_step(&g_mission, &input);
    medicine_publish_state();
    return !g_mission.fault;
}

int main(void)
{
    const medicine_mission_input_t no_event = {0};

    SYSCFG_DL_init();
    system_tick_init();
    g_firmware_version = FIRMWARE_VERSION;

    safe_outputs_init();
    encoder_init();
    gray_sensor_init();
    pwm_init();
    i2c0_init_100khz();
    motors_stop();

    g_gray_test_state = 0U;
    g_phase = PHASE_INIT;
    led_set(true);
    delay_ms_rough(100U);
    led_set(false);
    delay_ms_rough(GRAY_POWERUP_DELAY_MS - 100U);

    if (!mpu_init()) {
        fatal_error_loop(STOP_REASON_MPU_INIT);
    }
    if (!mpu_calibrate_gyro_z()) {
        fatal_error_loop(STOP_REASON_MPU_CALIBRATION);
    }

    g_gray_test_state = 1U;
    g_phase = PHASE_CALIBRATION;
    gray_use_fixed_white_baseline();

    led_set(true);
    delay_ms_rough(START_READY_HOLD_MS);
    led_set(false);

    medicine_mission_init(&g_mission);
    g_synced_action_index = UINT8_MAX;
    g_synced_action_kind = MEDICINE_ACTION_COMPLETE;
    g_mission_elapsed_ms = 0U;
    g_run_frame = 0U;
    g_stop_reason = STOP_REASON_NONE;
    g_single_turn_test_complete = 0U;
    medicine_publish_state();

    while (!g_mission.completed &&
           !g_mission.fault &&
           (g_single_turn_test_complete == 0U)) {
        const medicine_route_action_t *action;
        const uint32_t tick_start_ms = g_system_ms;
        bool mpu_sample_ok;
        bool step_ok = true;

        if (g_mission_elapsed_ms >= MISSION_TIMEOUT_MS) {
            medicine_fail(STOP_REASON_MISSION_TIMEOUT);
            break;
        }
        if (g_motor_disabled != 0U) {
            medicine_fail(STOP_REASON_MANUAL);
            break;
        }

        medicine_sync_action();
        if (g_mission.fault) {
            break;
        }
        action = medicine_mission_current_action(&g_mission);
        if (action == NULL) {
            medicine_fail(STOP_REASON_MISSION_FAULT);
            break;
        }

        if (medicine_route_is_turn_action(action->kind)) {
            /* A stalled I2C transaction must never preserve a spin command. */
            motors_brake();
        }
        mpu_sample_ok = mpu_sample_update();
        if (!mpu_sample_ok &&
            (g_mpu_read_fail_count >= MPU_MAX_CONSECUTIVE_FAILURES)) {
            medicine_fail(STOP_REASON_MPU_READ);
            break;
        }

        if (medicine_route_is_follow_action(action->kind)) {
            gray_scan_all_channels();
            gray_update_baseline_and_line();
            step_ok = medicine_follow_step(action);
        } else if (medicine_route_is_turn_action(action->kind)) {
            step_ok = medicine_turn_step(mpu_sample_ok);
        } else if (action->kind == MEDICINE_ACTION_WARD_STOP_2S) {
            step_ok = medicine_ward_stop_step(mpu_sample_ok);
        } else if (action->kind == MEDICINE_ACTION_COMPLETE) {
            medicine_mission_step(&g_mission, &no_event);
            medicine_publish_state();
        } else {
            medicine_fail(STOP_REASON_MISSION_FAULT);
            step_ok = false;
        }

        if (!step_ok && !g_mission.fault) {
            const uint8_t reason =
                (g_stop_reason == STOP_REASON_NONE)
                ? STOP_REASON_MISSION_FAULT
                : g_stop_reason;

            medicine_fail(reason);
        }

        gray_update_led();
        g_mission_tick_work_ms =
            (uint32_t)(g_system_ms - tick_start_ms);
        if (medicine_mission_tick_overrun(
                g_mission_tick_work_ms,
                MISSION_TICK_MS)) {
            g_mission_tick_overrun_count++;
            if (!g_mission.completed && !g_mission.fault) {
                medicine_fail(STOP_REASON_CONTROL_OVERRUN);
            }
            break;
        }
        wait_for_mission_tick(tick_start_ms);
        g_run_frame++;
        g_mission_elapsed_ms += MISSION_TICK_MS;
    }

    if (g_mission.fault) {
        g_phase = PHASE_STOP;
        if (g_stop_reason == STOP_REASON_NONE) {
            g_stop_reason = STOP_REASON_MISSION_FAULT;
        }
    } else if (g_single_turn_test_complete != 0U) {
        g_phase = PHASE_COMPLETE;
        g_stop_reason = STOP_REASON_TEST_COMPLETE;
    } else if (g_mission.completed) {
        g_phase = PHASE_COMPLETE;
        g_stop_reason = STOP_REASON_TEST_COMPLETE;
    } else if (g_stop_reason == STOP_REASON_NONE) {
        g_phase = PHASE_STOP;
        g_stop_reason = STOP_REASON_MISSION_FAULT;
    }
    motors_stop();

    while (1) {
        if (g_phase == PHASE_COMPLETE) {
            led_set(true);
            delay_ms_rough(80U);
            led_set(false);
            delay_ms_rough(320U);
        } else {
            led_set(true);
            delay_ms_rough(100U);
            led_set(false);
            delay_ms_rough(100U);
        }
    }
}

void GROUP1_IRQHandler(void)
{
    const uint32_t status = DL_GPIO_getEnabledInterruptStatus(
        ENCODER_PORT, ENCODER_INTERRUPT_MASK);

    if ((status & ENCODER_LEFT_A_PIN) != 0U) {
        const uint32_t pins = DL_GPIO_readPins(
            ENCODER_PORT, ENCODER_LEFT_A_PIN | ENCODER_LEFT_B_PIN);
        const bool a_high = (pins & ENCODER_LEFT_A_PIN) != 0U;
        const bool b_high = (pins & ENCODER_LEFT_B_PIN) != 0U;

        g_encoder_left_a_level = a_high ? 1U : 0U;
        g_encoder_left_b_level = b_high ? 1U : 0U;
        if (a_high != b_high) {
            g_encoder_left_count++;
        } else {
            g_encoder_left_count--;
        }
        g_encoder_left_edges++;
    }

    if ((status & ENCODER_RIGHT_A_PIN) != 0U) {
        const uint32_t pins = DL_GPIO_readPins(
            ENCODER_PORT, ENCODER_RIGHT_A_PIN | ENCODER_RIGHT_B_PIN);
        const bool a_high = (pins & ENCODER_RIGHT_A_PIN) != 0U;
        const bool b_high = (pins & ENCODER_RIGHT_B_PIN) != 0U;

        g_encoder_right_a_level = a_high ? 1U : 0U;
        g_encoder_right_b_level = b_high ? 1U : 0U;
        if (a_high != b_high) {
            g_encoder_right_count++;
        } else {
            g_encoder_right_count--;
        }
        g_encoder_right_edges++;
    }

    DL_GPIO_clearInterruptStatus(ENCODER_PORT, status);
}
