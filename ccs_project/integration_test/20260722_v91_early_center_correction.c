/*
 * V91: minimal eight-channel grayscale line follower with earlier correction.
 *
 * Wiring:
 *   PB5 -> AD0, PB6 -> AD1, PB7 -> AD2, PA7 <- OUT.
 *   PA12 -> PWMA, PA13 -> PWMB.
 *   PB0/PB1 -> AIN1/AIN2, PB2/PB3 -> BIN1/BIN2, PB4 -> STBY.
 *
 * Control:
 *   1. Calibrate all eight sensors on white background.
 *   2. Convert the detected line position directly to one P correction.
 *   3. If the line disappears briefly, search in the last known direction.
 *   4. Stop after a continuous lost-line timeout.
 *
 * OpenMV, MPU, servos, encoders and mission logic are not initialized.
 */

#include <stdbool.h>
#include <stdint.h>

#include "simple_line_control.h"
#include "simple_line_profile.h"
#include "ti_msp_dl_config.h"

#define NOINLINE                    __attribute__((noinline))

#define FIRMWARE_VERSION            91U

#define GRAY_CHANNEL_COUNT          8U
#define GRAY_SAMPLE_COUNT           9U
#define GRAY_HIGH_MAJORITY          5U
#define GRAY_ADDRESS_SETTLE_US      40U
#define GRAY_SAMPLE_INTERVAL_US     10U
#define GRAY_BASELINE_SCAN_COUNT    50U
#define GRAY_MAIN_LOOP_MS           10U
#define GRAY_POWERUP_DELAY_MS       300U
#define DELAY_LOOPS_PER_US          8U

#define PHASE_INIT                  0U
#define PHASE_CALIBRATION           1U
#define PHASE_LINE_RUN              2U
#define PHASE_COMPLETE              3U
#define PHASE_STOP                  4U

#define STOP_REASON_NONE            0U
#define STOP_REASON_TEST_COMPLETE   1U
#define STOP_REASON_BASELINE_INVALID 2U
#define STOP_REASON_LINE_LOST       3U

#define PWM_TIMER                   TIMG0
#define PWM_PERIOD                  1600U
#define PWM_LEFT_BASE_TICKS         SIMPLE_LINE_PROFILE_LEFT_BASE
#define PWM_RIGHT_BASE_TICKS        SIMPLE_LINE_PROFILE_RIGHT_BASE
#define LINE_MAX_ACTIVE_SENSORS     6U

#define RUN_MAX_FRAMES              3000U
#define START_READY_HOLD_MS         2000U

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

static const simple_line_config_t LINE_CONTROL_CONFIG =
    SIMPLE_LINE_PROFILE_CONFIG_INITIALIZER;

volatile uint32_t g_firmware_version = FIRMWARE_VERSION;
volatile uint8_t g_motor_disabled = 0U;
volatile uint8_t g_buzzer_idle_level = 1U;

volatile uint8_t g_phase = PHASE_INIT;
volatile uint8_t g_stop_reason = STOP_REASON_NONE;
volatile uint32_t g_run_frame = 0U;

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

volatile uint16_t g_gray_baseline_scan_count = 0U;
volatile uint16_t g_gray_baseline_votes[GRAY_CHANNEL_COUNT] = {0U};
volatile uint8_t g_gray_channel_high_count[GRAY_CHANNEL_COUNT] = {0U};
volatile uint32_t g_gray_scan_count = 0U;
volatile uint32_t g_gray_change_event_count = 0U;
volatile uint8_t g_led_state = 0U;

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

static NOINLINE void motors_stop(void)
{
    set_pwm(0U, 0U);
    DL_GPIO_clearPins(
        MOTOR_PORT,
        AIN1_PIN | AIN2_PIN | BIN1_PIN | BIN2_PIN | STBY_PIN);
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
        g_gray_line_error,
        g_line_last_nonzero_error,
        g_line_lost_count);

    g_line_correction = output.correction;
    g_line_lost_count = output.lost_count;

    if ((g_motor_disabled != 0U) || output.stop) {
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
    } else if (g_phase == PHASE_LINE_RUN) {
        if (g_gray_line_valid != 0U) {
            led_set(true);
        } else {
            led_set(((g_run_frame / 5U) & 0x01U) != 0U);
        }
    } else {
        led_set(false);
    }
}

static NOINLINE void baseline_error_loop(void)
{
    g_phase = PHASE_STOP;
    g_stop_reason = STOP_REASON_BASELINE_INVALID;
    motors_stop();

    while (1) {
        led_set(true);
        delay_ms_rough(100U);
        led_set(false);
        delay_ms_rough(100U);
    }
}

int main(void)
{
    SYSCFG_DL_init();
    g_firmware_version = FIRMWARE_VERSION;

    safe_outputs_init();
    gray_sensor_init();
    pwm_init();
    motors_stop();

    g_gray_test_state = 0U;
    g_phase = PHASE_INIT;
    led_set(true);
    delay_ms_rough(100U);
    led_set(false);
    delay_ms_rough(GRAY_POWERUP_DELAY_MS - 100U);

    g_gray_test_state = 1U;
    g_phase = PHASE_CALIBRATION;
    while (g_gray_baseline_ready == 0U) {
        gray_scan_all_channels();
        gray_update_baseline_and_line();
        gray_update_led();
        delay_ms_rough(GRAY_MAIN_LOOP_MS);
    }

    if (g_gray_baseline_uniform == 0U) {
        baseline_error_loop();
    }

    led_set(true);
    delay_ms_rough(START_READY_HOLD_MS);
    led_set(false);

    g_phase = PHASE_LINE_RUN;
    g_stop_reason = STOP_REASON_NONE;
    g_line_lost_count = 0U;
    set_pwm(PWM_LEFT_BASE_TICKS, PWM_RIGHT_BASE_TICKS);
    motors_forward();

    for (g_run_frame = 0U; g_run_frame < RUN_MAX_FRAMES; g_run_frame++) {
        gray_scan_all_channels();
        gray_update_baseline_and_line();

        if (!simple_line_follow_update()) {
            break;
        }

        gray_update_led();
        delay_ms_rough(GRAY_MAIN_LOOP_MS);
    }

    if (g_stop_reason == STOP_REASON_NONE) {
        g_phase = PHASE_COMPLETE;
        g_stop_reason = STOP_REASON_TEST_COMPLETE;
    }
    motors_stop();

    while (1) {
        led_set(true);
        delay_ms_rough(80U);
        led_set(false);
        delay_ms_rough(320U);
    }
}
