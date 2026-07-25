/*
 * MSPM0G3507 + YB-MYX05-V1.0 fixed-white grayscale diagnostic.
 *
 * Wiring after electrical isolation is verified:
 *   PB5 -> AD0, PB6 -> AD1, PB7 -> AD2.
 *   Sensor OUT -> 10k/20k divider -> PA7.
 *
 * Hardware gate:
 *   Do not run the eight-channel scan while connecting any AD line raises
 *   the MSPM0 3.3V rail. Fix the 5V-to-3.3V isolation first.
 *
 * Safety:
 *   Motors, PWM, servos and OpenMV are disabled.
 *   The active-low buzzer output is held HIGH (silent).
 */

#include <stdbool.h>
#include <stdint.h>
#include "ti_msp_dl_config.h"

#define NOINLINE                         __attribute__((noinline))

#define TEST_VERSION                     20260723U
#define GRAY_CHANNEL_COUNT               8U
#define GRAY_SAMPLE_COUNT                9U
#define GRAY_HIGH_MAJORITY               5U
#define GRAY_ADDRESS_SETTLE_US           40U
#define GRAY_SAMPLE_INTERVAL_US          10U
#define GRAY_BASELINE_SCAN_COUNT         50U
#define GRAY_LOOP_DELAY_MS               20U
#define GRAY_POWERUP_DELAY_MS            300U
#define DELAY_LOOPS_PER_US               8U

#define TEST_STATE_BOOT                  0U
#define TEST_STATE_CALIBRATING           1U
#define TEST_STATE_READY                 2U
#define TEST_STATE_BASELINE_MIXED        3U

#define MOTOR_PORT                       GPIOB
#define AIN1_PIN                         DL_GPIO_PIN_0
#define AIN2_PIN                         DL_GPIO_PIN_1
#define BIN1_PIN                         DL_GPIO_PIN_2
#define BIN2_PIN                         DL_GPIO_PIN_3
#define STBY_PIN                         DL_GPIO_PIN_4

#define PWM_PORT                         GPIOA
#define PWMA_PIN                         DL_GPIO_PIN_12
#define PWMB_PIN                         DL_GPIO_PIN_13

#define GRAY_ADDRESS_PORT                GPIOB
#define GRAY_AD0_PIN                     DL_GPIO_PIN_5
#define GRAY_AD1_PIN                     DL_GPIO_PIN_6
#define GRAY_AD2_PIN                     DL_GPIO_PIN_7
#define GRAY_ADDRESS_MASK                \
    (GRAY_AD0_PIN | GRAY_AD1_PIN | GRAY_AD2_PIN)

#define GRAY_OUT_PORT                    GPIOA
#define GRAY_OUT_PIN                     DL_GPIO_PIN_7

#define SERVO_PORT                       GPIOA
#define SERVO_X_PIN                      DL_GPIO_PIN_14
#define SERVO_Y_PIN                      DL_GPIO_PIN_17

#define LED_PORT                         GPIOA
#define LED_PIN                          DL_GPIO_PIN_15

#define BUZZER_PORT                      GPIOB
#define BUZZER_PIN                       DL_GPIO_PIN_11

/* Primary Watch variables. */
volatile uint32_t g_test_version = TEST_VERSION;
volatile uint8_t g_test_state = TEST_STATE_BOOT;
volatile uint32_t g_scan_count = 0U;
volatile uint8_t g_white_mask = 0xFFU;
volatile uint8_t g_raw_mask = 0U;
volatile uint8_t g_black_mask = 0U;
volatile uint8_t g_seen_black_mask = 0U;
volatile uint8_t g_last_black_mask = 0U;
volatile uint8_t g_unstable_mask = 0U;
volatile uint8_t g_black_count = 0U;
volatile uint32_t g_change_count = 0U;

/* Optional per-channel Watch variables: 1 means black is detected now. */
volatile uint8_t g_x1_black = 0U;
volatile uint8_t g_x2_black = 0U;
volatile uint8_t g_x3_black = 0U;
volatile uint8_t g_x4_black = 0U;
volatile uint8_t g_x5_black = 0U;
volatile uint8_t g_x6_black = 0U;
volatile uint8_t g_x7_black = 0U;
volatile uint8_t g_x8_black = 0U;

/* Secondary diagnostics. */
volatile uint8_t g_line_valid = 0U;
volatile int16_t g_line_error = 0;
volatile uint8_t g_selected_channel = 0U;
volatile uint8_t g_out_level = 0U;
volatile uint8_t g_scan_heartbeat = 0U;
volatile uint8_t g_baseline_uniform = 0U;
volatile uint16_t g_baseline_scan_count = 0U;
volatile uint16_t g_baseline_high_votes[GRAY_CHANNEL_COUNT] = {0U};
volatile uint8_t g_channel_high_count[GRAY_CHANNEL_COUNT] = {0U};

static NOINLINE void delay_cycles_rough(uint32_t cycles)
{
    volatile uint32_t i;

    for (i = 0U; i < cycles; i++) {
        __asm volatile("nop");
    }
}

static NOINLINE void delay_us_rough(uint32_t us)
{
    delay_cycles_rough(us * DELAY_LOOPS_PER_US);
}

static NOINLINE void delay_ms_rough(uint32_t ms)
{
    for (uint32_t i = 0U; i < ms; i++) {
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
    DL_GPIO_initDigitalInputFeatures(pincm,
        DL_GPIO_INVERSION_DISABLE,
        DL_GPIO_RESISTOR_NONE,
        DL_GPIO_HYSTERESIS_ENABLE,
        DL_GPIO_WAKEUP_DISABLE);
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

static NOINLINE void gray_sensor_init(void)
{
    init_output(IOMUX_PINCM18, GRAY_ADDRESS_PORT, GRAY_AD0_PIN, false);
    init_output(IOMUX_PINCM23, GRAY_ADDRESS_PORT, GRAY_AD1_PIN, false);
    init_output(IOMUX_PINCM24, GRAY_ADDRESS_PORT, GRAY_AD2_PIN, false);
    init_input(IOMUX_PINCM14);
}

static NOINLINE void led_set(bool on)
{
    if (on) {
        DL_GPIO_setPins(LED_PORT, LED_PIN);
    } else {
        DL_GPIO_clearPins(LED_PORT, LED_PIN);
    }
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

    g_selected_channel = (uint8_t) (channel + 1U);
    delay_us_rough(GRAY_ADDRESS_SETTLE_US);
}

static NOINLINE uint8_t gray_sample_selected_channel(void)
{
    uint8_t high_count = 0U;

    for (uint8_t sample = 0U; sample < GRAY_SAMPLE_COUNT; sample++) {
        uint8_t level =
            ((DL_GPIO_readPins(GRAY_OUT_PORT, GRAY_OUT_PIN) &
                GRAY_OUT_PIN) != 0U)
            ? 1U
            : 0U;

        g_out_level = level;
        high_count = (uint8_t) (high_count + level);
        delay_us_rough(GRAY_SAMPLE_INTERVAL_US);
    }

    return high_count;
}

static NOINLINE void gray_scan_all_channels(void)
{
    uint8_t raw_mask = 0U;
    uint8_t unstable_mask = 0U;

    for (uint8_t channel = 0U; channel < GRAY_CHANNEL_COUNT; channel++) {
        uint8_t high_count;

        gray_select_channel(channel);
        high_count = gray_sample_selected_channel();
        g_channel_high_count[channel] = high_count;

        if (high_count >= GRAY_HIGH_MAJORITY) {
            raw_mask |= (uint8_t) (1U << channel);
        }
        if ((high_count != 0U) && (high_count != GRAY_SAMPLE_COUNT)) {
            unstable_mask |= (uint8_t) (1U << channel);
        }
    }

    g_raw_mask = raw_mask;
    g_unstable_mask = unstable_mask;
    g_scan_count++;
    g_scan_heartbeat ^= 1U;
}

static NOINLINE uint8_t count_bits(uint8_t value)
{
    uint8_t count = 0U;

    while (value != 0U) {
        count = (uint8_t) (count + (value & 0x01U));
        value >>= 1U;
    }
    return count;
}

static NOINLINE void publish_black_channels(uint8_t black_mask)
{
    g_x1_black = ((black_mask & 0x01U) != 0U) ? 1U : 0U;
    g_x2_black = ((black_mask & 0x02U) != 0U) ? 1U : 0U;
    g_x3_black = ((black_mask & 0x04U) != 0U) ? 1U : 0U;
    g_x4_black = ((black_mask & 0x08U) != 0U) ? 1U : 0U;
    g_x5_black = ((black_mask & 0x10U) != 0U) ? 1U : 0U;
    g_x6_black = ((black_mask & 0x20U) != 0U) ? 1U : 0U;
    g_x7_black = ((black_mask & 0x40U) != 0U) ? 1U : 0U;
    g_x8_black = ((black_mask & 0x80U) != 0U) ? 1U : 0U;
}

static NOINLINE void update_line_summary(uint8_t black_mask)
{
    static const int16_t weights[GRAY_CHANNEL_COUNT] = {
        -3500, -2500, -1500, -500, 500, 1500, 2500, 3500
    };
    int32_t weighted_sum = 0;
    uint8_t count = count_bits(black_mask);

    g_black_count = count;
    g_line_valid = ((count >= 1U) && (count <= 6U)) ? 1U : 0U;
    if (g_line_valid == 0U) {
        g_line_error = 0;
        return;
    }

    for (uint8_t channel = 0U; channel < GRAY_CHANNEL_COUNT; channel++) {
        if ((black_mask & (uint8_t) (1U << channel)) != 0U) {
            weighted_sum += weights[channel];
        }
    }
    g_line_error = (int16_t) (weighted_sum / (int32_t) count);
}

static NOINLINE void update_baseline_and_black(void)
{
    static uint8_t previous_black_mask = 0U;

    if (g_test_state == TEST_STATE_CALIBRATING) {
        for (uint8_t channel = 0U; channel < GRAY_CHANNEL_COUNT; channel++) {
            if ((g_raw_mask & (uint8_t) (1U << channel)) != 0U) {
                g_baseline_high_votes[channel]++;
            }
        }

        g_baseline_scan_count++;
        if (g_baseline_scan_count >= GRAY_BASELINE_SCAN_COUNT) {
            uint8_t baseline_mask = 0U;

            for (uint8_t channel = 0U; channel < GRAY_CHANNEL_COUNT; channel++) {
                if (g_baseline_high_votes[channel] >=
                    ((GRAY_BASELINE_SCAN_COUNT + 1U) / 2U)) {
                    baseline_mask |= (uint8_t) (1U << channel);
                }
            }

            g_white_mask = baseline_mask;
            g_baseline_uniform =
                ((baseline_mask == 0x00U) || (baseline_mask == 0xFFU))
                ? 1U
                : 0U;
            g_test_state = (g_baseline_uniform != 0U)
                ? TEST_STATE_READY
                : TEST_STATE_BASELINE_MIXED;
            g_black_mask = 0U;
            g_seen_black_mask = 0U;
            g_last_black_mask = 0U;
            g_change_count = 0U;
            previous_black_mask = 0U;
        }
        return;
    }

    g_black_mask = (uint8_t) (g_raw_mask ^ g_white_mask);
    publish_black_channels(g_black_mask);
    update_line_summary(g_black_mask);

    if (g_black_mask != previous_black_mask) {
        g_change_count++;
        previous_black_mask = g_black_mask;
    }
    if (g_black_mask != 0U) {
        g_seen_black_mask |= g_black_mask;
        g_last_black_mask = g_black_mask;
    }
}

static NOINLINE void update_status_led(void)
{
    if (g_test_state == TEST_STATE_CALIBRATING) {
        led_set(((g_scan_count / 10U) & 0x01U) != 0U);
    } else if (g_test_state == TEST_STATE_BASELINE_MIXED) {
        led_set(((g_scan_count / 5U) & 0x01U) != 0U);
    } else {
        led_set(g_black_mask != 0U);
    }
}

int main(void)
{
    SYSCFG_DL_init();
    safe_outputs_init();
    g_test_version = TEST_VERSION;
    g_test_state = TEST_STATE_BOOT;

    led_set(true);
    delay_ms_rough(100U);
    led_set(false);
    delay_ms_rough(GRAY_POWERUP_DELAY_MS - 100U);

    gray_sensor_init();
    g_test_state = TEST_STATE_READY;

    while (1) {
        gray_scan_all_channels();
        update_baseline_and_black();
        update_status_led();
        delay_ms_rough(GRAY_LOOP_DELAY_MS);
    }
}
