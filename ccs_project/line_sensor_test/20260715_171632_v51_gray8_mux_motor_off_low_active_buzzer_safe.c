/*
 * V51 MSPM0G3507 + YB-MYX05-V1.0 8-channel grayscale sensor test.
 *
 * Wiring:
 *   PB5 -> AD0, PB6 -> AD1, PB7 -> AD2, PA7 <- OUT.
 *
 * Safety:
 *   Motors, TB6612 STBY/PWM and servos are forced LOW.
 *   The low-trigger buzzer module is held HIGH (silent).
 *   OpenMV UART is not initialized.
 *
 * Startup:
 *   Place X1..X8 on the same white background before reset. The first
 *   50 scans establish a white reference without assuming OUT polarity.
 */

#include <stdbool.h>
#include <stdint.h>
#include "ti_msp_dl_config.h"

#define NOINLINE                         __attribute__((noinline))

#define FIRMWARE_VERSION                 51U
#define GRAY_CHANNEL_COUNT               8U
#define GRAY_SAMPLE_COUNT                9U
#define GRAY_HIGH_MAJORITY               5U
#define GRAY_ADDRESS_SETTLE_US           40U
#define GRAY_SAMPLE_INTERVAL_US          10U
#define GRAY_BASELINE_SCAN_COUNT         50U
#define GRAY_MAIN_LOOP_MS                20U
#define GRAY_POWERUP_DELAY_MS            300U
#define DELAY_LOOPS_PER_US               8U

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
#define GRAY_ADDRESS_MASK                (GRAY_AD0_PIN | GRAY_AD1_PIN | GRAY_AD2_PIN)

#define GRAY_OUT_PORT                    GPIOA
#define GRAY_OUT_PIN                     DL_GPIO_PIN_7

#define SERVO_PORT                       GPIOA
#define SERVO_X_PIN                      DL_GPIO_PIN_14
#define SERVO_Y_PIN                      DL_GPIO_PIN_17

#define LED_PORT                         GPIOA
#define LED_PIN                          DL_GPIO_PIN_15

#define BUZZER_PORT                      GPIOB
#define BUZZER_PIN                       DL_GPIO_PIN_11

volatile uint32_t g_firmware_version = FIRMWARE_VERSION;
volatile uint8_t g_motor_disabled = 1U;
volatile uint8_t g_buzzer_active_low = 1U;
volatile uint8_t g_buzzer_idle_level = 1U;
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

volatile uint8_t g_gray_x1_high_count = 0U;
volatile uint8_t g_gray_x2_high_count = 0U;
volatile uint8_t g_gray_x3_high_count = 0U;
volatile uint8_t g_gray_x4_high_count = 0U;
volatile uint8_t g_gray_x5_high_count = 0U;
volatile uint8_t g_gray_x6_high_count = 0U;
volatile uint8_t g_gray_x7_high_count = 0U;
volatile uint8_t g_gray_x8_high_count = 0U;

volatile uint8_t g_gray_all_low = 0U;
volatile uint8_t g_gray_all_high = 0U;
volatile uint8_t g_gray_mixed = 0U;
volatile uint8_t g_gray_baseline_ready = 0U;
volatile uint8_t g_gray_baseline_uniform = 0U;
volatile uint8_t g_gray_white_level_code = 2U;
volatile uint8_t g_gray_changed_count = 0U;
volatile uint8_t g_gray_line_valid = 0U;
volatile int16_t g_gray_line_error = 0;

volatile uint16_t g_gray_baseline_scan_count = 0U;
volatile uint16_t g_gray_baseline_votes[GRAY_CHANNEL_COUNT] = {0U};
volatile uint8_t g_gray_channel_high_count[GRAY_CHANNEL_COUNT] = {0U};
volatile uint32_t g_gray_scan_count = 0U;
volatile uint32_t g_gray_change_event_count = 0U;
volatile uint8_t g_led_state = 0U;

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
    uint8_t high_count = 0U;

    for (uint8_t sample = 0U; sample < GRAY_SAMPLE_COUNT; sample++) {
        uint8_t level =
            ((DL_GPIO_readPins(GRAY_OUT_PORT, GRAY_OUT_PIN) & GRAY_OUT_PIN) != 0U)
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

static NOINLINE void gray_scan_all_channels(void)
{
    uint8_t high_mask = 0U;
    uint8_t unstable_mask = 0U;

    for (uint8_t channel = 0U; channel < GRAY_CHANNEL_COUNT; channel++) {
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
    g_gray_all_low = (high_mask == 0x00U) ? 1U : 0U;
    g_gray_all_high = (high_mask == 0xFFU) ? 1U : 0U;
    g_gray_mixed = ((high_mask != 0x00U) && (high_mask != 0xFFU)) ? 1U : 0U;
    gray_publish_channel_values();
    g_gray_scan_count++;
}

static NOINLINE void gray_update_line_position(uint8_t changed_mask)
{
    static const int16_t weights[GRAY_CHANNEL_COUNT] = {
        -3500, -2500, -1500, -500, 500, 1500, 2500, 3500
    };
    int32_t weighted_sum = 0;
    uint8_t count = 0U;

    for (uint8_t channel = 0U; channel < GRAY_CHANNEL_COUNT; channel++) {
        if ((changed_mask & (uint8_t) (1U << channel)) != 0U) {
            weighted_sum += weights[channel];
            count++;
        }
    }

    g_gray_changed_count = count;
    if (count == 0U) {
        g_gray_line_valid = 0U;
        g_gray_line_error = 0;
        return;
    }

    g_gray_line_valid = 1U;
    g_gray_line_error = (int16_t) (weighted_sum / (int32_t) count);
}

static NOINLINE void gray_update_baseline_and_changes(void)
{
    if (g_gray_baseline_ready == 0U) {
        for (uint8_t channel = 0U; channel < GRAY_CHANNEL_COUNT; channel++) {
            if ((g_gray_raw_high_mask & (uint8_t) (1U << channel)) != 0U) {
                g_gray_baseline_votes[channel]++;
            }
        }

        g_gray_baseline_scan_count++;
        if (g_gray_baseline_scan_count >= GRAY_BASELINE_SCAN_COUNT) {
            uint8_t baseline_mask = 0U;

            for (uint8_t channel = 0U; channel < GRAY_CHANNEL_COUNT; channel++) {
                if (g_gray_baseline_votes[channel] >=
                    ((GRAY_BASELINE_SCAN_COUNT + 1U) / 2U)) {
                    baseline_mask |= (uint8_t) (1U << channel);
                }
            }

            g_gray_white_baseline_mask = baseline_mask;
            g_gray_baseline_uniform =
                ((baseline_mask == 0x00U) || (baseline_mask == 0xFFU)) ? 1U : 0U;
            g_gray_white_level_code =
                (baseline_mask == 0x00U) ? 0U :
                (baseline_mask == 0xFFU) ? 1U : 2U;
            g_gray_baseline_ready = 1U;
            g_gray_test_state = (g_gray_baseline_uniform != 0U) ? 2U : 3U;
            g_gray_changed_from_white_mask = 0U;
            g_gray_previous_change_mask = 0U;
        }
        return;
    }

    g_gray_changed_from_white_mask =
        (uint8_t) (g_gray_raw_high_mask ^ g_gray_white_baseline_mask);
    if (g_gray_changed_from_white_mask != g_gray_previous_change_mask) {
        g_gray_change_event_count++;
        g_gray_previous_change_mask = g_gray_changed_from_white_mask;
    }
    gray_update_line_position(g_gray_changed_from_white_mask);
}

static NOINLINE void gray_update_led(void)
{
    if (g_gray_test_state == 1U) {
        led_set(((g_gray_scan_count / 10U) & 0x01U) != 0U);
        return;
    }
    if (g_gray_test_state == 3U) {
        led_set(((g_gray_scan_count / 5U) & 0x01U) != 0U);
        return;
    }

    led_set(g_gray_changed_from_white_mask != 0U);
}

int main(void)
{
    SYSCFG_DL_init();
    safe_outputs_init();
    gray_sensor_init();

    g_gray_test_state = 0U;
    led_set(true);
    delay_ms_rough(100U);
    led_set(false);
    delay_ms_rough(GRAY_POWERUP_DELAY_MS - 100U);
    g_gray_test_state = 1U;

    while (1) {
        gray_scan_all_channels();
        gray_update_baseline_and_changes();
        gray_update_led();
        delay_ms_rough(GRAY_MAIN_LOOP_MS);
    }
}
