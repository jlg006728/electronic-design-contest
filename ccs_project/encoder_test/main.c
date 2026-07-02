/**
 * Encoder test - MSPM0G3507 LaunchPad
 *
 * Wiring:
 *   TB6612 board E1A -> PB12
 *   TB6612 board E1B -> PB13
 *   TB6612 board E2A -> PB14
 *   TB6612 board E2B -> PB15
 *   TB6612 board GND -> LaunchPad GND
 *
 * Usage:
 *   1. Flash and run.
 *   2. In CCS Debug view, add these Expressions:
 *      g_left_count, g_right_count,
 *      g_left_a_level, g_left_b_level, g_right_a_level, g_right_b_level
 *   3. Rotate each wheel by hand. The corresponding count should change.
 */

#include <stdbool.h>
#include <stdint.h>
#include "ti_msp_dl_config.h"

#define LED_PORT            GPIOA
#define LED_PIN             DL_GPIO_PIN_15

#define ENCODER_PORT        GPIOB
#define LEFT_A_PIN          DL_GPIO_PIN_12
#define LEFT_B_PIN          DL_GPIO_PIN_13
#define RIGHT_A_PIN         DL_GPIO_PIN_14
#define RIGHT_B_PIN         DL_GPIO_PIN_15
volatile int32_t g_left_count = 0;
volatile int32_t g_right_count = 0;
volatile uint32_t g_left_edges = 0;
volatile uint32_t g_right_edges = 0;

volatile uint8_t g_left_a_level = 0;
volatile uint8_t g_left_b_level = 0;
volatile uint8_t g_right_a_level = 0;
volatile uint8_t g_right_b_level = 0;

static void delay_ms(uint32_t ms)
{
    for (uint32_t i = 0; i < ms; i++) {
        delay_cycles(32000);
    }
}

static void update_encoder_levels(void)
{
    uint32_t pins = DL_GPIO_readPins(ENCODER_PORT,
        LEFT_A_PIN | LEFT_B_PIN | RIGHT_A_PIN | RIGHT_B_PIN);

    g_left_a_level = ((pins & LEFT_A_PIN) != 0U) ? 1U : 0U;
    g_left_b_level = ((pins & LEFT_B_PIN) != 0U) ? 1U : 0U;
    g_right_a_level = ((pins & RIGHT_A_PIN) != 0U) ? 1U : 0U;
    g_right_b_level = ((pins & RIGHT_B_PIN) != 0U) ? 1U : 0U;
}

static void gpio_init(void)
{
    DL_GPIO_initDigitalOutput(IOMUX_PINCM37);  /* PA15 LED */
    DL_GPIO_clearPins(LED_PORT, LED_PIN);
    DL_GPIO_enableOutput(LED_PORT, LED_PIN);

    DL_GPIO_initDigitalInputFeatures(IOMUX_PINCM29,  /* PB12 left A */
        DL_GPIO_INVERSION_DISABLE,
        DL_GPIO_RESISTOR_NONE,
        DL_GPIO_HYSTERESIS_ENABLE,
        DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initDigitalInputFeatures(IOMUX_PINCM30,  /* PB13 left B */
        DL_GPIO_INVERSION_DISABLE,
        DL_GPIO_RESISTOR_NONE,
        DL_GPIO_HYSTERESIS_ENABLE,
        DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initDigitalInputFeatures(IOMUX_PINCM31,  /* PB14 right A */
        DL_GPIO_INVERSION_DISABLE,
        DL_GPIO_RESISTOR_NONE,
        DL_GPIO_HYSTERESIS_ENABLE,
        DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initDigitalInputFeatures(IOMUX_PINCM32,  /* PB15 right B */
        DL_GPIO_INVERSION_DISABLE,
        DL_GPIO_RESISTOR_NONE,
        DL_GPIO_HYSTERESIS_ENABLE,
        DL_GPIO_WAKEUP_DISABLE);

    update_encoder_levels();
}

static void poll_encoder_edges(void)
{
    static uint8_t last_left_a = 0U;
    static uint8_t last_right_a = 0U;
    static bool initialized = false;

    update_encoder_levels();

    if (!initialized) {
        last_left_a = g_left_a_level;
        last_right_a = g_right_a_level;
        initialized = true;
        return;
    }

    if (g_left_a_level != last_left_a) {
        last_left_a = g_left_a_level;
        if (g_left_a_level != g_left_b_level) {
            g_left_count++;
        } else {
            g_left_count--;
        }
        g_left_edges++;
    }

    if (g_right_a_level != last_right_a) {
        last_right_a = g_right_a_level;
        if (g_right_a_level != g_right_b_level) {
            g_right_count++;
        } else {
            g_right_count--;
        }
        g_right_edges++;
    }
}

int main(void)
{
    SYSCFG_DL_init();
    gpio_init();
    uint32_t last_total_edges = 0U;

    for (uint32_t i = 0; i < 3U; i++) {
        DL_GPIO_togglePins(LED_PORT, LED_PIN);
        delay_ms(150);
        DL_GPIO_togglePins(LED_PORT, LED_PIN);
        delay_ms(150);
    }

    while (1) {
        poll_encoder_edges();
        uint32_t total_edges = g_left_edges + g_right_edges;

        if (total_edges != last_total_edges) {
            last_total_edges = total_edges;
            DL_GPIO_setPins(LED_PORT, LED_PIN);
            delay_ms(20);
            DL_GPIO_clearPins(LED_PORT, LED_PIN);
        } else {
            delay_ms(20);
        }
    }
}
