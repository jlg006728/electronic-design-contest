/*
 * PA10/PA11 GPIO level diagnostic for LP-MSPM0G3507.
 *
 * Reason:
 *   PA3/PA4 are valid MCU pins, but they are not convenient pins on the
 *   LaunchPad 40-pin BoosterPack header. PA10 and PA11 are exposed on the
 *   BoosterPack header and are suitable for a temporary servo signal test.
 *
 * Test wiring:
 *   USB -> MSPM0 board only.
 *   Disconnect servo signal wires during this test.
 *   Measure PA10-to-GND and PA11-to-GND with a multimeter.
 *
 * Watch:
 *   g_phase
 *   g_pa10_expected
 *   g_pa11_expected
 *   g_cycle_count
 */

#include <stdbool.h>
#include <stdint.h>
#include "ti_msp_dl_config.h"

#define NOINLINE                __attribute__((noinline))

#define LED_PORT                GPIOA
#define LED_PIN                 DL_GPIO_PIN_15

#define TEST_PORT               GPIOA
#define PA10_TEST_PIN           DL_GPIO_PIN_10
#define PA11_TEST_PIN           DL_GPIO_PIN_11

#define MOTOR_PORT              GPIOB
#define AIN1_PIN                DL_GPIO_PIN_0
#define AIN2_PIN                DL_GPIO_PIN_1
#define BIN1_PIN                DL_GPIO_PIN_2
#define BIN2_PIN                DL_GPIO_PIN_3
#define STBY_PIN                DL_GPIO_PIN_4

volatile uint32_t g_phase = 0;
volatile uint32_t g_pa10_expected = 0;
volatile uint32_t g_pa11_expected = 0;
volatile uint32_t g_cycle_count = 0;

static NOINLINE void delay_ms(uint32_t ms)
{
    for (uint32_t i = 0; i < ms; i++) {
        delay_cycles(32000);
    }
}

static NOINLINE void init_output(uint32_t pincm, GPIO_Regs *port, uint32_t pin, bool high)
{
    DL_GPIO_initDigitalOutput(pincm);
    if (high) {
        DL_GPIO_setPins(port, pin);
    } else {
        DL_GPIO_clearPins(port, pin);
    }
    DL_GPIO_enableOutput(port, pin);
}

static NOINLINE void safe_motor_outputs_init(void)
{
    init_output(IOMUX_PINCM12, MOTOR_PORT, AIN1_PIN, false);
    init_output(IOMUX_PINCM13, MOTOR_PORT, AIN2_PIN, false);
    init_output(IOMUX_PINCM15, MOTOR_PORT, BIN1_PIN, false);
    init_output(IOMUX_PINCM16, MOTOR_PORT, BIN2_PIN, false);
    init_output(IOMUX_PINCM17, MOTOR_PORT, STBY_PIN, false);
}

static NOINLINE void set_test_outputs(bool pa10_high, bool pa11_high)
{
    if (pa10_high) {
        DL_GPIO_setPins(TEST_PORT, PA10_TEST_PIN);
    } else {
        DL_GPIO_clearPins(TEST_PORT, PA10_TEST_PIN);
    }

    if (pa11_high) {
        DL_GPIO_setPins(TEST_PORT, PA11_TEST_PIN);
    } else {
        DL_GPIO_clearPins(TEST_PORT, PA11_TEST_PIN);
    }

    g_pa10_expected = pa10_high ? 1U : 0U;
    g_pa11_expected = pa11_high ? 1U : 0U;
}

static NOINLINE void hold_phase(uint32_t phase, bool pa10_high, bool pa11_high, bool led_high, uint32_t hold_ms)
{
    g_phase = phase;
    set_test_outputs(pa10_high, pa11_high);

    if (led_high) {
        DL_GPIO_setPins(LED_PORT, LED_PIN);
    } else {
        DL_GPIO_clearPins(LED_PORT, LED_PIN);
    }

    delay_ms(hold_ms);
}

int main(void)
{
    SYSCFG_DL_init();

    safe_motor_outputs_init();
    init_output(IOMUX_PINCM37, LED_PORT, LED_PIN, false);
    init_output(IOMUX_PINCM21, TEST_PORT, PA10_TEST_PIN, false);
    init_output(IOMUX_PINCM22, TEST_PORT, PA11_TEST_PIN, false);

    while (1) {
        hold_phase(1U, true,  false, true,  3000U);  /* PA10 should be about 3.3V */
        hold_phase(2U, false, false, false, 1000U);  /* PA10/PA11 should be about 0V */
        hold_phase(3U, false, true,  true,  3000U);  /* PA11 should be about 3.3V */
        hold_phase(4U, false, false, false, 1000U);  /* PA10/PA11 should be about 0V */
        g_cycle_count++;
    }
}
