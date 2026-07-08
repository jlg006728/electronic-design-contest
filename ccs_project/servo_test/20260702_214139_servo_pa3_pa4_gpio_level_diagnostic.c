/*
 * PA3/PA4 GPIO level diagnostic for LP-MSPM0G3507.
 *
 * Purpose:
 *   Prove whether the physical PA3 and PA4 pins can be driven as ordinary
 *   GPIO outputs before reconnecting the two servo signal wires.
 *
 * Test wiring:
 *   USB -> MSPM0 board only.
 *   Disconnect servo signal wires from PA3/PA4 during this test.
 *   Measure PA3-to-GND and PA4-to-GND with a multimeter.
 *
 * Watch:
 *   g_phase
 *   g_pa3_expected
 *   g_pa4_expected
 *   g_cycle_count
 */

#include <stdbool.h>
#include <stdint.h>
#include "ti_msp_dl_config.h"

#define NOINLINE                __attribute__((noinline))

#define LED_PORT                GPIOA
#define LED_PIN                 DL_GPIO_PIN_15

#define TEST_PORT               GPIOA
#define PA3_TEST_PIN            DL_GPIO_PIN_3
#define PA4_TEST_PIN            DL_GPIO_PIN_4

#define MOTOR_PORT              GPIOB
#define AIN1_PIN                DL_GPIO_PIN_0
#define AIN2_PIN                DL_GPIO_PIN_1
#define BIN1_PIN                DL_GPIO_PIN_2
#define BIN2_PIN                DL_GPIO_PIN_3
#define STBY_PIN                DL_GPIO_PIN_4

volatile uint32_t g_phase = 0;
volatile uint32_t g_pa3_expected = 0;
volatile uint32_t g_pa4_expected = 0;
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

static NOINLINE void set_test_outputs(bool pa3_high, bool pa4_high)
{
    if (pa3_high) {
        DL_GPIO_setPins(TEST_PORT, PA3_TEST_PIN);
    } else {
        DL_GPIO_clearPins(TEST_PORT, PA3_TEST_PIN);
    }

    if (pa4_high) {
        DL_GPIO_setPins(TEST_PORT, PA4_TEST_PIN);
    } else {
        DL_GPIO_clearPins(TEST_PORT, PA4_TEST_PIN);
    }

    g_pa3_expected = pa3_high ? 1U : 0U;
    g_pa4_expected = pa4_high ? 1U : 0U;
}

static NOINLINE void hold_phase(uint32_t phase, bool pa3_high, bool pa4_high, bool led_high, uint32_t hold_ms)
{
    g_phase = phase;
    set_test_outputs(pa3_high, pa4_high);

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
    init_output(IOMUX_PINCM8, TEST_PORT, PA3_TEST_PIN, false);
    init_output(IOMUX_PINCM9, TEST_PORT, PA4_TEST_PIN, false);

    while (1) {
        hold_phase(1U, true,  false, true,  3000U);  /* PA3 should be about 3.3V */
        hold_phase(2U, false, false, false, 1000U);  /* PA3/PA4 should be about 0V */
        hold_phase(3U, false, true,  true,  3000U);  /* PA4 should be about 3.3V */
        hold_phase(4U, false, false, false, 1000U);  /* PA3/PA4 should be about 0V */
        g_cycle_count++;
    }
}
