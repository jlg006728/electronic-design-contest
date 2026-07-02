/*
 * Known-pin GPIO level diagnostic for LP-MSPM0G3507.
 *
 * Purpose:
 *   Use pins already proven or physically found by the user:
 *   - PA12 / PA13: previously verified as TB6612 PWM outputs.
 *   - PA24: user can physically locate this pin.
 *
 * Test wiring:
 *   USB -> MSPM0 board.
 *   Do not connect servo signal wires during this test.
 *   If TB6612 is still connected, STBY is held low here, so motors stay off.
 *
 * Watch:
 *   g_phase
 *   g_pin_name
 *   g_cycle_count
 */

#include <stdbool.h>
#include <stdint.h>
#include "ti_msp_dl_config.h"

#define NOINLINE                __attribute__((noinline))

#define LED_PORT                GPIOA
#define LED_PIN                 DL_GPIO_PIN_15

#define TEST_PORT               GPIOA
#define PA12_TEST_PIN           DL_GPIO_PIN_12
#define PA13_TEST_PIN           DL_GPIO_PIN_13
#define PA24_TEST_PIN           DL_GPIO_PIN_24

#define MOTOR_PORT              GPIOB
#define AIN1_PIN                DL_GPIO_PIN_0
#define AIN2_PIN                DL_GPIO_PIN_1
#define BIN1_PIN                DL_GPIO_PIN_2
#define BIN2_PIN                DL_GPIO_PIN_3
#define STBY_PIN                DL_GPIO_PIN_4

volatile uint32_t g_phase = 0;
volatile uint32_t g_cycle_count = 0;
volatile const char *g_pin_name = "INIT";

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

static NOINLINE void all_test_pins_low(void)
{
    DL_GPIO_clearPins(TEST_PORT, PA12_TEST_PIN | PA13_TEST_PIN | PA24_TEST_PIN);
}

static NOINLINE void hold_phase(uint32_t phase, const char *pin_name, uint32_t high_pin, uint32_t hold_ms)
{
    g_phase = phase;
    g_pin_name = pin_name;

    all_test_pins_low();
    if (high_pin != 0U) {
        DL_GPIO_setPins(TEST_PORT, high_pin);
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
    init_output(IOMUX_PINCM34, TEST_PORT, PA12_TEST_PIN, false);
    init_output(IOMUX_PINCM35, TEST_PORT, PA13_TEST_PIN, false);
    init_output(IOMUX_PINCM54, TEST_PORT, PA24_TEST_PIN, false);

    while (1) {
        hold_phase(1U, "PA12_HIGH", PA12_TEST_PIN, 3000U);
        hold_phase(2U, "ALL_LOW",   0U,            1000U);
        hold_phase(3U, "PA13_HIGH", PA13_TEST_PIN, 3000U);
        hold_phase(4U, "ALL_LOW",   0U,            1000U);
        hold_phase(5U, "PA24_HIGH", PA24_TEST_PIN, 3000U);
        hold_phase(6U, "ALL_LOW",   0U,            1000U);
        g_cycle_count++;
    }
}
