/*
 * PA3 constant-high diagnostic for LP-MSPM0G3507.
 *
 * Purpose:
 *   Remove all timing ambiguity from the previous PA3/PA4 toggling test.
 *   PA3 is held HIGH forever, PA4 is held LOW forever, and PA15 LED is ON.
 *
 * Expected multimeter readings:
 *   red PA3, black GND  -> about 3.3V
 *   red 3V3, black PA3  -> about 0V
 *   red PA4, black GND  -> about 0V
 *   red 3V3, black PA4  -> about 3.3V
 *
 * If these are not true while LED is on, first suspect the physical pin
 * location/contact/wire path before changing servo PWM code.
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
volatile uint32_t g_pa3_expected = 1;
volatile uint32_t g_pa4_expected = 0;

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

int main(void)
{
    SYSCFG_DL_init();

    safe_motor_outputs_init();
    init_output(IOMUX_PINCM37, LED_PORT, LED_PIN, true);
    init_output(IOMUX_PINCM8, TEST_PORT, PA3_TEST_PIN, true);
    init_output(IOMUX_PINCM9, TEST_PORT, PA4_TEST_PIN, false);

    g_phase = 1U;

    while (1) {
        DL_GPIO_setPins(LED_PORT, LED_PIN);
        DL_GPIO_setPins(TEST_PORT, PA3_TEST_PIN);
        DL_GPIO_clearPins(TEST_PORT, PA4_TEST_PIN);
        delay_ms(1000U);
    }
}
