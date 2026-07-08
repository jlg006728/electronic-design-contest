/*
 * PA14 / PA17 physical pin diagnostic for formal servo candidates.
 *
 * Purpose:
 *   The servos worked on PA12, but did not move on PA14/PA17. This program
 *   removes PWM and servo behavior from the test. It only drives PA14 and PA17
 *   as slow GPIO levels so a multimeter can verify whether the physical header
 *   pins are reachable.
 *
 * Behavior:
 *   phase 1: PA14 HIGH, PA17 LOW,  LED ON   for about 3 s
 *   phase 2: PA14 LOW,  PA17 LOW,  LED OFF  for about 1 s
 *   phase 3: PA14 LOW,  PA17 HIGH, LED ON   for about 3 s
 *   phase 4: PA14 LOW,  PA17 LOW,  LED OFF  for about 1 s
 *
 * Expected multimeter readings:
 *   phase 1: PA14-GND about 3.3 V, PA17-GND about 0 V
 *   phase 3: PA14-GND about 0 V,   PA17-GND about 3.3 V
 *
 * Keep servos disconnected from PA14/PA17 while measuring this.
 */

#include <stdbool.h>
#include <stdint.h>
#include "ti_msp_dl_config.h"

#define NOINLINE                __attribute__((noinline))

#define LED_PORT                GPIOA
#define LED_PIN                 DL_GPIO_PIN_15

#define TEST_PORT               GPIOA
#define PA14_TEST_PIN           DL_GPIO_PIN_14
#define PA17_TEST_PIN           DL_GPIO_PIN_17

#define MOTOR_PORT              GPIOB
#define AIN1_PIN                DL_GPIO_PIN_0
#define AIN2_PIN                DL_GPIO_PIN_1
#define BIN1_PIN                DL_GPIO_PIN_2
#define BIN2_PIN                DL_GPIO_PIN_3
#define STBY_PIN                DL_GPIO_PIN_4

volatile uint32_t g_phase = 0;
volatile uint32_t g_cycle_count = 0;
volatile uint32_t g_pa14_expected = 0;
volatile uint32_t g_pa17_expected = 0;

static NOINLINE void delay_us(uint32_t us)
{
    delay_cycles(us * 32U);
}

static NOINLINE void delay_ms(uint32_t ms)
{
    for (uint32_t i = 0; i < ms; i++) {
        delay_us(1000U);
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

static NOINLINE void tb6612_safe_init(void)
{
    init_output(IOMUX_PINCM12, MOTOR_PORT, AIN1_PIN, false);
    init_output(IOMUX_PINCM13, MOTOR_PORT, AIN2_PIN, false);
    init_output(IOMUX_PINCM15, MOTOR_PORT, BIN1_PIN, false);
    init_output(IOMUX_PINCM16, MOTOR_PORT, BIN2_PIN, false);
    init_output(IOMUX_PINCM17, MOTOR_PORT, STBY_PIN, false);
}

static NOINLINE void set_test_pins(bool pa14_high, bool pa17_high)
{
    if (pa14_high) {
        DL_GPIO_setPins(TEST_PORT, PA14_TEST_PIN);
    } else {
        DL_GPIO_clearPins(TEST_PORT, PA14_TEST_PIN);
    }

    if (pa17_high) {
        DL_GPIO_setPins(TEST_PORT, PA17_TEST_PIN);
    } else {
        DL_GPIO_clearPins(TEST_PORT, PA17_TEST_PIN);
    }

    g_pa14_expected = pa14_high ? 1U : 0U;
    g_pa17_expected = pa17_high ? 1U : 0U;
}

static NOINLINE void enter_phase(uint32_t phase, bool pa14_high, bool pa17_high, uint32_t hold_ms)
{
    g_phase = phase;
    set_test_pins(pa14_high, pa17_high);

    if (pa14_high || pa17_high) {
        DL_GPIO_setPins(LED_PORT, LED_PIN);
    } else {
        DL_GPIO_clearPins(LED_PORT, LED_PIN);
    }

    delay_ms(hold_ms);
}

int main(void)
{
    SYSCFG_DL_init();

    tb6612_safe_init();
    init_output(IOMUX_PINCM37, LED_PORT, LED_PIN, false);
    init_output(IOMUX_PINCM36, TEST_PORT, PA14_TEST_PIN, false);
    init_output(IOMUX_PINCM39, TEST_PORT, PA17_TEST_PIN, false);

    delay_ms(500U);

    while (1) {
        enter_phase(1U, true, false, 3000U);
        enter_phase(2U, false, false, 1000U);
        enter_phase(3U, false, true, 3000U);
        enter_phase(4U, false, false, 1000U);
        g_cycle_count++;
    }
}
