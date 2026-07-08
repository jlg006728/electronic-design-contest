/*
 * MSPM0G3507 PB11 buzzer isolation test.
 *
 * Purpose:
 *   Verify whether PB11 can drive the current buzzer wiring.
 *
 * Behavior:
 *   1. PB11 HIGH for about 1 second, LED on.
 *      This tests a 5V active buzzer or transistor/MOSFET driver.
 *   2. PB11 outputs a rough 2 kHz square wave for about 1 second, LED blinking.
 *      This tests a passive buzzer.
 *   3. PB11 LOW for about 1 second, LED off.
 *
 * Wiring under test:
 *   PB11 / IOMUX_PINCM28 -> buzzer driver input or buzzer +
 *   GND                 -> buzzer GND / driver GND
 */

#include <stdbool.h>
#include <stdint.h>
#include "ti_msp_dl_config.h"

#define NOINLINE                    __attribute__((noinline))

#define LED_PORT                    GPIOA
#define LED_PIN                     DL_GPIO_PIN_15
#define BUZZER_PORT                 GPIOB
#define BUZZER_PIN                  DL_GPIO_PIN_11

#define MOTOR_PORT                  GPIOB
#define AIN1_PIN                    DL_GPIO_PIN_0
#define AIN2_PIN                    DL_GPIO_PIN_1
#define BIN1_PIN                    DL_GPIO_PIN_2
#define BIN2_PIN                    DL_GPIO_PIN_3
#define STBY_PIN                    DL_GPIO_PIN_4

#define SERVO_PORT                  GPIOA
#define SERVO_X_PIN                 DL_GPIO_PIN_14
#define SERVO_Y_PIN                 DL_GPIO_PIN_17

volatile uint32_t g_test_loop_count = 0;
volatile uint8_t g_test_phase = 0;
volatile uint8_t g_buzzer_output_on = 0;

static NOINLINE void delay_cycles_rough(uint32_t cycles)
{
    volatile uint32_t i;

    for (i = 0U; i < cycles; i++) {
        __asm volatile("nop");
    }
}

static NOINLINE void delay_us_rough(uint32_t us)
{
    delay_cycles_rough(us * 8U);
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

static NOINLINE void buzzer_set(bool on)
{
    if (on) {
        DL_GPIO_setPins(BUZZER_PORT, BUZZER_PIN);
        g_buzzer_output_on = 1U;
    } else {
        DL_GPIO_clearPins(BUZZER_PORT, BUZZER_PIN);
        g_buzzer_output_on = 0U;
    }
}

static NOINLINE void led_set(bool on)
{
    if (on) {
        DL_GPIO_setPins(LED_PORT, LED_PIN);
    } else {
        DL_GPIO_clearPins(LED_PORT, LED_PIN);
    }
}

static NOINLINE void init_safe_outputs(void)
{
    init_output(IOMUX_PINCM12, MOTOR_PORT, AIN1_PIN, false);
    init_output(IOMUX_PINCM13, MOTOR_PORT, AIN2_PIN, false);
    init_output(IOMUX_PINCM15, MOTOR_PORT, BIN1_PIN, false);
    init_output(IOMUX_PINCM16, MOTOR_PORT, BIN2_PIN, false);
    init_output(IOMUX_PINCM17, MOTOR_PORT, STBY_PIN, false);

    init_output(IOMUX_PINCM36, SERVO_PORT, SERVO_X_PIN, false);
    init_output(IOMUX_PINCM39, SERVO_PORT, SERVO_Y_PIN, false);

    init_output(IOMUX_PINCM37, LED_PORT, LED_PIN, false);
    init_output(IOMUX_PINCM28, BUZZER_PORT, BUZZER_PIN, false);
}

static NOINLINE void active_buzzer_dc_test(void)
{
    g_test_phase = 1U;
    led_set(true);
    buzzer_set(true);
    delay_ms_rough(1000U);
    buzzer_set(false);
    led_set(false);
    delay_ms_rough(400U);
}

static NOINLINE void passive_buzzer_square_wave_test(void)
{
    g_test_phase = 2U;

    for (uint32_t i = 0U; i < 2000U; i++) {
        buzzer_set(true);
        if ((i & 0x40U) == 0U) {
            led_set(true);
        } else {
            led_set(false);
        }
        delay_us_rough(250U);

        buzzer_set(false);
        delay_us_rough(250U);
    }

    led_set(false);
    delay_ms_rough(400U);
}

static NOINLINE void off_gap(void)
{
    g_test_phase = 3U;
    buzzer_set(false);
    led_set(false);
    delay_ms_rough(1000U);
}

int main(void)
{
    SYSCFG_DL_init();
    init_safe_outputs();

    while (1) {
        g_test_loop_count++;
        active_buzzer_dc_test();
        passive_buzzer_square_wave_test();
        off_gap();
    }
}
