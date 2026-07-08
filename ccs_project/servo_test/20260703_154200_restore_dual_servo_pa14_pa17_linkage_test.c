/*
 * Dual-servo linkage test on formal candidate pins: PA14 and PA17.
 *
 * Why these pins:
 *   PA14 and PA17 are not used by the current motor, line sensor, encoder,
 *   UART, button, SWD, or PA2-PA6 forbidden-pin list.
 *   SDK LaunchPad notes show PA14 on J28_7 and PA17 on J3_28.
 *
 * Important:
 *   This is still a servo-only integration test. The program keeps TB6612
 *   STBY low and all direction pins low so the DC motors stay disabled.
 *
 * Wiring:
 *   TB6612 5V      -> both servo red wires
 *   TB6612 GND     -> both servo black/brown wires
 *   MSPM0 GND      -> TB6612 GND, common ground
 *   MSPM0 PA14     -> X / horizontal servo signal
 *   MSPM0 PA17     -> Y / pitch servo signal
 *
 * Expected behavior:
 *   Both servos move gently around center:
 *     center -> X left/Y up -> center -> X right/Y down -> loop.
 */

#include <stdbool.h>
#include <stdint.h>
#include "ti_msp_dl_config.h"

#define NOINLINE                __attribute__((noinline))

#define LED_PORT                GPIOA
#define LED_PIN                 DL_GPIO_PIN_15

#define SERVO_PORT              GPIOA
#define SERVO_X_PIN             DL_GPIO_PIN_14
#define SERVO_Y_PIN             DL_GPIO_PIN_17

#define MOTOR_PORT              GPIOB
#define AIN1_PIN                DL_GPIO_PIN_0
#define AIN2_PIN                DL_GPIO_PIN_1
#define BIN1_PIN                DL_GPIO_PIN_2
#define BIN2_PIN                DL_GPIO_PIN_3
#define STBY_PIN                DL_GPIO_PIN_4

#define SERVO_X_LEFT_US         1350U
#define SERVO_X_CENTER_US       1500U
#define SERVO_X_RIGHT_US        1650U

#define SERVO_Y_DOWN_US         1350U
#define SERVO_Y_CENTER_US       1500U
#define SERVO_Y_UP_US           1650U

#define SERVO_PERIOD_US         20000U
#define HOLD_FRAMES             70U

volatile uint16_t g_servo_x_us = SERVO_X_CENTER_US;
volatile uint16_t g_servo_y_us = SERVO_Y_CENTER_US;
volatile uint32_t g_step = 0;
volatile uint32_t g_frame_count = 0;

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

static NOINLINE void servo_signal_init(void)
{
    init_output(IOMUX_PINCM36, SERVO_PORT, SERVO_X_PIN, false);
    init_output(IOMUX_PINCM39, SERVO_PORT, SERVO_Y_PIN, false);
}

static NOINLINE void servo_output_one_frame(void)
{
    uint16_t x_us = g_servo_x_us;
    uint16_t y_us = g_servo_y_us;
    uint16_t first_us;
    uint16_t second_us;
    uint32_t first_pin;
    uint32_t second_pin;

    if (x_us <= y_us) {
        first_us = x_us;
        second_us = y_us;
        first_pin = SERVO_X_PIN;
        second_pin = SERVO_Y_PIN;
    } else {
        first_us = y_us;
        second_us = x_us;
        first_pin = SERVO_Y_PIN;
        second_pin = SERVO_X_PIN;
    }

    DL_GPIO_setPins(SERVO_PORT, SERVO_X_PIN | SERVO_Y_PIN);
    delay_us(first_us);
    DL_GPIO_clearPins(SERVO_PORT, first_pin);
    delay_us(second_us - first_us);
    DL_GPIO_clearPins(SERVO_PORT, second_pin);
    delay_us(SERVO_PERIOD_US - second_us);

    g_frame_count++;
}

static NOINLINE void hold_position(uint16_t x_us, uint16_t y_us, uint32_t step)
{
    g_step = step;
    g_servo_x_us = x_us;
    g_servo_y_us = y_us;
    DL_GPIO_togglePins(LED_PORT, LED_PIN);

    for (uint32_t i = 0; i < HOLD_FRAMES; i++) {
        servo_output_one_frame();
    }
}

int main(void)
{
    SYSCFG_DL_init();

    tb6612_safe_init();
    init_output(IOMUX_PINCM37, LED_PORT, LED_PIN, false);
    servo_signal_init();

    delay_ms(500U);

    while (1) {
        hold_position(SERVO_X_CENTER_US, SERVO_Y_CENTER_US, 0U);
        hold_position(SERVO_X_LEFT_US, SERVO_Y_UP_US, 1U);
        hold_position(SERVO_X_CENTER_US, SERVO_Y_CENTER_US, 2U);
        hold_position(SERVO_X_RIGHT_US, SERVO_Y_DOWN_US, 3U);
    }
}
