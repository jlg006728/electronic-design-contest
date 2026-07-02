/*
 * Two-servo GPIO software PWM test for LP-MSPM0G3507.
 *
 * This version avoids timer/IOMUX PWM complexity. PA3 and PA4 are ordinary
 * GPIO outputs, manually generating 50 Hz servo pulses.
 *
 * Wiring:
 *   Lower servo signal -> PA3
 *   Upper servo signal -> PA4
 *   Servo red          -> external 5V
 *   Servo black/brown  -> external GND
 *   MSPM0 GND          -> external GND
 *
 * Watch:
 *   g_step
 *   g_servo_x_us
 *   g_servo_y_us
 */

#include <stdbool.h>
#include <stdint.h>
#include "ti_msp_dl_config.h"

#define NOINLINE                __attribute__((noinline))

#define LED_PORT                GPIOA
#define LED_PIN                 DL_GPIO_PIN_15

#define SERVO_PORT              GPIOA
#define SERVO_X_PIN             DL_GPIO_PIN_3
#define SERVO_Y_PIN             DL_GPIO_PIN_4

#define MOTOR_PORT              GPIOB
#define AIN1_PIN                DL_GPIO_PIN_0
#define AIN2_PIN                DL_GPIO_PIN_1
#define BIN1_PIN                DL_GPIO_PIN_2
#define BIN2_PIN                DL_GPIO_PIN_3
#define STBY_PIN                DL_GPIO_PIN_4

#define SERVO_MIN_US            1000U
#define SERVO_CENTER_US         1500U
#define SERVO_MAX_US            2000U
#define SERVO_PERIOD_US         20000U
#define HOLD_FRAMES             75U

volatile uint16_t g_servo_x_us = SERVO_CENTER_US;
volatile uint16_t g_servo_y_us = SERVO_CENTER_US;
volatile uint32_t g_step = 0;
volatile uint32_t g_frame_count = 0;

static NOINLINE void delay_us(uint32_t us)
{
    for (uint32_t i = 0; i < us; i++) {
        delay_cycles(32);
    }
}

static NOINLINE void delay_ms(uint32_t ms)
{
    for (uint32_t i = 0; i < ms; i++) {
        delay_us(1000U);
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

static NOINLINE uint16_t clamp_servo_us(uint16_t pulse_us)
{
    if (pulse_us < SERVO_MIN_US) {
        return SERVO_MIN_US;
    }
    if (pulse_us > SERVO_MAX_US) {
        return SERVO_MAX_US;
    }
    return pulse_us;
}

static NOINLINE void safe_motor_outputs_init(void)
{
    init_output(IOMUX_PINCM12, MOTOR_PORT, AIN1_PIN, false);
    init_output(IOMUX_PINCM13, MOTOR_PORT, AIN2_PIN, false);
    init_output(IOMUX_PINCM15, MOTOR_PORT, BIN1_PIN, false);
    init_output(IOMUX_PINCM16, MOTOR_PORT, BIN2_PIN, false);
    init_output(IOMUX_PINCM17, MOTOR_PORT, STBY_PIN, false);
}

static NOINLINE void servo_gpio_init(void)
{
    init_output(IOMUX_PINCM8, SERVO_PORT, SERVO_X_PIN, false);
    init_output(IOMUX_PINCM9, SERVO_PORT, SERVO_Y_PIN, false);
}

static NOINLINE void servo_set_us(uint16_t x_us, uint16_t y_us)
{
    g_servo_x_us = clamp_servo_us(x_us);
    g_servo_y_us = clamp_servo_us(y_us);
}

static NOINLINE void servo_output_one_frame(void)
{
    uint16_t first_us = g_servo_x_us;
    uint16_t second_us = g_servo_y_us;

    DL_GPIO_setPins(SERVO_PORT, SERVO_X_PIN | SERVO_Y_PIN);

    if (first_us <= second_us) {
        delay_us(first_us);
        DL_GPIO_clearPins(SERVO_PORT, SERVO_X_PIN);
        delay_us((uint32_t)second_us - first_us);
        DL_GPIO_clearPins(SERVO_PORT, SERVO_Y_PIN);
    } else {
        delay_us(second_us);
        DL_GPIO_clearPins(SERVO_PORT, SERVO_Y_PIN);
        delay_us((uint32_t)first_us - second_us);
        DL_GPIO_clearPins(SERVO_PORT, SERVO_X_PIN);
    }

    delay_us(SERVO_PERIOD_US - ((first_us > second_us) ? first_us : second_us));
    g_frame_count++;
}

static NOINLINE void hold_position(uint16_t x_us, uint16_t y_us, uint32_t step)
{
    g_step = step;
    servo_set_us(x_us, y_us);
    DL_GPIO_togglePins(LED_PORT, LED_PIN);

    for (uint32_t i = 0; i < HOLD_FRAMES; i++) {
        servo_output_one_frame();
    }
}

int main(void)
{
    SYSCFG_DL_init();

    safe_motor_outputs_init();
    init_output(IOMUX_PINCM37, LED_PORT, LED_PIN, false);
    servo_gpio_init();

    delay_ms(500U);

    while (1) {
        hold_position(SERVO_CENTER_US, SERVO_CENTER_US, 0U);
        hold_position(SERVO_MIN_US, SERVO_CENTER_US, 1U);
        hold_position(SERVO_MAX_US, SERVO_CENTER_US, 2U);
        hold_position(SERVO_CENTER_US, SERVO_CENTER_US, 3U);
        hold_position(SERVO_CENTER_US, SERVO_MIN_US, 4U);
        hold_position(SERVO_CENTER_US, SERVO_MAX_US, 5U);
    }
}
