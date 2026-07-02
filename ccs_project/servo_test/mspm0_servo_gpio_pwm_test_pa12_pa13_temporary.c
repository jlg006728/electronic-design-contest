/*
 * Temporary two-servo GPIO software PWM test for LP-MSPM0G3507.
 *
 * Signal pins:
 *   Lower servo signal -> PA12
 *   Upper servo signal -> PA13
 *
 * Important:
 *   PA12/PA13 are also the proven TB6612 PWMA/PWMB pins on the car. This
 *   program holds TB6612 STBY low, so motors stay disabled. Use this only as
 *   a temporary servo movement test, not as the final integrated pin plan.
 *
 * Servo power:
 *   Servo red          -> external 5V to 5.6V supply +
 *   Servo black/brown  -> external supply GND
 *   MSPM0 GND          -> external supply GND
 *
 * Watch:
 *   g_step
 *   g_servo_lower_us
 *   g_servo_upper_us
 *   g_frame_count
 */

#include <stdbool.h>
#include <stdint.h>
#include "ti_msp_dl_config.h"

#define NOINLINE                __attribute__((noinline))

#define LED_PORT                GPIOA
#define LED_PIN                 DL_GPIO_PIN_15

#define SERVO_PORT              GPIOA
#define SERVO_LOWER_PIN         DL_GPIO_PIN_12
#define SERVO_UPPER_PIN         DL_GPIO_PIN_13

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

volatile uint16_t g_servo_lower_us = SERVO_CENTER_US;
volatile uint16_t g_servo_upper_us = SERVO_CENTER_US;
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
    init_output(IOMUX_PINCM34, SERVO_PORT, SERVO_LOWER_PIN, false);
    init_output(IOMUX_PINCM35, SERVO_PORT, SERVO_UPPER_PIN, false);
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

static NOINLINE void servo_set_us(uint16_t lower_us, uint16_t upper_us)
{
    g_servo_lower_us = clamp_servo_us(lower_us);
    g_servo_upper_us = clamp_servo_us(upper_us);
}

static NOINLINE void servo_output_one_frame(void)
{
    uint16_t lower_us = g_servo_lower_us;
    uint16_t upper_us = g_servo_upper_us;

    DL_GPIO_setPins(SERVO_PORT, SERVO_LOWER_PIN | SERVO_UPPER_PIN);

    if (lower_us <= upper_us) {
        delay_us(lower_us);
        DL_GPIO_clearPins(SERVO_PORT, SERVO_LOWER_PIN);
        delay_us((uint32_t) upper_us - lower_us);
        DL_GPIO_clearPins(SERVO_PORT, SERVO_UPPER_PIN);
    } else {
        delay_us(upper_us);
        DL_GPIO_clearPins(SERVO_PORT, SERVO_UPPER_PIN);
        delay_us((uint32_t) lower_us - upper_us);
        DL_GPIO_clearPins(SERVO_PORT, SERVO_LOWER_PIN);
    }

    delay_us(SERVO_PERIOD_US - ((lower_us > upper_us) ? lower_us : upper_us));
    g_frame_count++;
}

static NOINLINE void hold_position(uint16_t lower_us, uint16_t upper_us, uint32_t step)
{
    g_step = step;
    servo_set_us(lower_us, upper_us);
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
        hold_position(SERVO_MIN_US,    SERVO_CENTER_US, 1U);
        hold_position(SERVO_MAX_US,    SERVO_CENTER_US, 2U);
        hold_position(SERVO_CENTER_US, SERVO_CENTER_US, 3U);
        hold_position(SERVO_CENTER_US, SERVO_MIN_US,    4U);
        hold_position(SERVO_CENTER_US, SERVO_MAX_US,    5U);
    }
}
