/**
 * TB6612 full-speed GPIO retest - MSPM0G3507 LaunchPad
 *
 * This test bypasses timer PWM:
 *   PA12 -> PWMA is forced HIGH as GPIO
 *   PA13 -> PWMB is forced HIGH as GPIO
 *
 * If this does not spin the motors, the fault is not PWM code. Check VM power,
 * TB6612 wiring, motor 6P connector, and common ground.
 */

#include <stdbool.h>
#include <stdint.h>
#include "ti_msp_dl_config.h"

#define MOTOR_PORT       GPIOB
#define AIN1_PIN         DL_GPIO_PIN_0
#define AIN2_PIN         DL_GPIO_PIN_1
#define BIN1_PIN         DL_GPIO_PIN_2
#define BIN2_PIN         DL_GPIO_PIN_3
#define STBY_PIN         DL_GPIO_PIN_4

#define PWM_GPIO_PORT    GPIOA
#define PWMA_PIN         DL_GPIO_PIN_12
#define PWMB_PIN         DL_GPIO_PIN_13

#define LED_PORT         GPIOA
#define LED_PIN          DL_GPIO_PIN_15

static void delay_ms(uint32_t ms)
{
    for (uint32_t i = 0; i < ms; i++) {
        delay_cycles(32000);
    }
}

static void init_output(uint32_t pincm, GPIO_Regs *port, uint32_t pin, bool high)
{
    DL_GPIO_initDigitalOutput(pincm);
    if (high) {
        DL_GPIO_setPins(port, pin);
    } else {
        DL_GPIO_clearPins(port, pin);
    }
    DL_GPIO_enableOutput(port, pin);
}

static void all_stop(void)
{
    DL_GPIO_clearPins(MOTOR_PORT, AIN1_PIN | AIN2_PIN | BIN1_PIN | BIN2_PIN);
    DL_GPIO_clearPins(PWM_GPIO_PORT, PWMA_PIN | PWMB_PIN);
}

int main(void)
{
    SYSCFG_DL_init();

    init_output(IOMUX_PINCM12, GPIOB, AIN1_PIN, false);  /* PB0 */
    init_output(IOMUX_PINCM13, GPIOB, AIN2_PIN, false);  /* PB1 */
    init_output(IOMUX_PINCM15, GPIOB, BIN1_PIN, false);  /* PB2 */
    init_output(IOMUX_PINCM16, GPIOB, BIN2_PIN, false);  /* PB3 */
    init_output(IOMUX_PINCM17, GPIOB, STBY_PIN, true);   /* PB4 */

    init_output(IOMUX_PINCM34, GPIOA, PWMA_PIN, false);  /* PA12 as GPIO */
    init_output(IOMUX_PINCM35, GPIOA, PWMB_PIN, false);  /* PA13 as GPIO */
    init_output(IOMUX_PINCM37, GPIOA, LED_PIN, false);   /* PA15 */

    for (uint32_t i = 0; i < 3U; i++) {
        DL_GPIO_setPins(LED_PORT, LED_PIN);
        delay_ms(150);
        DL_GPIO_clearPins(LED_PORT, LED_PIN);
        delay_ms(150);
    }

    /* Left motor forward: AIN1=H, AIN2=L, PWMA=H */
    all_stop();
    DL_GPIO_setPins(MOTOR_PORT, AIN1_PIN | STBY_PIN);
    DL_GPIO_setPins(PWM_GPIO_PORT, PWMA_PIN);
    delay_ms(2500);

    /* Left motor reverse: AIN1=L, AIN2=H, PWMA=H */
    all_stop();
    DL_GPIO_setPins(MOTOR_PORT, AIN2_PIN | STBY_PIN);
    DL_GPIO_setPins(PWM_GPIO_PORT, PWMA_PIN);
    delay_ms(2500);

    /* Right motor forward: BIN1=H, BIN2=L, PWMB=H */
    all_stop();
    DL_GPIO_setPins(MOTOR_PORT, BIN1_PIN | STBY_PIN);
    DL_GPIO_setPins(PWM_GPIO_PORT, PWMB_PIN);
    delay_ms(2500);

    /* Right motor reverse: BIN1=L, BIN2=H, PWMB=H */
    all_stop();
    DL_GPIO_setPins(MOTOR_PORT, BIN2_PIN | STBY_PIN);
    DL_GPIO_setPins(PWM_GPIO_PORT, PWMB_PIN);
    delay_ms(2500);

    /* Both forward */
    all_stop();
    DL_GPIO_setPins(MOTOR_PORT, AIN1_PIN | BIN1_PIN | STBY_PIN);
    DL_GPIO_setPins(PWM_GPIO_PORT, PWMA_PIN | PWMB_PIN);
    delay_ms(3000);

    all_stop();

    while (1) {
        DL_GPIO_setPins(LED_PORT, LED_PIN);
        delay_ms(50);
        DL_GPIO_clearPins(LED_PORT, LED_PIN);
        delay_ms(200);
    }
}
