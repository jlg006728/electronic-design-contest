/**
 * Hold left motor forward - MSPM0G3507 LaunchPad
 *
 * Static diagnostic state:
 *   STBY = HIGH
 *   AIN1 = HIGH
 *   AIN2 = LOW
 *   PWMA = HIGH
 *   BIN1/BIN2/PWMB = LOW
 *
 * Expected:
 *   AO1-AO2 should be close to VM voltage and the left motor should spin.
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

int main(void)
{
    SYSCFG_DL_init();

    init_output(IOMUX_PINCM12, GPIOB, AIN1_PIN, true);   /* PB0 -> AIN1 */
    init_output(IOMUX_PINCM13, GPIOB, AIN2_PIN, false);  /* PB1 -> AIN2 */
    init_output(IOMUX_PINCM15, GPIOB, BIN1_PIN, false);  /* PB2 -> BIN1 */
    init_output(IOMUX_PINCM16, GPIOB, BIN2_PIN, false);  /* PB3 -> BIN2 */
    init_output(IOMUX_PINCM17, GPIOB, STBY_PIN, true);   /* PB4 -> STBY */

    init_output(IOMUX_PINCM34, GPIOA, PWMA_PIN, true);   /* PA12 -> PWMA */
    init_output(IOMUX_PINCM35, GPIOA, PWMB_PIN, false);  /* PA13 -> PWMB */
    init_output(IOMUX_PINCM37, GPIOA, LED_PIN, false);   /* PA15 */

    while (1) {
        DL_GPIO_setPins(LED_PORT, LED_PIN);
        delay_ms(100);
        DL_GPIO_clearPins(LED_PORT, LED_PIN);
        delay_ms(900);
    }
}
