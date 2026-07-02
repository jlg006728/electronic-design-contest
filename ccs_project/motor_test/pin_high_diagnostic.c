/**
 * Pin high diagnostic - MSPM0G3507 LaunchPad
 *
 * Forces PA15 and PB0-PB4 high forever.
 * Use a multimeter to check whether the MCU pins and TB6612 inputs are wired.
 */

#include <stdbool.h>
#include <stdint.h>
#include "ti_msp_dl_config.h"

#define LED_PIN       DL_GPIO_PIN_15  /* PA15 */
#define AIN1_PIN      DL_GPIO_PIN_0   /* PB0 */
#define AIN2_PIN      DL_GPIO_PIN_1   /* PB1 */
#define BIN1_PIN      DL_GPIO_PIN_2   /* PB2 */
#define BIN2_PIN      DL_GPIO_PIN_3   /* PB3 */
#define STBY_PIN      DL_GPIO_PIN_4   /* PB4 */

static void init_output_high(uint32_t pincm, GPIO_Regs *port, uint32_t pin)
{
    DL_GPIO_initDigitalOutput(pincm);
    DL_GPIO_setPins(port, pin);
    DL_GPIO_enableOutput(port, pin);
}

int main(void)
{
    SYSCFG_DL_init();

    init_output_high(IOMUX_PINCM37, GPIOA, LED_PIN);   /* PA15 */
    init_output_high(IOMUX_PINCM12, GPIOB, AIN1_PIN);  /* PB0 */
    init_output_high(IOMUX_PINCM13, GPIOB, AIN2_PIN);  /* PB1 */
    init_output_high(IOMUX_PINCM15, GPIOB, BIN1_PIN);  /* PB2 */
    init_output_high(IOMUX_PINCM16, GPIOB, BIN2_PIN);  /* PB3 */
    init_output_high(IOMUX_PINCM17, GPIOB, STBY_PIN);  /* PB4 */

    while (1) {
        __WFI();
    }
}
