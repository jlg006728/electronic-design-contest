/**
 * 7-channel TCRT5000 digital line sensor read test - MSPM0G3507 LaunchPad
 *
 * Sensor wiring, from car front left to right:
 *   S1 DO -> PA2
 *   S2 DO -> PA5
 *   S3 DO -> PA6
 *   S4 DO -> PA7
 *   S5 DO -> PA8
 *   S6 DO -> PB9
 *   S7 DO -> PB10
 *
 * Sensor power:
 *   VCC -> 3.3V
 *   GND -> GND
 *   AO  -> not connected
 *
 * Watch these CCS Expressions:
 *   g_s1, g_s2, g_s3, g_s4, g_s5, g_s6, g_s7
 *   g_line_raw_mask
 */

#include <stdint.h>
#include "ti_msp_dl_config.h"

#define LED_PORT       GPIOA
#define LED_PIN        DL_GPIO_PIN_15

#define S1_PIN         DL_GPIO_PIN_2   /* PA2 */
#define S2_PIN         DL_GPIO_PIN_5   /* PA5 */
#define S3_PIN         DL_GPIO_PIN_6   /* PA6 */
#define S4_PIN         DL_GPIO_PIN_7   /* PA7 */
#define S5_PIN         DL_GPIO_PIN_8   /* PA8 */
#define S6_PIN         DL_GPIO_PIN_9   /* PB9 */
#define S7_PIN         DL_GPIO_PIN_10  /* PB10 */

volatile uint8_t g_s1 = 0;
volatile uint8_t g_s2 = 0;
volatile uint8_t g_s3 = 0;
volatile uint8_t g_s4 = 0;
volatile uint8_t g_s5 = 0;
volatile uint8_t g_s6 = 0;
volatile uint8_t g_s7 = 0;
volatile uint8_t g_line_raw_mask = 0;

static void delay_ms(uint32_t ms)
{
    for (uint32_t i = 0; i < ms; i++) {
        delay_cycles(32000);
    }
}

static void init_input(uint32_t pincm)
{
    DL_GPIO_initDigitalInputFeatures(pincm,
        DL_GPIO_INVERSION_DISABLE,
        DL_GPIO_RESISTOR_NONE,
        DL_GPIO_HYSTERESIS_ENABLE,
        DL_GPIO_WAKEUP_DISABLE);
}

static void init_output(uint32_t pincm, GPIO_Regs *port, uint32_t pin)
{
    DL_GPIO_initDigitalOutput(pincm);
    DL_GPIO_clearPins(port, pin);
    DL_GPIO_enableOutput(port, pin);
}

static void read_line_sensors(void)
{
    uint32_t a = DL_GPIO_readPins(GPIOA, S1_PIN | S2_PIN | S3_PIN | S4_PIN | S5_PIN);
    uint32_t b = DL_GPIO_readPins(GPIOB, S6_PIN | S7_PIN);

    g_s1 = ((a & S1_PIN) != 0U) ? 1U : 0U;
    g_s2 = ((a & S2_PIN) != 0U) ? 1U : 0U;
    g_s3 = ((a & S3_PIN) != 0U) ? 1U : 0U;
    g_s4 = ((a & S4_PIN) != 0U) ? 1U : 0U;
    g_s5 = ((a & S5_PIN) != 0U) ? 1U : 0U;
    g_s6 = ((b & S6_PIN) != 0U) ? 1U : 0U;
    g_s7 = ((b & S7_PIN) != 0U) ? 1U : 0U;

    g_line_raw_mask = 0U;
    g_line_raw_mask |= (uint8_t)(g_s1 << 0);
    g_line_raw_mask |= (uint8_t)(g_s2 << 1);
    g_line_raw_mask |= (uint8_t)(g_s3 << 2);
    g_line_raw_mask |= (uint8_t)(g_s4 << 3);
    g_line_raw_mask |= (uint8_t)(g_s5 << 4);
    g_line_raw_mask |= (uint8_t)(g_s6 << 5);
    g_line_raw_mask |= (uint8_t)(g_s7 << 6);
}

int main(void)
{
    SYSCFG_DL_init();

    init_input(IOMUX_PINCM7);   /* PA2  S1 */
    init_input(IOMUX_PINCM10);  /* PA5  S2 */
    init_input(IOMUX_PINCM11);  /* PA6  S3 */
    init_input(IOMUX_PINCM14);  /* PA7  S4 */
    init_input(IOMUX_PINCM19);  /* PA8  S5 */
    init_input(IOMUX_PINCM26);  /* PB9  S6 */
    init_input(IOMUX_PINCM27);  /* PB10 S7 */

    init_output(IOMUX_PINCM37, LED_PORT, LED_PIN);

    while (1) {
        read_line_sensors();
        DL_GPIO_togglePins(LED_PORT, LED_PIN);
        delay_ms(100);
    }
}
