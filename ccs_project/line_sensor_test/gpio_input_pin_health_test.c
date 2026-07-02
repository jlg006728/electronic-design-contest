/**
 * GPIO input pin health test - MSPM0G3507 LaunchPad
 *
 * Purpose:
 *   Check whether the MSPM0 input pins used for line sensors still work.
 *
 * Test method:
 *   1. Disconnect all TCRT5000 DO wires from MSPM0.
 *   2. Flash and run this program.
 *   3. In CCS Watch, add:
 *        g_pa2, g_pa5, g_pa6, g_pa7, g_pa8, g_pb9, g_pb10, g_mask
 *   4. Use one jumper wire to connect each tested pin to 3.3V briefly.
 *      The corresponding variable should become 1.
 *   5. Remove the jumper. The variable should return to 0 due to pulldown.
 */

#include <stdint.h>
#include "ti_msp_dl_config.h"

#define LED_PORT       GPIOA
#define LED_PIN        DL_GPIO_PIN_15

#define PA2_PIN        DL_GPIO_PIN_2
#define PA5_PIN        DL_GPIO_PIN_5
#define PA6_PIN        DL_GPIO_PIN_6
#define PA7_PIN        DL_GPIO_PIN_7
#define PA8_PIN        DL_GPIO_PIN_8
#define PB9_PIN        DL_GPIO_PIN_9
#define PB10_PIN       DL_GPIO_PIN_10

volatile uint8_t g_pa2 = 0;
volatile uint8_t g_pa5 = 0;
volatile uint8_t g_pa6 = 0;
volatile uint8_t g_pa7 = 0;
volatile uint8_t g_pa8 = 0;
volatile uint8_t g_pb9 = 0;
volatile uint8_t g_pb10 = 0;
volatile uint8_t g_mask = 0;

static void delay_ms(uint32_t ms)
{
    for (uint32_t i = 0; i < ms; i++) {
        delay_cycles(32000);
    }
}

static void init_input_pulldown(uint32_t pincm)
{
    DL_GPIO_initDigitalInputFeatures(pincm,
        DL_GPIO_INVERSION_DISABLE,
        DL_GPIO_RESISTOR_PULL_DOWN,
        DL_GPIO_HYSTERESIS_ENABLE,
        DL_GPIO_WAKEUP_DISABLE);
}

static void init_output(uint32_t pincm, GPIO_Regs *port, uint32_t pin)
{
    DL_GPIO_initDigitalOutput(pincm);
    DL_GPIO_clearPins(port, pin);
    DL_GPIO_enableOutput(port, pin);
}

static void read_pins(void)
{
    uint32_t a = DL_GPIO_readPins(GPIOA, PA2_PIN | PA5_PIN | PA6_PIN | PA7_PIN | PA8_PIN);
    uint32_t b = DL_GPIO_readPins(GPIOB, PB9_PIN | PB10_PIN);

    g_pa2 = ((a & PA2_PIN) != 0U) ? 1U : 0U;
    g_pa5 = ((a & PA5_PIN) != 0U) ? 1U : 0U;
    g_pa6 = ((a & PA6_PIN) != 0U) ? 1U : 0U;
    g_pa7 = ((a & PA7_PIN) != 0U) ? 1U : 0U;
    g_pa8 = ((a & PA8_PIN) != 0U) ? 1U : 0U;
    g_pb9 = ((b & PB9_PIN) != 0U) ? 1U : 0U;
    g_pb10 = ((b & PB10_PIN) != 0U) ? 1U : 0U;

    g_mask = 0U;
    g_mask |= (uint8_t)(g_pa2 << 0);
    g_mask |= (uint8_t)(g_pa5 << 1);
    g_mask |= (uint8_t)(g_pa6 << 2);
    g_mask |= (uint8_t)(g_pa7 << 3);
    g_mask |= (uint8_t)(g_pa8 << 4);
    g_mask |= (uint8_t)(g_pb9 << 5);
    g_mask |= (uint8_t)(g_pb10 << 6);
}

int main(void)
{
    SYSCFG_DL_init();

    init_input_pulldown(IOMUX_PINCM7);   /* PA2 */
    init_input_pulldown(IOMUX_PINCM10);  /* PA5 */
    init_input_pulldown(IOMUX_PINCM11);  /* PA6 */
    init_input_pulldown(IOMUX_PINCM14);  /* PA7 */
    init_input_pulldown(IOMUX_PINCM19);  /* PA8 */
    init_input_pulldown(IOMUX_PINCM26);  /* PB9 */
    init_input_pulldown(IOMUX_PINCM27);  /* PB10 */
    init_output(IOMUX_PINCM37, LED_PORT, LED_PIN);

    while (1) {
        read_pins();
        DL_GPIO_togglePins(LED_PORT, LED_PIN);
        delay_ms(100);
    }
}
