/**
 * 7-channel TCRT5000 line position test.
 *
 * This version only reads the line sensors and calculates line position.
 * It does not drive the motors.
 *
 * Sensor wiring, from car front left to right:
 *   S1 DO -> PB5
 *   S2 DO -> PB6
 *   S3 DO -> PB7
 *   S4 DO -> PA7
 *   S5 DO -> PA8
 *   S6 DO -> PB9
 *   S7 DO -> PB10
 *
 * Expected polarity for the current modules:
 *   black line = 1
 *   white ground = 0
 */

#include <stdint.h>
#include "ti_msp_dl_config.h"

#define LED_PORT       GPIOA
#define LED_PIN        DL_GPIO_PIN_15

#define S1_PIN         DL_GPIO_PIN_5   /* PB5 */
#define S2_PIN         DL_GPIO_PIN_6   /* PB6 */
#define S3_PIN         DL_GPIO_PIN_7   /* PB7 */
#define S4_PIN         DL_GPIO_PIN_7   /* PA7 */
#define S5_PIN         DL_GPIO_PIN_8   /* PA8 */
#define S6_PIN         DL_GPIO_PIN_9   /* PB9 */
#define S7_PIN         DL_GPIO_PIN_10  /* PB10 */

#define LINE_BLACK_IS_HIGH 1

volatile uint8_t g_s1 = 0;
volatile uint8_t g_s2 = 0;
volatile uint8_t g_s3 = 0;
volatile uint8_t g_s4 = 0;
volatile uint8_t g_s5 = 0;
volatile uint8_t g_s6 = 0;
volatile uint8_t g_s7 = 0;

volatile uint8_t g_line_raw_mask = 0;
volatile uint8_t g_line_active_count = 0;
volatile uint8_t g_line_valid = 0;
volatile int16_t g_line_error = 0;
volatile int16_t g_line_last_error = 0;
volatile int32_t g_line_weighted_sum = 0;
volatile uint32_t g_line_lost_count = 0;
volatile uint32_t g_debug_tick = 0;

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

static uint8_t normalize_sensor(uint8_t raw)
{
#if LINE_BLACK_IS_HIGH
    return raw;
#else
    return (raw == 0U) ? 1U : 0U;
#endif
}

static void read_line_sensors(void)
{
    uint32_t a = DL_GPIO_readPins(GPIOA, S4_PIN | S5_PIN);
    uint32_t b = DL_GPIO_readPins(GPIOB, S1_PIN | S2_PIN | S3_PIN | S6_PIN | S7_PIN);

    g_s1 = normalize_sensor(((b & S1_PIN) != 0U) ? 1U : 0U);
    g_s2 = normalize_sensor(((b & S2_PIN) != 0U) ? 1U : 0U);
    g_s3 = normalize_sensor(((b & S3_PIN) != 0U) ? 1U : 0U);
    g_s4 = normalize_sensor(((a & S4_PIN) != 0U) ? 1U : 0U);
    g_s5 = normalize_sensor(((a & S5_PIN) != 0U) ? 1U : 0U);
    g_s6 = normalize_sensor(((b & S6_PIN) != 0U) ? 1U : 0U);
    g_s7 = normalize_sensor(((b & S7_PIN) != 0U) ? 1U : 0U);

    g_line_raw_mask = 0U;
    g_line_raw_mask |= (uint8_t)(g_s1 << 0);
    g_line_raw_mask |= (uint8_t)(g_s2 << 1);
    g_line_raw_mask |= (uint8_t)(g_s3 << 2);
    g_line_raw_mask |= (uint8_t)(g_s4 << 3);
    g_line_raw_mask |= (uint8_t)(g_s5 << 4);
    g_line_raw_mask |= (uint8_t)(g_s6 << 5);
    g_line_raw_mask |= (uint8_t)(g_s7 << 6);
}

static void update_line_position(void)
{
    int32_t sum = 0;
    uint8_t count = 0;

    if (g_s1 != 0U) {
        sum += -3000;
        count++;
    }
    if (g_s2 != 0U) {
        sum += -2000;
        count++;
    }
    if (g_s3 != 0U) {
        sum += -1000;
        count++;
    }
    if (g_s4 != 0U) {
        sum += 0;
        count++;
    }
    if (g_s5 != 0U) {
        sum += 1000;
        count++;
    }
    if (g_s6 != 0U) {
        sum += 2000;
        count++;
    }
    if (g_s7 != 0U) {
        sum += 3000;
        count++;
    }

    g_line_active_count = count;
    g_line_weighted_sum = sum;

    if (count == 0U) {
        g_line_valid = 0U;
        g_line_lost_count++;
        g_line_error = g_line_last_error;
        return;
    }

    g_line_valid = 1U;
    g_line_error = (int16_t)(sum / (int32_t) count);
    g_line_last_error = g_line_error;
}

int main(void)
{
    SYSCFG_DL_init();

    init_input(IOMUX_PINCM18);  /* PB5  S1 */
    init_input(IOMUX_PINCM23);  /* PB6  S2 */
    init_input(IOMUX_PINCM24);  /* PB7  S3 */
    init_input(IOMUX_PINCM14);  /* PA7  S4 */
    init_input(IOMUX_PINCM19);  /* PA8  S5 */
    init_input(IOMUX_PINCM26);  /* PB9  S6 */
    init_input(IOMUX_PINCM27);  /* PB10 S7 */

    init_output(IOMUX_PINCM37, LED_PORT, LED_PIN);

    while (1) {
        read_line_sensors();
        update_line_position();
        g_debug_tick++;
        DL_GPIO_togglePins(LED_PORT, LED_PIN);
        delay_ms(100);
    }
}
