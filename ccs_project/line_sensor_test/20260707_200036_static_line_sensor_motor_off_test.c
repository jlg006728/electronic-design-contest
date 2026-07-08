/*
 * MSPM0G3507 7-channel line sensor static test.
 *
 * Motors are forced off:
 *   PB0/PB1/PB2/PB3/PB4 are configured as outputs and held LOW.
 *   PWM is not initialized.
 *
 * Watch variables:
 *   g_raw_s1..g_raw_s7: raw GPIO input level.
 *   g_s1..g_s7: normalized black-line state.
 *   g_line_raw_mask: raw bit mask.
 *   g_line_mask: normalized bit mask.
 *   g_line_valid / g_line_error / g_line_active_count.
 */

#include <stdbool.h>
#include <stdint.h>
#include "ti_msp_dl_config.h"

#define NOINLINE                    __attribute__((noinline))

#define LINE_SENSOR_ACTIVE_HIGH     1U

#define LED_PORT                    GPIOA
#define LED_PIN                     DL_GPIO_PIN_15

#define MOTOR_PORT                  GPIOB
#define AIN1_PIN                    DL_GPIO_PIN_0
#define AIN2_PIN                    DL_GPIO_PIN_1
#define BIN1_PIN                    DL_GPIO_PIN_2
#define BIN2_PIN                    DL_GPIO_PIN_3
#define STBY_PIN                    DL_GPIO_PIN_4

#define S1_PORT                     GPIOB
#define S1_PIN                      DL_GPIO_PIN_5
#define S2_PORT                     GPIOB
#define S2_PIN                      DL_GPIO_PIN_6
#define S3_PORT                     GPIOB
#define S3_PIN                      DL_GPIO_PIN_7
#define S4_PORT                     GPIOA
#define S4_PIN                      DL_GPIO_PIN_7
#define S5_PORT                     GPIOA
#define S5_PIN                      DL_GPIO_PIN_8
#define S6_PORT                     GPIOB
#define S6_PIN                      DL_GPIO_PIN_9
#define S7_PORT                     GPIOB
#define S7_PIN                      DL_GPIO_PIN_10

#define LINE_CENTER_OFFSET          0

volatile uint8_t g_raw_s1 = 0;
volatile uint8_t g_raw_s2 = 0;
volatile uint8_t g_raw_s3 = 0;
volatile uint8_t g_raw_s4 = 0;
volatile uint8_t g_raw_s5 = 0;
volatile uint8_t g_raw_s6 = 0;
volatile uint8_t g_raw_s7 = 0;

volatile uint8_t g_s1 = 0;
volatile uint8_t g_s2 = 0;
volatile uint8_t g_s3 = 0;
volatile uint8_t g_s4 = 0;
volatile uint8_t g_s5 = 0;
volatile uint8_t g_s6 = 0;
volatile uint8_t g_s7 = 0;

volatile uint8_t g_line_raw_mask = 0;
volatile uint8_t g_line_mask = 0;
volatile uint8_t g_line_valid = 0;
volatile uint8_t g_line_active_count = 0;
volatile int16_t g_line_error = 0;
volatile int32_t g_line_weighted_sum = 0;
volatile uint32_t g_loop_count = 0;
volatile uint8_t g_led_state = 0;

static NOINLINE void delay_cycles_rough(uint32_t cycles)
{
    volatile uint32_t i;

    for (i = 0U; i < cycles; i++) {
        __asm volatile("nop");
    }
}

static NOINLINE void delay_ms_rough(uint32_t ms)
{
    for (uint32_t i = 0U; i < ms; i++) {
        delay_cycles_rough(8000U);
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

static NOINLINE void init_input(uint32_t pincm)
{
    DL_GPIO_initDigitalInputFeatures(pincm,
        DL_GPIO_INVERSION_DISABLE,
        DL_GPIO_RESISTOR_PULL_DOWN,
        DL_GPIO_HYSTERESIS_ENABLE,
        DL_GPIO_WAKEUP_DISABLE);
}

static NOINLINE void motors_force_off(void)
{
    init_output(IOMUX_PINCM12, MOTOR_PORT, AIN1_PIN, false);
    init_output(IOMUX_PINCM13, MOTOR_PORT, AIN2_PIN, false);
    init_output(IOMUX_PINCM15, MOTOR_PORT, BIN1_PIN, false);
    init_output(IOMUX_PINCM16, MOTOR_PORT, BIN2_PIN, false);
    init_output(IOMUX_PINCM17, MOTOR_PORT, STBY_PIN, false);
}

static NOINLINE void line_inputs_init(void)
{
    init_input(IOMUX_PINCM18);
    init_input(IOMUX_PINCM23);
    init_input(IOMUX_PINCM24);
    init_input(IOMUX_PINCM14);
    init_input(IOMUX_PINCM19);
    init_input(IOMUX_PINCM26);
    init_input(IOMUX_PINCM27);
}

static NOINLINE uint8_t normalize_sensor(uint8_t raw)
{
    if (LINE_SENSOR_ACTIVE_HIGH != 0U) {
        return raw;
    }

    return (raw == 0U) ? 1U : 0U;
}

static NOINLINE void led_set(bool on)
{
    if (on) {
        DL_GPIO_setPins(LED_PORT, LED_PIN);
        g_led_state = 1U;
    } else {
        DL_GPIO_clearPins(LED_PORT, LED_PIN);
        g_led_state = 0U;
    }
}

static NOINLINE void read_line_sensors(void)
{
    uint32_t b = DL_GPIO_readPins(GPIOB,
        S1_PIN | S2_PIN | S3_PIN | S6_PIN | S7_PIN);
    uint32_t a = DL_GPIO_readPins(GPIOA, S4_PIN | S5_PIN);
    uint8_t raw_mask = 0U;
    uint8_t mask = 0U;

    g_raw_s1 = ((b & S1_PIN) != 0U) ? 1U : 0U;
    g_raw_s2 = ((b & S2_PIN) != 0U) ? 1U : 0U;
    g_raw_s3 = ((b & S3_PIN) != 0U) ? 1U : 0U;
    g_raw_s4 = ((a & S4_PIN) != 0U) ? 1U : 0U;
    g_raw_s5 = ((a & S5_PIN) != 0U) ? 1U : 0U;
    g_raw_s6 = ((b & S6_PIN) != 0U) ? 1U : 0U;
    g_raw_s7 = ((b & S7_PIN) != 0U) ? 1U : 0U;

    g_s1 = normalize_sensor(g_raw_s1);
    g_s2 = normalize_sensor(g_raw_s2);
    g_s3 = normalize_sensor(g_raw_s3);
    g_s4 = normalize_sensor(g_raw_s4);
    g_s5 = normalize_sensor(g_raw_s5);
    g_s6 = normalize_sensor(g_raw_s6);
    g_s7 = normalize_sensor(g_raw_s7);

    if (g_raw_s1 != 0U) { raw_mask |= 0x01U; }
    if (g_raw_s2 != 0U) { raw_mask |= 0x02U; }
    if (g_raw_s3 != 0U) { raw_mask |= 0x04U; }
    if (g_raw_s4 != 0U) { raw_mask |= 0x08U; }
    if (g_raw_s5 != 0U) { raw_mask |= 0x10U; }
    if (g_raw_s6 != 0U) { raw_mask |= 0x20U; }
    if (g_raw_s7 != 0U) { raw_mask |= 0x40U; }

    if (g_s1 != 0U) { mask |= 0x01U; }
    if (g_s2 != 0U) { mask |= 0x02U; }
    if (g_s3 != 0U) { mask |= 0x04U; }
    if (g_s4 != 0U) { mask |= 0x08U; }
    if (g_s5 != 0U) { mask |= 0x10U; }
    if (g_s6 != 0U) { mask |= 0x20U; }
    if (g_s7 != 0U) { mask |= 0x40U; }

    g_line_raw_mask = raw_mask;
    g_line_mask = mask;
}

static NOINLINE void update_line_position(void)
{
    int32_t sum = 0;
    uint8_t count = 0U;

    if (g_s1 != 0U) { sum += -3000; count++; }
    if (g_s2 != 0U) { sum += -2000; count++; }
    if (g_s3 != 0U) { sum += -1000; count++; }
    if (g_s4 != 0U) { sum += 0; count++; }
    if (g_s5 != 0U) { sum += 1000; count++; }
    if (g_s6 != 0U) { sum += 2000; count++; }
    if (g_s7 != 0U) { sum += 3000; count++; }

    g_line_active_count = count;
    g_line_weighted_sum = sum;

    if (count == 0U) {
        g_line_valid = 0U;
        g_line_error = 0;
        return;
    }

    g_line_valid = 1U;
    g_line_error = (int16_t) ((sum / (int32_t) count) - LINE_CENTER_OFFSET);
}

int main(void)
{
    SYSCFG_DL_init();
    motors_force_off();
    line_inputs_init();
    init_output(IOMUX_PINCM37, LED_PORT, LED_PIN, false);

    while (1) {
        read_line_sensors();
        update_line_position();
        led_set(g_line_valid != 0U);
        g_loop_count++;
        delay_ms_rough(20U);
    }
}
