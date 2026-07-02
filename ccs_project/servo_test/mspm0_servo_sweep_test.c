/*
 * Two-servo gimbal sweep test for LP-MSPM0G3507.
 *
 * Wiring:
 *   Servo X signal -> PA3 / TIMG8_CCP0
 *   Servo Y signal -> PA4 / TIMG8_CCP1
 *   Servo red      -> external 5V
 *   Servo brown    -> external GND
 *   MSPM0 GND      -> external GND
 *
 * Do not power servos from the LaunchPad 3.3V/5V pin.
 * Motors are forced off in this test.
 *
 * Watch variables:
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

#define MOTOR_PORT              GPIOB
#define AIN1_PIN                DL_GPIO_PIN_0
#define AIN2_PIN                DL_GPIO_PIN_1
#define BIN1_PIN                DL_GPIO_PIN_2
#define BIN2_PIN                DL_GPIO_PIN_3
#define STBY_PIN                DL_GPIO_PIN_4

#define SERVO_TIMER             TIMG8
#define SERVO_PERIOD_US         20000U
#define SERVO_MIN_US            1000U
#define SERVO_CENTER_US         1500U
#define SERVO_MAX_US            2000U
#define SERVO_STEP_DELAY_MS     1200U

volatile uint16_t g_servo_x_us = SERVO_CENTER_US;
volatile uint16_t g_servo_y_us = SERVO_CENTER_US;
volatile uint16_t g_servo_x_compare = SERVO_PERIOD_US - SERVO_CENTER_US;
volatile uint16_t g_servo_y_compare = SERVO_PERIOD_US - SERVO_CENTER_US;
volatile uint32_t g_step = 0;

static NOINLINE void delay_ms(uint32_t ms)
{
    for (uint32_t i = 0; i < ms; i++) {
        delay_cycles(32000);
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

static NOINLINE void servo_pwm_init(void)
{
    DL_TimerG_reset(SERVO_TIMER);
    DL_TimerG_enablePower(SERVO_TIMER);
    delay_cycles(16);

    DL_GPIO_initPeripheralOutputFunction(IOMUX_PINCM8, IOMUX_PINCM8_PF_TIMG8_CCP0);
    DL_GPIO_enableOutput(GPIOA, DL_GPIO_PIN_3);
    DL_GPIO_initPeripheralOutputFunction(IOMUX_PINCM9, IOMUX_PINCM9_PF_TIMG8_CCP1);
    DL_GPIO_enableOutput(GPIOA, DL_GPIO_PIN_4);

    /*
     * BUSCLK is 32 MHz. prescale=31 gives 1 MHz timer ticks, so one timer
     * tick is 1 us and period=20000 gives 50 Hz servo PWM.
     */
    DL_TimerG_ClockConfig clock_cfg = {
        .clockSel = DL_TIMER_CLOCK_BUSCLK,
        .divideRatio = DL_TIMER_CLOCK_DIVIDE_1,
        .prescale = 31U,
    };
    DL_TimerG_setClockConfig(SERVO_TIMER, &clock_cfg);

    DL_TimerG_PWMConfig pwm_cfg = {
        .pwmMode = DL_TIMER_PWM_MODE_EDGE_ALIGN,
        .period = SERVO_PERIOD_US,
        .isTimerWithFourCC = false,
        .startTimer = DL_TIMER_STOP,
    };
    DL_TimerG_initPWMMode(SERVO_TIMER, &pwm_cfg);

    DL_TimerG_setCaptureCompareOutCtl(SERVO_TIMER,
        DL_TIMER_CC_OCTL_INIT_VAL_LOW,
        DL_TIMER_CC_OCTL_INV_OUT_DISABLED,
        DL_TIMER_CC_OCTL_SRC_FUNCVAL,
        DL_TIMERG_CAPTURE_COMPARE_0_INDEX);
    DL_TimerG_setCaptureCompareOutCtl(SERVO_TIMER,
        DL_TIMER_CC_OCTL_INIT_VAL_LOW,
        DL_TIMER_CC_OCTL_INV_OUT_DISABLED,
        DL_TIMER_CC_OCTL_SRC_FUNCVAL,
        DL_TIMERG_CAPTURE_COMPARE_1_INDEX);

    DL_TimerG_setCaptCompUpdateMethod(SERVO_TIMER,
        DL_TIMER_CC_UPDATE_METHOD_IMMEDIATE,
        DL_TIMERG_CAPTURE_COMPARE_0_INDEX);
    DL_TimerG_setCaptCompUpdateMethod(SERVO_TIMER,
        DL_TIMER_CC_UPDATE_METHOD_IMMEDIATE,
        DL_TIMERG_CAPTURE_COMPARE_1_INDEX);

    DL_TimerG_enableClock(SERVO_TIMER);
    DL_TimerG_setCCPDirection(SERVO_TIMER, DL_TIMER_CC0_OUTPUT | DL_TIMER_CC1_OUTPUT);
    DL_TimerG_startCounter(SERVO_TIMER);
}

static NOINLINE void servo_set_us(uint16_t x_us, uint16_t y_us)
{
    g_servo_x_us = clamp_servo_us(x_us);
    g_servo_y_us = clamp_servo_us(y_us);

    /*
     * This timer output polarity matches the motor PWM observation:
     * logical high time = period - compare.
     */
    g_servo_x_compare = SERVO_PERIOD_US - g_servo_x_us;
    g_servo_y_compare = SERVO_PERIOD_US - g_servo_y_us;

    DL_TimerG_setCaptureCompareValue(SERVO_TIMER, g_servo_x_compare, DL_TIMER_CC_0_INDEX);
    DL_TimerG_setCaptureCompareValue(SERVO_TIMER, g_servo_y_compare, DL_TIMER_CC_1_INDEX);
}

static NOINLINE void led_toggle(void)
{
    DL_GPIO_togglePins(LED_PORT, LED_PIN);
}

int main(void)
{
    SYSCFG_DL_init();

    safe_motor_outputs_init();
    init_output(IOMUX_PINCM37, LED_PORT, LED_PIN, false);
    servo_pwm_init();

    while (1) {
        g_step = 0U;
        servo_set_us(SERVO_CENTER_US, SERVO_CENTER_US);
        led_toggle();
        delay_ms(2000U);

        g_step = 1U;
        servo_set_us(SERVO_MIN_US, SERVO_CENTER_US);
        led_toggle();
        delay_ms(SERVO_STEP_DELAY_MS);

        g_step = 2U;
        servo_set_us(SERVO_MAX_US, SERVO_CENTER_US);
        led_toggle();
        delay_ms(SERVO_STEP_DELAY_MS);

        g_step = 3U;
        servo_set_us(SERVO_CENTER_US, SERVO_CENTER_US);
        led_toggle();
        delay_ms(SERVO_STEP_DELAY_MS);

        g_step = 4U;
        servo_set_us(SERVO_CENTER_US, SERVO_MIN_US);
        led_toggle();
        delay_ms(SERVO_STEP_DELAY_MS);

        g_step = 5U;
        servo_set_us(SERVO_CENTER_US, SERVO_MAX_US);
        led_toggle();
        delay_ms(SERVO_STEP_DELAY_MS);
    }
}
