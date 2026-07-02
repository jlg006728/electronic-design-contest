/**
 * Motor test with manual GPIO/PWM init - MSPM0G3507 LaunchPad
 *
 * This version does not rely on SysConfig GPIO pin setup.
 *
 * Wiring:
 *   PB0  -> AIN1
 *   PB1  -> AIN2
 *   PB2  -> BIN1
 *   PB3  -> BIN2
 *   PB4  -> STBY
 *   PA12 -> PWMA
 *   PA13 -> PWMB
 *   GND  -> TB6612 GND
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

#define LED_PORT         GPIOA
#define LED_PIN          DL_GPIO_PIN_15

#define PWM_TIMER        TIMG0
#define PWM_PERIOD       1600U

static void delay_ms(uint32_t ms)
{
    for (uint32_t i = 0; i < ms; i++) {
        delay_cycles(32000);
    }
}

static void init_output_pin(uint32_t pincm, GPIO_Regs *port, uint32_t pin, bool high)
{
    DL_GPIO_initDigitalOutput(pincm);
    if (high) {
        DL_GPIO_setPins(port, pin);
    } else {
        DL_GPIO_clearPins(port, pin);
    }
    DL_GPIO_enableOutput(port, pin);
}

static void gpio_init_manual(void)
{
    init_output_pin(IOMUX_PINCM12, GPIOB, AIN1_PIN, false);  /* PB0 */
    init_output_pin(IOMUX_PINCM13, GPIOB, AIN2_PIN, false);  /* PB1 */
    init_output_pin(IOMUX_PINCM15, GPIOB, BIN1_PIN, false);  /* PB2 */
    init_output_pin(IOMUX_PINCM16, GPIOB, BIN2_PIN, false);  /* PB3 */
    init_output_pin(IOMUX_PINCM17, GPIOB, STBY_PIN, false);  /* PB4 */
    init_output_pin(IOMUX_PINCM37, GPIOA, LED_PIN, false);   /* PA15 */
}

static void led_blink(uint32_t times, uint32_t half_period_ms)
{
    for (uint32_t i = 0; i < times; i++) {
        DL_GPIO_setPins(LED_PORT, LED_PIN);
        delay_ms(half_period_ms);
        DL_GPIO_clearPins(LED_PORT, LED_PIN);
        delay_ms(half_period_ms);
    }
}

static void pwm_init_manual(void)
{
    DL_TimerG_reset(PWM_TIMER);
    DL_TimerG_enablePower(PWM_TIMER);
    delay_cycles(16);

    DL_GPIO_initPeripheralOutputFunction(IOMUX_PINCM34, IOMUX_PINCM34_PF_TIMG0_CCP0);  /* PA12 */
    DL_GPIO_enableOutput(GPIOA, DL_GPIO_PIN_12);
    DL_GPIO_initPeripheralOutputFunction(IOMUX_PINCM35, IOMUX_PINCM35_PF_TIMG0_CCP1);  /* PA13 */
    DL_GPIO_enableOutput(GPIOA, DL_GPIO_PIN_13);

    DL_TimerG_ClockConfig clock_cfg = {
        .clockSel = DL_TIMER_CLOCK_BUSCLK,
        .divideRatio = DL_TIMER_CLOCK_DIVIDE_1,
        .prescale = 0U,
    };
    DL_TimerG_setClockConfig(PWM_TIMER, &clock_cfg);

    DL_TimerG_PWMConfig pwm_cfg = {
        .pwmMode = DL_TIMER_PWM_MODE_EDGE_ALIGN,
        .period = PWM_PERIOD,
        .isTimerWithFourCC = false,
        .startTimer = DL_TIMER_STOP,
    };
    DL_TimerG_initPWMMode(PWM_TIMER, &pwm_cfg);

    DL_TimerG_setCaptureCompareOutCtl(PWM_TIMER,
        DL_TIMER_CC_OCTL_INIT_VAL_LOW,
        DL_TIMER_CC_OCTL_INV_OUT_DISABLED,
        DL_TIMER_CC_OCTL_SRC_FUNCVAL,
        DL_TIMERG_CAPTURE_COMPARE_0_INDEX);
    DL_TimerG_setCaptureCompareOutCtl(PWM_TIMER,
        DL_TIMER_CC_OCTL_INIT_VAL_LOW,
        DL_TIMER_CC_OCTL_INV_OUT_DISABLED,
        DL_TIMER_CC_OCTL_SRC_FUNCVAL,
        DL_TIMERG_CAPTURE_COMPARE_1_INDEX);

    DL_TimerG_setCaptCompUpdateMethod(PWM_TIMER,
        DL_TIMER_CC_UPDATE_METHOD_IMMEDIATE,
        DL_TIMERG_CAPTURE_COMPARE_0_INDEX);
    DL_TimerG_setCaptCompUpdateMethod(PWM_TIMER,
        DL_TIMER_CC_UPDATE_METHOD_IMMEDIATE,
        DL_TIMERG_CAPTURE_COMPARE_1_INDEX);

    DL_TimerG_setCaptureCompareValue(PWM_TIMER, 0U, DL_TIMER_CC_0_INDEX);
    DL_TimerG_setCaptureCompareValue(PWM_TIMER, 0U, DL_TIMER_CC_1_INDEX);
    DL_TimerG_enableClock(PWM_TIMER);
    DL_TimerG_setCCPDirection(PWM_TIMER, DL_TIMER_CC0_OUTPUT | DL_TIMER_CC1_OUTPUT);
    DL_TimerG_startCounter(PWM_TIMER);
}

static uint32_t duty_to_ticks(int32_t duty_percent)
{
    if (duty_percent < 0) {
        duty_percent = -duty_percent;
    }
    if (duty_percent > 100) {
        duty_percent = 100;
    }
    return ((uint32_t)duty_percent * PWM_PERIOD) / 100U;
}

static void motor_left(int32_t duty_percent)
{
    DL_GPIO_clearPins(MOTOR_PORT, AIN1_PIN | AIN2_PIN);
    if (duty_percent > 0) {
        DL_GPIO_setPins(MOTOR_PORT, AIN1_PIN);
    } else if (duty_percent < 0) {
        DL_GPIO_setPins(MOTOR_PORT, AIN2_PIN);
    }
    DL_TimerG_setCaptureCompareValue(PWM_TIMER, duty_to_ticks(duty_percent), DL_TIMER_CC_0_INDEX);
}

static void motor_right(int32_t duty_percent)
{
    DL_GPIO_clearPins(MOTOR_PORT, BIN1_PIN | BIN2_PIN);
    if (duty_percent > 0) {
        DL_GPIO_setPins(MOTOR_PORT, BIN1_PIN);
    } else if (duty_percent < 0) {
        DL_GPIO_setPins(MOTOR_PORT, BIN2_PIN);
    }
    DL_TimerG_setCaptureCompareValue(PWM_TIMER, duty_to_ticks(duty_percent), DL_TIMER_CC_1_INDEX);
}

static void motor_stop(void)
{
    motor_left(0);
    motor_right(0);
    DL_GPIO_clearPins(MOTOR_PORT, AIN1_PIN | AIN2_PIN | BIN1_PIN | BIN2_PIN);
}

int main(void)
{
    SYSCFG_DL_init();
    gpio_init_manual();
    pwm_init_manual();

    led_blink(3, 150);

    DL_GPIO_setPins(MOTOR_PORT, STBY_PIN);
    delay_ms(500);

    motor_left(50);
    delay_ms(2000);
    motor_stop();
    delay_ms(500);

    motor_left(-50);
    delay_ms(2000);
    motor_stop();
    delay_ms(500);

    motor_right(50);
    delay_ms(2000);
    motor_stop();
    delay_ms(500);

    motor_right(-50);
    delay_ms(2000);
    motor_stop();
    delay_ms(500);

    motor_left(30);
    motor_right(30);
    delay_ms(1500);

    motor_left(50);
    motor_right(50);
    delay_ms(1500);

    motor_left(70);
    motor_right(70);
    delay_ms(1500);

    motor_stop();
    DL_GPIO_clearPins(MOTOR_PORT, STBY_PIN);

    while (1) {
        led_blink(1, 50);
        delay_ms(200);
    }
}
