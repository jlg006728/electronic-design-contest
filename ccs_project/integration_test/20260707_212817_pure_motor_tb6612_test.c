/*
 * Pure motor test for MSPM0G3507 + TB6612.
 *
 * Purpose:
 *   Isolate motor/TB6612 wiring from line sensors, OpenMV, servos, buzzer,
 *   and mission logic.
 *
 * Test sequence, repeated forever:
 *   0: stop
 *   1: left motor forward
 *   2: left motor reverse
 *   3: right motor forward
 *   4: right motor reverse
 *   5: both motors forward
 *
 * Put the car on a stand before running this test.
 */

#include <stdbool.h>
#include <stdint.h>
#include "ti_msp_dl_config.h"

#define NOINLINE                    __attribute__((noinline))

#define SYSCLK_HZ                   32000000U

#define LED_PORT                    GPIOA
#define LED_PIN                     DL_GPIO_PIN_15

#define MOTOR_PORT                  GPIOB
#define AIN1_PIN                    DL_GPIO_PIN_0
#define AIN2_PIN                    DL_GPIO_PIN_1
#define BIN1_PIN                    DL_GPIO_PIN_2
#define BIN2_PIN                    DL_GPIO_PIN_3
#define STBY_PIN                    DL_GPIO_PIN_4

#define PWM_TIMER                   TIMG0
#define PWM_PERIOD                  1600U
#define TEST_PWM_TICKS              520U

#define STOP_MS                     800U
#define SINGLE_MOTOR_MS             2200U
#define BOTH_MOTOR_MS               3000U
#define LED_TOGGLE_MS               120U

volatile uint8_t g_test_phase = 0U;
volatile uint32_t g_left_pwm_ticks = 0U;
volatile uint32_t g_right_pwm_ticks = 0U;
volatile uint32_t g_left_compare_ticks = PWM_PERIOD;
volatile uint32_t g_right_compare_ticks = PWM_PERIOD;
volatile uint8_t g_stby_state = 0U;
volatile uint32_t g_phase_loop_count = 0U;

static NOINLINE void delay_ms(uint32_t ms)
{
    while (ms > 0U) {
        delay_cycles(SYSCLK_HZ / 1000U);
        ms--;
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

static NOINLINE void led_set(bool on)
{
    if (on) {
        DL_GPIO_setPins(LED_PORT, LED_PIN);
    } else {
        DL_GPIO_clearPins(LED_PORT, LED_PIN);
    }
}

static NOINLINE void led_toggle(void)
{
    DL_GPIO_togglePins(LED_PORT, LED_PIN);
}

static NOINLINE void motor_gpio_init(void)
{
    init_output(IOMUX_PINCM12, MOTOR_PORT, AIN1_PIN, false);
    init_output(IOMUX_PINCM13, MOTOR_PORT, AIN2_PIN, false);
    init_output(IOMUX_PINCM15, MOTOR_PORT, BIN1_PIN, false);
    init_output(IOMUX_PINCM16, MOTOR_PORT, BIN2_PIN, false);
    init_output(IOMUX_PINCM17, MOTOR_PORT, STBY_PIN, false);
}

static NOINLINE void pwm_init(void)
{
    DL_TimerG_reset(PWM_TIMER);
    DL_TimerG_enablePower(PWM_TIMER);
    delay_cycles(16U);

    DL_GPIO_initPeripheralOutputFunction(IOMUX_PINCM34, IOMUX_PINCM34_PF_TIMG0_CCP0);
    DL_GPIO_enableOutput(GPIOA, DL_GPIO_PIN_12);
    DL_GPIO_initPeripheralOutputFunction(IOMUX_PINCM35, IOMUX_PINCM35_PF_TIMG0_CCP1);
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

    DL_TimerG_enableClock(PWM_TIMER);
    DL_TimerG_setCCPDirection(PWM_TIMER, DL_TIMER_CC0_OUTPUT | DL_TIMER_CC1_OUTPUT);
    DL_TimerG_startCounter(PWM_TIMER);
}

static NOINLINE void set_pwm(uint32_t left_duty_ticks, uint32_t right_duty_ticks)
{
    if (left_duty_ticks > PWM_PERIOD) {
        left_duty_ticks = PWM_PERIOD;
    }
    if (right_duty_ticks > PWM_PERIOD) {
        right_duty_ticks = PWM_PERIOD;
    }

    g_left_pwm_ticks = left_duty_ticks;
    g_right_pwm_ticks = right_duty_ticks;
    g_left_compare_ticks = PWM_PERIOD - left_duty_ticks;
    g_right_compare_ticks = PWM_PERIOD - right_duty_ticks;

    DL_TimerG_setCaptureCompareValue(PWM_TIMER, g_left_compare_ticks, DL_TIMER_CC_0_INDEX);
    DL_TimerG_setCaptureCompareValue(PWM_TIMER, g_right_compare_ticks, DL_TIMER_CC_1_INDEX);
}

static NOINLINE void set_stby(bool enabled)
{
    if (enabled) {
        DL_GPIO_setPins(MOTOR_PORT, STBY_PIN);
        g_stby_state = 1U;
    } else {
        DL_GPIO_clearPins(MOTOR_PORT, STBY_PIN);
        g_stby_state = 0U;
    }
}

static NOINLINE void motors_coast(void)
{
    set_pwm(0U, 0U);
    DL_GPIO_clearPins(MOTOR_PORT, AIN1_PIN | AIN2_PIN | BIN1_PIN | BIN2_PIN);
    set_stby(true);
}

static NOINLINE void left_forward(void)
{
    DL_GPIO_clearPins(MOTOR_PORT, AIN1_PIN | AIN2_PIN | BIN1_PIN | BIN2_PIN);
    DL_GPIO_setPins(MOTOR_PORT, AIN1_PIN);
    set_stby(true);
    set_pwm(TEST_PWM_TICKS, 0U);
}

static NOINLINE void left_reverse(void)
{
    DL_GPIO_clearPins(MOTOR_PORT, AIN1_PIN | AIN2_PIN | BIN1_PIN | BIN2_PIN);
    DL_GPIO_setPins(MOTOR_PORT, AIN2_PIN);
    set_stby(true);
    set_pwm(TEST_PWM_TICKS, 0U);
}

static NOINLINE void right_forward(void)
{
    DL_GPIO_clearPins(MOTOR_PORT, AIN1_PIN | AIN2_PIN | BIN1_PIN | BIN2_PIN);
    DL_GPIO_setPins(MOTOR_PORT, BIN1_PIN);
    set_stby(true);
    set_pwm(0U, TEST_PWM_TICKS);
}

static NOINLINE void right_reverse(void)
{
    DL_GPIO_clearPins(MOTOR_PORT, AIN1_PIN | AIN2_PIN | BIN1_PIN | BIN2_PIN);
    DL_GPIO_setPins(MOTOR_PORT, BIN2_PIN);
    set_stby(true);
    set_pwm(0U, TEST_PWM_TICKS);
}

static NOINLINE void both_forward(void)
{
    DL_GPIO_clearPins(MOTOR_PORT, AIN1_PIN | AIN2_PIN | BIN1_PIN | BIN2_PIN);
    DL_GPIO_setPins(MOTOR_PORT, AIN1_PIN | BIN1_PIN);
    set_stby(true);
    set_pwm(TEST_PWM_TICKS, TEST_PWM_TICKS);
}

static NOINLINE void run_phase(uint8_t phase, uint32_t duration_ms)
{
    uint32_t elapsed_ms = 0U;

    g_test_phase = phase;
    g_phase_loop_count++;

    switch (phase) {
        case 1U:
            left_forward();
            break;
        case 2U:
            left_reverse();
            break;
        case 3U:
            right_forward();
            break;
        case 4U:
            right_reverse();
            break;
        case 5U:
            both_forward();
            break;
        default:
            motors_coast();
            led_set(false);
            break;
    }

    while (elapsed_ms < duration_ms) {
        delay_ms(LED_TOGGLE_MS);
        elapsed_ms += LED_TOGGLE_MS;
        if (phase != 0U) {
            led_toggle();
        }
    }
}

static NOINLINE void startup_blink(void)
{
    for (uint8_t i = 0U; i < 4U; i++) {
        led_set(true);
        delay_ms(80U);
        led_set(false);
        delay_ms(80U);
    }
}

int main(void)
{
    SYSCFG_DL_init();

    init_output(IOMUX_PINCM37, LED_PORT, LED_PIN, false);
    motor_gpio_init();
    pwm_init();
    motors_coast();
    startup_blink();

    while (1) {
        run_phase(0U, STOP_MS);
        run_phase(1U, SINGLE_MOTOR_MS);
        run_phase(0U, STOP_MS);
        run_phase(2U, SINGLE_MOTOR_MS);
        run_phase(0U, STOP_MS);
        run_phase(3U, SINGLE_MOTOR_MS);
        run_phase(0U, STOP_MS);
        run_phase(4U, SINGLE_MOTOR_MS);
        run_phase(0U, STOP_MS);
        run_phase(5U, BOTH_MOTOR_MS);
    }
}
