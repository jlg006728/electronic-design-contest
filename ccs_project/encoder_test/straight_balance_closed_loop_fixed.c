/**
 * Fixed straight balance closed-loop test - MSPM0G3507 LaunchPad
 *
 * Use this version after confirming TIMG0 compare value is inverse to the
 * effective motor duty on the current PA12/PA13 PWM configuration.
 *
 * g_left_pwm_ticks / g_right_pwm_ticks are logical duty ticks:
 *   larger value = faster motor
 *
 * Internally the code writes:
 *   compare = PWM_PERIOD - duty_ticks
 */

#include <stdbool.h>
#include <stdint.h>
#include "ti_msp_dl_config.h"

#define MOTOR_PORT          GPIOB
#define AIN1_PIN            DL_GPIO_PIN_0
#define AIN2_PIN            DL_GPIO_PIN_1
#define BIN1_PIN            DL_GPIO_PIN_2
#define BIN2_PIN            DL_GPIO_PIN_3
#define STBY_PIN            DL_GPIO_PIN_4

#define PWM_TIMER           TIMG0
#define PWM_PERIOD          1600U
#define PWM_BASE_TICKS      900U
#define PWM_MIN_TICKS       260U
#define PWM_MAX_TICKS       1500U
#define PWM_CORR_LIMIT      620

#define ENCODER_PORT        GPIOB
#define LEFT_A_PIN          DL_GPIO_PIN_12
#define LEFT_B_PIN          DL_GPIO_PIN_13
#define RIGHT_A_PIN         DL_GPIO_PIN_14
#define RIGHT_B_PIN         DL_GPIO_PIN_15
#define ENCODER_INT_MASK    (LEFT_A_PIN | RIGHT_A_PIN)

#define LED_PORT            GPIOA
#define LED_PIN             DL_GPIO_PIN_15

volatile int32_t g_left_count = 0;
volatile int32_t g_right_count = 0;
volatile uint32_t g_left_edges = 0;
volatile uint32_t g_right_edges = 0;

volatile int32_t g_left_delta_edges = 0;
volatile int32_t g_right_delta_edges = 0;
volatile int32_t g_balance_error = 0;
volatile int32_t g_pwm_correction = 0;
volatile uint32_t g_left_pwm_ticks = PWM_BASE_TICKS;
volatile uint32_t g_right_pwm_ticks = PWM_BASE_TICKS;
volatile uint32_t g_left_compare_ticks = PWM_PERIOD - PWM_BASE_TICKS;
volatile uint32_t g_right_compare_ticks = PWM_PERIOD - PWM_BASE_TICKS;

volatile int32_t g_debug_left_edges_1s = 0;
volatile int32_t g_debug_right_edges_1s = 0;
volatile int32_t g_debug_error_1s = 0;
volatile uint32_t g_debug_left_pwm_1s = PWM_BASE_TICKS;
volatile uint32_t g_debug_right_pwm_1s = PWM_BASE_TICKS;

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

static void init_input(uint32_t pincm)
{
    DL_GPIO_initDigitalInputFeatures(pincm,
        DL_GPIO_INVERSION_DISABLE,
        DL_GPIO_RESISTOR_NONE,
        DL_GPIO_HYSTERESIS_ENABLE,
        DL_GPIO_WAKEUP_DISABLE);
}

static void encoder_init(void)
{
    init_input(IOMUX_PINCM29);  /* PB12 left A */
    init_input(IOMUX_PINCM30);  /* PB13 left B */
    init_input(IOMUX_PINCM31);  /* PB14 right A */
    init_input(IOMUX_PINCM32);  /* PB15 right B */

    DL_GPIO_setLowerPinsPolarity(ENCODER_PORT,
        DL_GPIO_PIN_12_EDGE_RISE_FALL |
        DL_GPIO_PIN_14_EDGE_RISE_FALL);
    DL_GPIO_clearInterruptStatus(ENCODER_PORT, ENCODER_INT_MASK);
    DL_GPIO_enableInterrupt(ENCODER_PORT, ENCODER_INT_MASK);
    NVIC_EnableIRQ(GPIOB_INT_IRQn);
}

static void pwm_init(void)
{
    DL_TimerG_reset(PWM_TIMER);
    DL_TimerG_enablePower(PWM_TIMER);
    delay_cycles(16);

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

static uint32_t clamp_pwm(int32_t ticks)
{
    if (ticks < (int32_t) PWM_MIN_TICKS) {
        return PWM_MIN_TICKS;
    }
    if (ticks > (int32_t) PWM_MAX_TICKS) {
        return PWM_MAX_TICKS;
    }
    return (uint32_t) ticks;
}

static void set_pwm(uint32_t left_duty_ticks, uint32_t right_duty_ticks)
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

static void motors_forward(void)
{
    DL_GPIO_clearPins(MOTOR_PORT, AIN1_PIN | AIN2_PIN | BIN1_PIN | BIN2_PIN);
    DL_GPIO_setPins(MOTOR_PORT, AIN1_PIN | BIN1_PIN | STBY_PIN);
}

static void balance_update(void)
{
    static int32_t prev_left_forward = 0;
    static int32_t prev_right_forward = 0;
    static int32_t left_sum_1s = 0;
    static int32_t right_sum_1s = 0;
    static uint32_t sample_count_1s = 0;

    int32_t left_forward = -g_left_count;
    int32_t right_forward = g_right_count;

    g_left_delta_edges = left_forward - prev_left_forward;
    g_right_delta_edges = right_forward - prev_right_forward;

    left_sum_1s += g_left_delta_edges;
    right_sum_1s += g_right_delta_edges;
    sample_count_1s++;

    if (sample_count_1s >= 10U) {
        g_debug_left_edges_1s = left_sum_1s;
        g_debug_right_edges_1s = right_sum_1s;
        g_debug_error_1s = right_sum_1s - left_sum_1s;
        g_debug_left_pwm_1s = g_left_pwm_ticks;
        g_debug_right_pwm_1s = g_right_pwm_ticks;

        left_sum_1s = 0;
        right_sum_1s = 0;
        sample_count_1s = 0;
    }

    prev_left_forward = left_forward;
    prev_right_forward = right_forward;

    /*
     * Positive error means right wheel is faster.
     * Increase left logical duty and decrease right logical duty.
     */
    g_balance_error = g_right_delta_edges - g_left_delta_edges;
    g_pwm_correction = g_balance_error / 4;

    if (g_pwm_correction > PWM_CORR_LIMIT) {
        g_pwm_correction = PWM_CORR_LIMIT;
    } else if (g_pwm_correction < -PWM_CORR_LIMIT) {
        g_pwm_correction = -PWM_CORR_LIMIT;
    }

    set_pwm(
        clamp_pwm((int32_t) PWM_BASE_TICKS + g_pwm_correction),
        clamp_pwm((int32_t) PWM_BASE_TICKS - g_pwm_correction));
}

int main(void)
{
    SYSCFG_DL_init();

    init_output(IOMUX_PINCM12, GPIOB, AIN1_PIN, false);
    init_output(IOMUX_PINCM13, GPIOB, AIN2_PIN, false);
    init_output(IOMUX_PINCM15, GPIOB, BIN1_PIN, false);
    init_output(IOMUX_PINCM16, GPIOB, BIN2_PIN, false);
    init_output(IOMUX_PINCM17, GPIOB, STBY_PIN, true);
    init_output(IOMUX_PINCM37, GPIOA, LED_PIN, false);

    encoder_init();
    pwm_init();
    motors_forward();
    set_pwm(PWM_BASE_TICKS, PWM_BASE_TICKS);

    while (1) {
        delay_ms(100);
        balance_update();
        DL_GPIO_togglePins(LED_PORT, LED_PIN);
    }
}

void GROUP1_IRQHandler(void)
{
    uint32_t status = DL_GPIO_getEnabledInterruptStatus(ENCODER_PORT, ENCODER_INT_MASK);

    if ((status & LEFT_A_PIN) != 0U) {
        uint32_t pins = DL_GPIO_readPins(ENCODER_PORT, LEFT_A_PIN | LEFT_B_PIN);
        bool a_high = ((pins & LEFT_A_PIN) != 0U);
        bool b_high = ((pins & LEFT_B_PIN) != 0U);

        if (a_high != b_high) {
            g_left_count++;
        } else {
            g_left_count--;
        }
        g_left_edges++;
    }

    if ((status & RIGHT_A_PIN) != 0U) {
        uint32_t pins = DL_GPIO_readPins(ENCODER_PORT, RIGHT_A_PIN | RIGHT_B_PIN);
        bool a_high = ((pins & RIGHT_A_PIN) != 0U);
        bool b_high = ((pins & RIGHT_B_PIN) != 0U);

        if (a_high != b_high) {
            g_right_count++;
        } else {
            g_right_count--;
        }
        g_right_edges++;
    }

    DL_GPIO_clearInterruptStatus(ENCODER_PORT, status);
}
