/**
 * Encoder + motor calibration - MSPM0G3507 LaunchPad
 *
 * Purpose:
 *   1. Confirm left/right encoder mapping.
 *   2. Confirm encoder count direction.
 *   3. Compare left/right motor counts under the same PWM duty.
 *
 * Sequence:
 *   Phase 1: left motor forward 3 s
 *   Phase 2: right motor forward 3 s
 *   Phase 3: both motors forward 3 s
 *   Then stop forever.
 *
 * Watch these CCS Expressions after the program stops:
 *   g_left_count, g_right_count, g_left_edges, g_right_edges
 *   g_left_phase_left_count, g_left_phase_right_count
 *   g_right_phase_left_count, g_right_phase_right_count
 *   g_both_phase_left_count, g_both_phase_right_count
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

#define PWM_TIMER        TIMG0
#define PWM_PERIOD       1600U
#define PWM_DUTY         960U   /* 60% */

#define ENCODER_PORT     GPIOB
#define LEFT_A_PIN       DL_GPIO_PIN_12
#define LEFT_B_PIN       DL_GPIO_PIN_13
#define RIGHT_A_PIN      DL_GPIO_PIN_14
#define RIGHT_B_PIN      DL_GPIO_PIN_15
#define ENCODER_INT_MASK (LEFT_A_PIN | RIGHT_A_PIN)

#define LED_PORT         GPIOA
#define LED_PIN          DL_GPIO_PIN_15

volatile int32_t g_left_count = 0;
volatile int32_t g_right_count = 0;
volatile uint32_t g_left_edges = 0;
volatile uint32_t g_right_edges = 0;

volatile uint8_t g_left_a_level = 0;
volatile uint8_t g_left_b_level = 0;
volatile uint8_t g_right_a_level = 0;
volatile uint8_t g_right_b_level = 0;

volatile uint8_t g_phase = 0;

volatile int32_t g_left_phase_left_count = 0;
volatile int32_t g_left_phase_right_count = 0;
volatile uint32_t g_left_phase_left_edges = 0;
volatile uint32_t g_left_phase_right_edges = 0;

volatile int32_t g_right_phase_left_count = 0;
volatile int32_t g_right_phase_right_count = 0;
volatile uint32_t g_right_phase_left_edges = 0;
volatile uint32_t g_right_phase_right_edges = 0;

volatile int32_t g_both_phase_left_count = 0;
volatile int32_t g_both_phase_right_count = 0;
volatile uint32_t g_both_phase_left_edges = 0;
volatile uint32_t g_both_phase_right_edges = 0;

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

static void update_encoder_levels(void)
{
    uint32_t pins = DL_GPIO_readPins(ENCODER_PORT,
        LEFT_A_PIN | LEFT_B_PIN | RIGHT_A_PIN | RIGHT_B_PIN);

    g_left_a_level = ((pins & LEFT_A_PIN) != 0U) ? 1U : 0U;
    g_left_b_level = ((pins & LEFT_B_PIN) != 0U) ? 1U : 0U;
    g_right_a_level = ((pins & RIGHT_A_PIN) != 0U) ? 1U : 0U;
    g_right_b_level = ((pins & RIGHT_B_PIN) != 0U) ? 1U : 0U;
}

static void encoder_init(void)
{
    init_input(IOMUX_PINCM29);  /* PB12 left A */
    init_input(IOMUX_PINCM30);  /* PB13 left B */
    init_input(IOMUX_PINCM31);  /* PB14 right A */
    init_input(IOMUX_PINCM32);  /* PB15 right B */

    update_encoder_levels();

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
    DL_TimerG_setCaptureCompareValue(PWM_TIMER, 0U, DL_TIMER_CC_0_INDEX);
    DL_TimerG_setCaptureCompareValue(PWM_TIMER, 0U, DL_TIMER_CC_1_INDEX);

    DL_TimerG_enableClock(PWM_TIMER);
    DL_TimerG_setCCPDirection(PWM_TIMER, DL_TIMER_CC0_OUTPUT | DL_TIMER_CC1_OUTPUT);
    DL_TimerG_startCounter(PWM_TIMER);
}

static void set_pwm(uint32_t left_ticks, uint32_t right_ticks)
{
    DL_TimerG_setCaptureCompareValue(PWM_TIMER, left_ticks, DL_TIMER_CC_0_INDEX);
    DL_TimerG_setCaptureCompareValue(PWM_TIMER, right_ticks, DL_TIMER_CC_1_INDEX);
}

static void motors_stop(void)
{
    set_pwm(0U, 0U);
    DL_GPIO_clearPins(MOTOR_PORT, AIN1_PIN | AIN2_PIN | BIN1_PIN | BIN2_PIN);
}

static void left_forward(void)
{
    DL_GPIO_clearPins(MOTOR_PORT, AIN1_PIN | AIN2_PIN | BIN1_PIN | BIN2_PIN);
    DL_GPIO_setPins(MOTOR_PORT, AIN1_PIN);
    set_pwm(PWM_DUTY, 0U);
}

static void right_forward(void)
{
    DL_GPIO_clearPins(MOTOR_PORT, AIN1_PIN | AIN2_PIN | BIN1_PIN | BIN2_PIN);
    DL_GPIO_setPins(MOTOR_PORT, BIN1_PIN);
    set_pwm(0U, PWM_DUTY);
}

static void both_forward(void)
{
    DL_GPIO_clearPins(MOTOR_PORT, AIN1_PIN | AIN2_PIN | BIN1_PIN | BIN2_PIN);
    DL_GPIO_setPins(MOTOR_PORT, AIN1_PIN | BIN1_PIN);
    set_pwm(PWM_DUTY, PWM_DUTY);
}

static void reset_counts(void)
{
    __disable_irq();
    g_left_count = 0;
    g_right_count = 0;
    g_left_edges = 0;
    g_right_edges = 0;
    DL_GPIO_clearInterruptStatus(ENCODER_PORT, ENCODER_INT_MASK);
    __enable_irq();
}

static void blink_phase(uint32_t times)
{
    for (uint32_t i = 0; i < times; i++) {
        DL_GPIO_setPins(LED_PORT, LED_PIN);
        delay_ms(80);
        DL_GPIO_clearPins(LED_PORT, LED_PIN);
        delay_ms(120);
    }
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

    g_phase = 1U;
    reset_counts();
    blink_phase(1U);
    left_forward();
    delay_ms(3000);
    motors_stop();
    g_left_phase_left_count = g_left_count;
    g_left_phase_right_count = g_right_count;
    g_left_phase_left_edges = g_left_edges;
    g_left_phase_right_edges = g_right_edges;
    delay_ms(1000);

    g_phase = 2U;
    reset_counts();
    blink_phase(2U);
    right_forward();
    delay_ms(3000);
    motors_stop();
    g_right_phase_left_count = g_left_count;
    g_right_phase_right_count = g_right_count;
    g_right_phase_left_edges = g_left_edges;
    g_right_phase_right_edges = g_right_edges;
    delay_ms(1000);

    g_phase = 3U;
    reset_counts();
    blink_phase(3U);
    both_forward();
    delay_ms(3000);
    motors_stop();
    g_both_phase_left_count = g_left_count;
    g_both_phase_right_count = g_right_count;
    g_both_phase_left_edges = g_left_edges;
    g_both_phase_right_edges = g_right_edges;

    g_phase = 4U;
    while (1) {
        DL_GPIO_setPins(LED_PORT, LED_PIN);
        delay_ms(50);
        DL_GPIO_clearPins(LED_PORT, LED_PIN);
        delay_ms(250);
    }
}

void GROUP1_IRQHandler(void)
{
    uint32_t status = DL_GPIO_getEnabledInterruptStatus(ENCODER_PORT, ENCODER_INT_MASK);

    if ((status & LEFT_A_PIN) != 0U) {
        uint32_t pins = DL_GPIO_readPins(ENCODER_PORT, LEFT_A_PIN | LEFT_B_PIN);
        bool a_high = ((pins & LEFT_A_PIN) != 0U);
        bool b_high = ((pins & LEFT_B_PIN) != 0U);
        g_left_a_level = a_high ? 1U : 0U;
        g_left_b_level = b_high ? 1U : 0U;

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
        g_right_a_level = a_high ? 1U : 0U;
        g_right_b_level = b_high ? 1U : 0U;

        if (a_high != b_high) {
            g_right_count++;
        } else {
            g_right_count--;
        }
        g_right_edges++;
    }

    DL_GPIO_clearInterruptStatus(ENCODER_PORT, status);
}
