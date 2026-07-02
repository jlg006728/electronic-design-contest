/**
 * Encoder test while motors rotate slowly - MSPM0G3507 LaunchPad
 *
 * Motor wiring:
 *   PB0  -> AIN1
 *   PB1  -> AIN2
 *   PB2  -> BIN1
 *   PB3  -> BIN2
 *   PB4  -> STBY
 *   PA12 -> PWMA
 *   PA13 -> PWMB
 *
 * Encoder wiring:
 *   E1A -> PB12
 *   E1B -> PB13
 *   E2A -> PB14
 *   E2B -> PB15
 *   GND -> GND
 *
 * Add these CCS Expressions:
 *   g_left_count, g_right_count, g_left_edges, g_right_edges,
 *   g_left_a_level, g_left_b_level, g_right_a_level, g_right_b_level
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
#define PWM_DUTY         1120U  /* 70%, easier startup */

#define ENCODER_PORT     GPIOB
#define LEFT_A_PIN       DL_GPIO_PIN_12
#define LEFT_B_PIN       DL_GPIO_PIN_13
#define RIGHT_A_PIN      DL_GPIO_PIN_14
#define RIGHT_B_PIN      DL_GPIO_PIN_15

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

static void pwm_init(void)
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
    DL_TimerG_setCaptureCompareValue(PWM_TIMER, PWM_DUTY, DL_TIMER_CC_0_INDEX);
    DL_TimerG_setCaptureCompareValue(PWM_TIMER, PWM_DUTY, DL_TIMER_CC_1_INDEX);

    DL_TimerG_enableClock(PWM_TIMER);
    DL_TimerG_setCCPDirection(PWM_TIMER, DL_TIMER_CC0_OUTPUT | DL_TIMER_CC1_OUTPUT);
    DL_TimerG_startCounter(PWM_TIMER);
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

static void poll_encoder_edges(void)
{
    static uint8_t last_left_a = 0U;
    static uint8_t last_right_a = 0U;
    static bool initialized = false;

    update_encoder_levels();

    if (!initialized) {
        last_left_a = g_left_a_level;
        last_right_a = g_right_a_level;
        initialized = true;
        return;
    }

    if (g_left_a_level != last_left_a) {
        last_left_a = g_left_a_level;
        if (g_left_a_level != g_left_b_level) {
            g_left_count++;
        } else {
            g_left_count--;
        }
        g_left_edges++;
    }

    if (g_right_a_level != last_right_a) {
        last_right_a = g_right_a_level;
        if (g_right_a_level != g_right_b_level) {
            g_right_count++;
        } else {
            g_right_count--;
        }
        g_right_edges++;
    }
}

int main(void)
{
    SYSCFG_DL_init();

    init_output(IOMUX_PINCM12, GPIOB, AIN1_PIN, true);
    init_output(IOMUX_PINCM13, GPIOB, AIN2_PIN, false);
    init_output(IOMUX_PINCM15, GPIOB, BIN1_PIN, true);
    init_output(IOMUX_PINCM16, GPIOB, BIN2_PIN, false);
    init_output(IOMUX_PINCM17, GPIOB, STBY_PIN, true);
    init_output(IOMUX_PINCM37, GPIOA, LED_PIN, false);

    init_input(IOMUX_PINCM29);  /* PB12 */
    init_input(IOMUX_PINCM30);  /* PB13 */
    init_input(IOMUX_PINCM31);  /* PB14 */
    init_input(IOMUX_PINCM32);  /* PB15 */

    pwm_init();

    while (1) {
        poll_encoder_edges();
        if (((g_left_edges + g_right_edges) & 0x3FU) == 0U) {
            DL_GPIO_togglePins(LED_PORT, LED_PIN);
        }
    }
}
