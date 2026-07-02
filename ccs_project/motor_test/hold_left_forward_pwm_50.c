/**
 * Hold left motor forward with 50% PWM - MSPM0G3507 LaunchPad
 *
 * Static diagnostic state:
 *   STBY = HIGH
 *   AIN1 = HIGH
 *   AIN2 = LOW
 *   PA12/TIMG0_CCP0 -> PWMA = 20 kHz, 50% duty
 *
 * Expected:
 *   PWMA-GND reads about 1.6 V on a DC multimeter.
 *   AO1-AO2 reads about half VM on a DC multimeter, and the left motor spins.
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
#define PWM_50_PERCENT   800U

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

static void pwm_init_50_percent(void)
{
    DL_TimerG_reset(PWM_TIMER);
    DL_TimerG_enablePower(PWM_TIMER);
    delay_cycles(16);

    DL_GPIO_initPeripheralOutputFunction(IOMUX_PINCM34, IOMUX_PINCM34_PF_TIMG0_CCP0);  /* PA12 */
    DL_GPIO_enableOutput(GPIOA, DL_GPIO_PIN_12);

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
    DL_TimerG_setCaptCompUpdateMethod(PWM_TIMER,
        DL_TIMER_CC_UPDATE_METHOD_IMMEDIATE,
        DL_TIMERG_CAPTURE_COMPARE_0_INDEX);
    DL_TimerG_setCaptureCompareValue(PWM_TIMER, PWM_50_PERCENT, DL_TIMER_CC_0_INDEX);

    DL_TimerG_enableClock(PWM_TIMER);
    DL_TimerG_setCCPDirection(PWM_TIMER, DL_TIMER_CC0_OUTPUT);
    DL_TimerG_startCounter(PWM_TIMER);
}

int main(void)
{
    SYSCFG_DL_init();

    init_output(IOMUX_PINCM12, GPIOB, AIN1_PIN, true);   /* PB0 -> AIN1 */
    init_output(IOMUX_PINCM13, GPIOB, AIN2_PIN, false);  /* PB1 -> AIN2 */
    init_output(IOMUX_PINCM15, GPIOB, BIN1_PIN, false);  /* PB2 -> BIN1 */
    init_output(IOMUX_PINCM16, GPIOB, BIN2_PIN, false);  /* PB3 -> BIN2 */
    init_output(IOMUX_PINCM17, GPIOB, STBY_PIN, true);   /* PB4 -> STBY */
    init_output(IOMUX_PINCM37, GPIOA, LED_PIN, false);   /* PA15 */

    pwm_init_50_percent();

    while (1) {
        DL_GPIO_setPins(LED_PORT, LED_PIN);
        delay_ms(100);
        DL_GPIO_clearPins(LED_PORT, LED_PIN);
        delay_ms(900);
    }
}
