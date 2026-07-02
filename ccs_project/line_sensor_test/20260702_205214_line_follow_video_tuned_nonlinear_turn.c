/**
 * Simple line-following with minimal right-angle assist.
 *
 * Strategy:
 *   Keep the pre-sharp-turn version's simple P controller for normal driving.
 *   Add only one narrow sharp-turn branch for clear right-angle/lost-line cases.
 *   No post-turn state machine, no straight-state lock, no cooldown window.
 *
 * Sensors, from car front left to right:
 *   S1 DO -> PB5
 *   S2 DO -> PB6
 *   S3 DO -> PB7
 *   S4 DO -> PA7
 *   S5 DO -> PA8
 *   S6 DO -> PB9
 *   S7 DO -> PB10
 *
 * Motor driver:
 *   PB0 -> AIN1
 *   PB1 -> AIN2
 *   PB2 -> BIN1
 *   PB3 -> BIN2
 *   PB4 -> STBY
 *   PA12 -> PWMA / TIMG0_CCP0
 *   PA13 -> PWMB / TIMG0_CCP1
 *
 * Control modes:
 *   0 = normal simple P
 *   1 = visible sharp turn assist
 *   2 = full-lost recovery by last direction
 *   4 = startup/search straight
 */

#include <stdbool.h>
#include <stdint.h>
#include "ti_msp_dl_config.h"

#define MOTOR_PORT              GPIOB
#define AIN1_PIN                DL_GPIO_PIN_0
#define AIN2_PIN                DL_GPIO_PIN_1
#define BIN1_PIN                DL_GPIO_PIN_2
#define BIN2_PIN                DL_GPIO_PIN_3
#define STBY_PIN                DL_GPIO_PIN_4

#define PWM_TIMER               TIMG0
#define PWM_PERIOD              1600U
#define PWM_LEFT_BASE_TICKS     620U
#define PWM_RIGHT_BASE_TICKS    590U
#define PWM_MIN_TICKS           430U
#define PWM_MAX_TICKS           850U
#define LINE_KP_DIVISOR         20
#define LINE_CORR_LIMIT         150
#define LINE_MEDIUM_ERROR       1500
#define LINE_MEDIUM_KP_DIVISOR  12
#define LINE_MEDIUM_CORR_LIMIT  260
#define LINE_SHARP_ERROR        1800
#define LINE_SHARP_KP_DIVISOR   7
#define LINE_SHARP_CORR_LIMIT   430
#define SHARP_LEFT_BASE_TICKS   580U
#define SHARP_RIGHT_BASE_TICKS  552U
#define SHARP_MAX_TICKS         780U
#define SHARP_INNER_FLOOR_TICKS 160U
#define SEARCH_LEFT_BASE_TICKS  560U
#define SEARCH_RIGHT_BASE_TICKS 533U

#define RUN_SAMPLES             4500U

#define LED_PORT                GPIOA
#define LED_PIN                 DL_GPIO_PIN_15

#define S1_PIN                  DL_GPIO_PIN_5
#define S2_PIN                  DL_GPIO_PIN_6
#define S3_PIN                  DL_GPIO_PIN_7
#define S4_PIN                  DL_GPIO_PIN_7
#define S5_PIN                  DL_GPIO_PIN_8
#define S6_PIN                  DL_GPIO_PIN_9
#define S7_PIN                  DL_GPIO_PIN_10

#define LINE_BLACK_IS_HIGH      1

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
volatile int32_t g_line_correction = 0;
volatile uint8_t g_turn_mode = 0;

volatile uint32_t g_left_pwm_ticks = PWM_LEFT_BASE_TICKS;
volatile uint32_t g_right_pwm_ticks = PWM_RIGHT_BASE_TICKS;
volatile uint32_t g_left_compare_ticks = PWM_PERIOD - PWM_LEFT_BASE_TICKS;
volatile uint32_t g_right_compare_ticks = PWM_PERIOD - PWM_RIGHT_BASE_TICKS;

volatile uint32_t g_run_sample = 0;
volatile uint8_t g_phase = 0;
volatile uint8_t g_stop_reason = 0;

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

static uint8_t normalize_sensor(uint8_t raw)
{
#if LINE_BLACK_IS_HIGH
    return raw;
#else
    return (raw == 0U) ? 1U : 0U;
#endif
}

static int32_t clamp_int32(int32_t value, int32_t min_value, int32_t max_value)
{
    if (value < min_value) {
        return min_value;
    }
    if (value > max_value) {
        return max_value;
    }
    return value;
}

static int32_t abs_int32(int32_t value)
{
    return (value < 0) ? -value : value;
}

static uint32_t clamp_pwm(int32_t ticks)
{
    if (ticks <= 0) {
        return 0U;
    }
    if (ticks < (int32_t) PWM_MIN_TICKS) {
        return PWM_MIN_TICKS;
    }
    if (ticks > (int32_t) PWM_MAX_TICKS) {
        return PWM_MAX_TICKS;
    }
    return (uint32_t) ticks;
}

static uint32_t clamp_sharp_pwm(int32_t ticks)
{
    if (ticks <= 0) {
        return 0U;
    }
    if (ticks > (int32_t) SHARP_MAX_TICKS) {
        return SHARP_MAX_TICKS;
    }
    return (uint32_t) ticks;
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

static void set_pwm(uint32_t left_duty_ticks, uint32_t right_duty_ticks)
{
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

static void motors_stop(void)
{
    set_pwm(0U, 0U);
    DL_GPIO_clearPins(MOTOR_PORT, AIN1_PIN | AIN2_PIN | BIN1_PIN | BIN2_PIN);
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
    g_line_lost_count = 0U;
    g_line_error = (int16_t)(sum / (int32_t) count);
    g_line_last_error = g_line_error;
}

static void line_follow_update(void)
{
    int32_t error = (int32_t) g_line_error;
    bool line_visible = (g_line_valid != 0U);
    bool lost_recovery = (!line_visible) && (g_line_last_error != 0);
    bool visible_sharp_turn = line_visible &&
        ((abs_int32(error) >= LINE_SHARP_ERROR) ||
        (g_s1 != 0U) || (g_s7 != 0U));
    bool medium_turn = line_visible &&
        (abs_int32(error) >= LINE_MEDIUM_ERROR);

    if (!line_visible && !lost_recovery) {
        g_turn_mode = 4U;
        g_line_correction = 0;
        set_pwm(
            clamp_pwm((int32_t) SEARCH_LEFT_BASE_TICKS),
            clamp_pwm((int32_t) SEARCH_RIGHT_BASE_TICKS));
        return;
    }

    if (lost_recovery || visible_sharp_turn) {
        g_turn_mode = lost_recovery ? 2U : 1U;

        g_line_correction = clamp_int32(
            error / LINE_SHARP_KP_DIVISOR,
            -LINE_SHARP_CORR_LIMIT,
            LINE_SHARP_CORR_LIMIT);

        int32_t left_target = (int32_t) SHARP_LEFT_BASE_TICKS - g_line_correction;
        int32_t right_target = (int32_t) SHARP_RIGHT_BASE_TICKS + g_line_correction;

        if ((g_line_correction >= 0) &&
            (left_target < (int32_t) SHARP_INNER_FLOOR_TICKS)) {
            left_target = (int32_t) SHARP_INNER_FLOOR_TICKS;
        } else if ((g_line_correction < 0) &&
            (right_target < (int32_t) SHARP_INNER_FLOOR_TICKS)) {
            right_target = (int32_t) SHARP_INNER_FLOOR_TICKS;
        }

        set_pwm(
            clamp_sharp_pwm(left_target),
            clamp_sharp_pwm(right_target));
        return;
    }

    if (medium_turn) {
        g_turn_mode = 3U;

        g_line_correction = clamp_int32(
            error / LINE_MEDIUM_KP_DIVISOR,
            -LINE_MEDIUM_CORR_LIMIT,
            LINE_MEDIUM_CORR_LIMIT);

        set_pwm(
            clamp_pwm((int32_t) PWM_LEFT_BASE_TICKS - g_line_correction),
            clamp_pwm((int32_t) PWM_RIGHT_BASE_TICKS + g_line_correction));
        return;
    }

    g_turn_mode = 0U;
    g_line_correction = clamp_int32(
        error / LINE_KP_DIVISOR,
        -LINE_CORR_LIMIT,
        LINE_CORR_LIMIT);

    set_pwm(
        clamp_pwm((int32_t) PWM_LEFT_BASE_TICKS - g_line_correction),
        clamp_pwm((int32_t) PWM_RIGHT_BASE_TICKS + g_line_correction));
}

static void blink(uint32_t times, uint32_t on_ms, uint32_t off_ms)
{
    for (uint32_t i = 0; i < times; i++) {
        DL_GPIO_setPins(LED_PORT, LED_PIN);
        delay_ms(on_ms);
        DL_GPIO_clearPins(LED_PORT, LED_PIN);
        delay_ms(off_ms);
    }
}

int main(void)
{
    SYSCFG_DL_init();

    init_output(IOMUX_PINCM12, MOTOR_PORT, AIN1_PIN, false);
    init_output(IOMUX_PINCM13, MOTOR_PORT, AIN2_PIN, false);
    init_output(IOMUX_PINCM15, MOTOR_PORT, BIN1_PIN, false);
    init_output(IOMUX_PINCM16, MOTOR_PORT, BIN2_PIN, false);
    init_output(IOMUX_PINCM17, MOTOR_PORT, STBY_PIN, true);
    init_output(IOMUX_PINCM37, LED_PORT, LED_PIN, false);

    init_input(IOMUX_PINCM18);
    init_input(IOMUX_PINCM23);
    init_input(IOMUX_PINCM24);
    init_input(IOMUX_PINCM14);
    init_input(IOMUX_PINCM19);
    init_input(IOMUX_PINCM26);
    init_input(IOMUX_PINCM27);

    pwm_init();
    motors_stop();

    g_phase = 1U;
    blink(5U, 50U, 150U);

    read_line_sensors();
    update_line_position();

    g_phase = 2U;
    motors_forward();

    for (g_run_sample = 0U; g_run_sample < RUN_SAMPLES; g_run_sample++) {
        read_line_sensors();
        update_line_position();

        motors_forward();
        line_follow_update();
        delay_ms(20);
    }

    if (g_phase == 2U) {
        g_phase = 3U;
        g_stop_reason = 1U;
    }

    motors_stop();

    while (1) {
        blink(1U, 50U, 250U);
    }
}
