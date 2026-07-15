/**
 * V37 slower motor-enabled line follow with reduced bridge right bias.
 *
 * Goal:
 *   Re-enable motors while preserving the confirmed raw sensor standard:
 *   white surface is high/1, black line is low/0.
 *
 * This version tests only:
 *   - 7 digital line sensors, raw white-high standard
 *   - TB6612 motor output
 *   - A-start no-line bridge
 *   - line following using the inverted black-line mask
 *   - LED/buzzer state feedback
 *
 * Pin map:
 *   S1 DO -> PB5
 *   S2 DO -> PB6
 *   S3 DO -> PB7
 *   S4 DO -> PA7
 *   S5 DO -> PA8
 *   S6 DO -> PB9
 *   S7 DO -> PB10
 *
 *   PB0 -> AIN1
 *   PB1 -> AIN2
 *   PB2 -> BIN1
 *   PB3 -> BIN2
 *   PB4 -> STBY
 *   PA12 -> PWMA / TIMG0_CCP0
 *   PA13 -> PWMB / TIMG0_CCP1
 *
 *   PA15 -> LED
 *   PB11 -> active buzzer
 */

#include <stdbool.h>
#include <stdint.h>
#include "ti_msp_dl_config.h"

#define MOTOR_PORT                  GPIOB
#define AIN1_PIN                    DL_GPIO_PIN_0
#define AIN2_PIN                    DL_GPIO_PIN_1
#define BIN1_PIN                    DL_GPIO_PIN_2
#define BIN2_PIN                    DL_GPIO_PIN_3
#define STBY_PIN                    DL_GPIO_PIN_4

#define PWM_TIMER                   TIMG0
#define PWM_PERIOD                  1600U
#define PWM_LEFT_BASE_TICKS         500U
#define PWM_RIGHT_BASE_TICKS        485U
#define PWM_MIN_TICKS               350U
#define PWM_MAX_TICKS               700U

#define START_GAP_LEFT_TICKS        520U
#define START_GAP_RIGHT_TICKS       485U
#define START_GAP_MAX_SAMPLES       900U

#define LINE_KP_DIVISOR             20
#define LINE_CORR_LIMIT             150
#define LINE_SHARP_ERROR            1800
#define LINE_SHARP_KP_DIVISOR       8
#define LINE_SHARP_CORR_LIMIT       380
#define SENSOR_HIGH_IS_WHITE        1
#define LINE_SAMPLE_COUNT           9U
#define LINE_WHITE_MIN_HIGH_SAMPLES 5U
#define LINE_SAMPLE_DELAY_CYCLES    120U

#define CONTROL_DELAY_MS            20U
#define TRANSITION_HOLD_MS          2000U
#define RUN_SAMPLES                 9000U
#define LOST_STOP_SAMPLES           120U

#define LED_PORT                    GPIOA
#define LED_PIN                     DL_GPIO_PIN_15

#define BUZZER_PORT                 GPIOB
#define BUZZER_PIN                  DL_GPIO_PIN_11

#define S1_PIN                      DL_GPIO_PIN_5
#define S2_PIN                      DL_GPIO_PIN_6
#define S3_PIN                      DL_GPIO_PIN_7
#define S4_PIN                      DL_GPIO_PIN_7
#define S5_PIN                      DL_GPIO_PIN_8
#define S6_PIN                      DL_GPIO_PIN_9
#define S7_PIN                      DL_GPIO_PIN_10

#define PHASE_STARTUP               1U
#define PHASE_LINE_RUN              2U
#define PHASE_DONE                  3U
#define PHASE_STOP                  4U
#define PHASE_START_GAP_BRIDGE      8U

#define STOP_REASON_NONE            0U
#define STOP_REASON_RUN_TIMEOUT     1U
#define STOP_REASON_LINE_LOST       2U
#define STOP_REASON_START_GAP       6U

#define HOLD_REASON_BRIDGE          1U
#define HOLD_REASON_REACQUIRE       2U
#define HOLD_REASON_LINE_START      3U

volatile uint8_t g_s1 = 0;
volatile uint8_t g_s2 = 0;
volatile uint8_t g_s3 = 0;
volatile uint8_t g_s4 = 0;
volatile uint8_t g_s5 = 0;
volatile uint8_t g_s6 = 0;
volatile uint8_t g_s7 = 0;

volatile uint8_t g_line_raw_mask = 0;
volatile uint8_t g_black_line_mask = 0;
volatile uint8_t g_line_active_count = 0;
volatile uint8_t g_line_valid = 0;
volatile int16_t g_line_error = 0;
volatile int16_t g_line_last_error = 0;
volatile int32_t g_line_weighted_sum = 0;
volatile uint32_t g_line_lost_count = 0;
volatile int32_t g_line_correction = 0;
volatile uint8_t g_turn_mode = 0;
volatile uint32_t g_line_sample_call_count = 0;
volatile uint8_t g_line_raw_high_last_mask = 0;
volatile uint8_t g_line_raw_high_or_mask = 0;
volatile uint8_t g_line_raw_high_and_mask = 0;

volatile uint8_t g_s1_high_count = 0;
volatile uint8_t g_s2_high_count = 0;
volatile uint8_t g_s3_high_count = 0;
volatile uint8_t g_s4_high_count = 0;
volatile uint8_t g_s5_high_count = 0;
volatile uint8_t g_s6_high_count = 0;
volatile uint8_t g_s7_high_count = 0;

volatile uint32_t g_left_pwm_ticks = PWM_LEFT_BASE_TICKS;
volatile uint32_t g_right_pwm_ticks = PWM_RIGHT_BASE_TICKS;
volatile uint32_t g_left_compare_ticks = PWM_PERIOD - PWM_LEFT_BASE_TICKS;
volatile uint32_t g_right_compare_ticks = PWM_PERIOD - PWM_RIGHT_BASE_TICKS;

volatile uint32_t g_run_sample = 0;
volatile uint32_t g_start_gap_samples = 0;
volatile uint32_t g_reacquire_event_count = 0;
volatile uint8_t g_phase = 0;
volatile uint8_t g_stop_reason = STOP_REASON_NONE;
volatile uint8_t g_state_switch_reason = 0;
volatile uint32_t g_state_switch_event_count = 0;

volatile uint8_t g_startup_line_valid_snapshot = 0;
volatile uint8_t g_startup_line_raw_mask_snapshot = 0;
volatile uint8_t g_startup_line_active_count_snapshot = 0;
volatile int16_t g_startup_line_error_snapshot = 0;

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

static void led_set(bool on)
{
    if (on) {
        DL_GPIO_setPins(LED_PORT, LED_PIN);
    } else {
        DL_GPIO_clearPins(LED_PORT, LED_PIN);
    }
}

static void buzzer_set(bool on)
{
    if (on) {
        DL_GPIO_setPins(BUZZER_PORT, BUZZER_PIN);
    } else {
        DL_GPIO_clearPins(BUZZER_PORT, BUZZER_PIN);
    }
}

static void buzzer_beep(uint8_t count)
{
    for (uint8_t i = 0; i < count; i++) {
        buzzer_set(true);
        delay_ms(90U);
        buzzer_set(false);
        delay_ms(110U);
    }
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

static uint32_t clamp_pwm_allow_slow_inner(int32_t ticks)
{
    if (ticks <= 0) {
        return 0U;
    }
    if (ticks > (int32_t) PWM_MAX_TICKS) {
        return PWM_MAX_TICKS;
    }
    return (uint32_t) ticks;
}

static void pwm_init(void)
{
    DL_TimerG_reset(PWM_TIMER);
    DL_TimerG_enablePower(PWM_TIMER);
    delay_cycles(16);

    DL_GPIO_initPeripheralOutputFunction(
        IOMUX_PINCM34, IOMUX_PINCM34_PF_TIMG0_CCP0);
    DL_GPIO_enableOutput(GPIOA, DL_GPIO_PIN_12);
    DL_GPIO_initPeripheralOutputFunction(
        IOMUX_PINCM35, IOMUX_PINCM35_PF_TIMG0_CCP1);
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
    DL_TimerG_setCCPDirection(
        PWM_TIMER, DL_TIMER_CC0_OUTPUT | DL_TIMER_CC1_OUTPUT);
    DL_TimerG_startCounter(PWM_TIMER);
}

static void set_pwm(uint32_t left_duty_ticks, uint32_t right_duty_ticks)
{
    g_left_pwm_ticks = left_duty_ticks;
    g_right_pwm_ticks = right_duty_ticks;
    g_left_compare_ticks = PWM_PERIOD - left_duty_ticks;
    g_right_compare_ticks = PWM_PERIOD - right_duty_ticks;

    DL_TimerG_setCaptureCompareValue(
        PWM_TIMER, g_left_compare_ticks, DL_TIMER_CC_0_INDEX);
    DL_TimerG_setCaptureCompareValue(
        PWM_TIMER, g_right_compare_ticks, DL_TIMER_CC_1_INDEX);
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

static void transition_hold(uint8_t reason, uint8_t beep_count)
{
    g_state_switch_reason = reason;
    g_state_switch_event_count++;

    motors_stop();
    buzzer_beep(beep_count);

    for (uint32_t elapsed = 0U; elapsed < TRANSITION_HOLD_MS; elapsed += 100U) {
        led_set(true);
        delay_ms(50U);
        led_set(false);
        delay_ms(50U);
    }
}

static uint8_t read_line_raw_high_mask_once(void)
{
    uint32_t a = DL_GPIO_readPins(GPIOA, S4_PIN | S5_PIN);
    uint32_t b =
        DL_GPIO_readPins(GPIOB, S1_PIN | S2_PIN | S3_PIN | S6_PIN | S7_PIN);
    uint8_t mask = 0U;

    mask |= (uint8_t) ((((b & S1_PIN) != 0U) ? 1U : 0U) << 0);
    mask |= (uint8_t) ((((b & S2_PIN) != 0U) ? 1U : 0U) << 1);
    mask |= (uint8_t) ((((b & S3_PIN) != 0U) ? 1U : 0U) << 2);
    mask |= (uint8_t) ((((a & S4_PIN) != 0U) ? 1U : 0U) << 3);
    mask |= (uint8_t) ((((a & S5_PIN) != 0U) ? 1U : 0U) << 4);
    mask |= (uint8_t) ((((b & S6_PIN) != 0U) ? 1U : 0U) << 5);
    mask |= (uint8_t) ((((b & S7_PIN) != 0U) ? 1U : 0U) << 6);

    return mask;
}

static uint8_t high_count_to_white(uint8_t high_count)
{
#if SENSOR_HIGH_IS_WHITE
    return (high_count >= LINE_WHITE_MIN_HIGH_SAMPLES) ? 1U : 0U;
#else
    return (high_count <= (LINE_SAMPLE_COUNT - LINE_WHITE_MIN_HIGH_SAMPLES)) ? 1U : 0U;
#endif
}

static void read_line_sensors(void)
{
    uint8_t high_last = 0U;
    uint8_t high_or = 0U;
    uint8_t high_and = 0x7FU;
    uint8_t c1 = 0U;
    uint8_t c2 = 0U;
    uint8_t c3 = 0U;
    uint8_t c4 = 0U;
    uint8_t c5 = 0U;
    uint8_t c6 = 0U;
    uint8_t c7 = 0U;

    for (uint8_t i = 0U; i < LINE_SAMPLE_COUNT; i++) {
        high_last = read_line_raw_high_mask_once();
        high_or |= high_last;
        high_and &= high_last;

        c1 += (uint8_t) ((high_last >> 0) & 0x01U);
        c2 += (uint8_t) ((high_last >> 1) & 0x01U);
        c3 += (uint8_t) ((high_last >> 2) & 0x01U);
        c4 += (uint8_t) ((high_last >> 3) & 0x01U);
        c5 += (uint8_t) ((high_last >> 4) & 0x01U);
        c6 += (uint8_t) ((high_last >> 5) & 0x01U);
        c7 += (uint8_t) ((high_last >> 6) & 0x01U);

        if (i + 1U < LINE_SAMPLE_COUNT) {
            delay_cycles(LINE_SAMPLE_DELAY_CYCLES);
        }
    }

    g_line_sample_call_count++;
    g_line_raw_high_last_mask = high_last;
    g_line_raw_high_or_mask = high_or;
    g_line_raw_high_and_mask = high_and;

    g_s1_high_count = c1;
    g_s2_high_count = c2;
    g_s3_high_count = c3;
    g_s4_high_count = c4;
    g_s5_high_count = c5;
    g_s6_high_count = c6;
    g_s7_high_count = c7;

    g_s1 = high_count_to_white(c1);
    g_s2 = high_count_to_white(c2);
    g_s3 = high_count_to_white(c3);
    g_s4 = high_count_to_white(c4);
    g_s5 = high_count_to_white(c5);
    g_s6 = high_count_to_white(c6);
    g_s7 = high_count_to_white(c7);

    g_line_raw_mask = 0U;
    g_line_raw_mask |= (uint8_t) (g_s1 << 0);
    g_line_raw_mask |= (uint8_t) (g_s2 << 1);
    g_line_raw_mask |= (uint8_t) (g_s3 << 2);
    g_line_raw_mask |= (uint8_t) (g_s4 << 3);
    g_line_raw_mask |= (uint8_t) (g_s5 << 4);
    g_line_raw_mask |= (uint8_t) (g_s6 << 5);
    g_line_raw_mask |= (uint8_t) (g_s7 << 6);
    g_black_line_mask = (uint8_t) ((~g_line_raw_mask) & 0x7FU);
}

static void update_line_position(void)
{
    int32_t sum = 0;
    uint8_t count = 0;

    if ((g_black_line_mask & (1U << 0)) != 0U) { sum += -3000; count++; }
    if ((g_black_line_mask & (1U << 1)) != 0U) { sum += -2000; count++; }
    if ((g_black_line_mask & (1U << 2)) != 0U) { sum += -1000; count++; }
    if ((g_black_line_mask & (1U << 3)) != 0U) { sum += 0;     count++; }
    if ((g_black_line_mask & (1U << 4)) != 0U) { sum += 1000;  count++; }
    if ((g_black_line_mask & (1U << 5)) != 0U) { sum += 2000;  count++; }
    if ((g_black_line_mask & (1U << 6)) != 0U) { sum += 3000;  count++; }

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
    g_line_error = (int16_t) (sum / (int32_t) count);
    g_line_last_error = g_line_error;
}

static void line_follow_update(void)
{
    int32_t error = (int32_t) g_line_error;
    bool lost_recovery = (g_line_valid == 0U) && (g_line_last_error != 0);
    bool sharp_turn = (abs_int32(error) >= LINE_SHARP_ERROR) || lost_recovery;

    if (lost_recovery) {
        g_turn_mode = 2U;
    } else if (sharp_turn) {
        g_turn_mode = 1U;
    } else {
        g_turn_mode = 0U;
    }

    if (sharp_turn) {
        g_line_correction = clamp_int32(
            error / LINE_SHARP_KP_DIVISOR,
            -LINE_SHARP_CORR_LIMIT,
            LINE_SHARP_CORR_LIMIT);
        set_pwm(
            clamp_pwm_allow_slow_inner(
                (int32_t) PWM_LEFT_BASE_TICKS - g_line_correction),
            clamp_pwm_allow_slow_inner(
                (int32_t) PWM_RIGHT_BASE_TICKS + g_line_correction));
        return;
    }

    g_line_correction = clamp_int32(
        error / LINE_KP_DIVISOR,
        -LINE_CORR_LIMIT,
        LINE_CORR_LIMIT);
    set_pwm(
        clamp_pwm((int32_t) PWM_LEFT_BASE_TICKS - g_line_correction),
        clamp_pwm((int32_t) PWM_RIGHT_BASE_TICKS + g_line_correction));
}

static void stop_with_reason(uint8_t reason)
{
    g_phase = PHASE_STOP;
    g_stop_reason = reason;
    motors_stop();
    buzzer_set(true);
    delay_ms(700U);
    buzzer_set(false);
}

static void hardware_init(void)
{
    SYSCFG_DL_init();

    init_output(IOMUX_PINCM12, MOTOR_PORT, AIN1_PIN, false);
    init_output(IOMUX_PINCM13, MOTOR_PORT, AIN2_PIN, false);
    init_output(IOMUX_PINCM15, MOTOR_PORT, BIN1_PIN, false);
    init_output(IOMUX_PINCM16, MOTOR_PORT, BIN2_PIN, false);
    init_output(IOMUX_PINCM17, MOTOR_PORT, STBY_PIN, true);
    init_output(IOMUX_PINCM37, LED_PORT, LED_PIN, false);
    init_output(IOMUX_PINCM28, BUZZER_PORT, BUZZER_PIN, false);

    init_input(IOMUX_PINCM18);
    init_input(IOMUX_PINCM23);
    init_input(IOMUX_PINCM24);
    init_input(IOMUX_PINCM14);
    init_input(IOMUX_PINCM19);
    init_input(IOMUX_PINCM26);
    init_input(IOMUX_PINCM27);

    pwm_init();
    motors_stop();
}

int main(void)
{
    hardware_init();

    g_phase = PHASE_STARTUP;
    buzzer_beep(1U);

    read_line_sensors();
    update_line_position();
    g_startup_line_valid_snapshot = g_line_valid;
    g_startup_line_raw_mask_snapshot = g_line_raw_mask;
    g_startup_line_active_count_snapshot = g_line_active_count;
    g_startup_line_error_snapshot = g_line_error;

    if (g_line_valid == 0U) {
        g_phase = PHASE_START_GAP_BRIDGE;
        transition_hold(HOLD_REASON_BRIDGE, 2U);
        set_pwm(START_GAP_LEFT_TICKS, START_GAP_RIGHT_TICKS);
        motors_forward();

        while (g_phase == PHASE_START_GAP_BRIDGE) {
            read_line_sensors();
            update_line_position();

            if (g_line_valid != 0U) {
                g_reacquire_event_count++;
                g_phase = PHASE_LINE_RUN;
                transition_hold(HOLD_REASON_REACQUIRE, 3U);
                line_follow_update();
                motors_forward();
                break;
            }

            if (g_start_gap_samples >= START_GAP_MAX_SAMPLES) {
                stop_with_reason(STOP_REASON_START_GAP);
                break;
            }

            g_start_gap_samples++;
            set_pwm(START_GAP_LEFT_TICKS, START_GAP_RIGHT_TICKS);
            motors_forward();
            delay_ms(CONTROL_DELAY_MS);
        }
    } else {
        g_phase = PHASE_LINE_RUN;
        transition_hold(HOLD_REASON_LINE_START, 4U);
        line_follow_update();
        motors_forward();
    }

    while (g_phase == PHASE_LINE_RUN) {
        read_line_sensors();
        update_line_position();

        if (g_line_lost_count >= LOST_STOP_SAMPLES) {
            stop_with_reason(STOP_REASON_LINE_LOST);
            break;
        }

        line_follow_update();
        motors_forward();

        g_run_sample++;
        if (g_run_sample >= RUN_SAMPLES) {
            g_phase = PHASE_DONE;
            g_stop_reason = STOP_REASON_RUN_TIMEOUT;
            motors_stop();
            break;
        }

        delay_ms(CONTROL_DELAY_MS);
    }

    motors_stop();
    while (1) {
        led_set(true);
        delay_ms(80U);
        led_set(false);
        delay_ms(420U);
    }
}
