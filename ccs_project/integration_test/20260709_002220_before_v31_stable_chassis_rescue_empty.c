/*
 * V30 reference-style chassis rewrite for MSPM0G3507.
 *
 * Goal:
 *   Rebuild the chassis controller around the ideas used by the 25E
 *   reference code: periodic sensor update, encoder distance state, task
 *   phases, and differential motor commands.
 *
 * Kept interfaces:
 *   TB6612: PB0/PB1/PB2/PB3 direction, PB4 STBY, PA12/PA13 PWM.
 *   Line sensors: S1..S7 -> PB5/PB6/PB7/PA7/PA8/PB9/PB10.
 *   Encoders: left PB12/PB13, right PB14/PB15.
 *   Buzzer: PB11. LED: PA15.
 *
 * OpenMV and servos are intentionally not controlled in this chassis-only
 * rewrite. Their pins are left untouched.
 */

#include <stdbool.h>
#include <stdint.h>
#include "ti_msp_dl_config.h"

#define NOINLINE                    __attribute__((noinline))

#define SYSCLK_HZ                   32000000U

#define LED_PORT                    GPIOA
#define LED_PIN                     DL_GPIO_PIN_15

#define BUZZER_PORT                 GPIOB
#define BUZZER_PIN                  DL_GPIO_PIN_11

#define MOTOR_PORT                  GPIOB
#define AIN1_PIN                    DL_GPIO_PIN_0
#define AIN2_PIN                    DL_GPIO_PIN_1
#define BIN1_PIN                    DL_GPIO_PIN_2
#define BIN2_PIN                    DL_GPIO_PIN_3
#define STBY_PIN                    DL_GPIO_PIN_4

#define PWM_TIMER                   TIMG0
#define PWM_PERIOD                  1600U
#define PWM_LEFT_BASE_TICKS         520U
#define PWM_RIGHT_BASE_TICKS        500U
#define PWM_BRIDGE_LEFT_TICKS       500U
#define PWM_BRIDGE_RIGHT_TICKS      520U
#define PWM_MIN_TICKS               330U
#define PWM_MAX_TICKS               760U
#define PWM_FORCE_BASE_TICKS        470U
#define PWM_FORCE_CORR_TICKS        330

#define S1_PIN                      DL_GPIO_PIN_5
#define S2_PIN                      DL_GPIO_PIN_6
#define S3_PIN                      DL_GPIO_PIN_7
#define S4_PIN                      DL_GPIO_PIN_7
#define S5_PIN                      DL_GPIO_PIN_8
#define S6_PIN                      DL_GPIO_PIN_9
#define S7_PIN                      DL_GPIO_PIN_10

#define LINE_BLACK_IS_HIGH          1
#define LINE_SENSOR_MASK_ALL        0x7FU
#define LINE_SAMPLE_COUNT           7U
#define LINE_SAMPLE_DELAY_CYCLES    80U
#define LINE_SEEN_LATCH_FRAMES      10U
#define LINE_KP_DIVISOR             16
#define LINE_CORR_LIMIT             190
#define LINE_SHARP_ERROR            1800
#define LINE_SHARP_KP_DIVISOR       7
#define LINE_SHARP_CORR_LIMIT       430
#define LINE_CENTER_OFFSET          0
#define LINE_LOST_FORCE_FRAMES      6U
#define LINE_LOST_STOP_FRAMES       260U

#define ENCODER_PORT                GPIOB
#define LEFT_A_PIN                  DL_GPIO_PIN_12
#define LEFT_B_PIN                  DL_GPIO_PIN_13
#define RIGHT_A_PIN                 DL_GPIO_PIN_14
#define RIGHT_B_PIN                 DL_GPIO_PIN_15
#define ENCODER_INT_MASK            (LEFT_A_PIN | RIGHT_A_PIN)

#define CONTROL_LOOP_DELAY_MS       5U
#define TRANSITION_HOLD_FRAMES      100U
#define START_GAP_MAX_FRAMES        900U
#define RUN_MAX_FRAMES              18000U

#define PHASE_STOP                  4U
#define PHASE_LINE_RUN              2U
#define PHASE_START_GAP_BRIDGE      8U
#define PHASE_FORCE_TURN            10U

#define STOP_REASON_NONE            0U
#define STOP_REASON_RUN_TIMEOUT     1U
#define STOP_REASON_LINE_LOST       2U
#define STOP_REASON_START_GAP       6U

#define HOLD_REASON_BRIDGE          1U
#define HOLD_REASON_LINE_START      3U
#define HOLD_REASON_REACQUIRE       2U

volatile uint8_t g_phase = 0;
volatile uint8_t g_stop_reason = STOP_REASON_NONE;
volatile uint8_t g_turn_mode = 0;
volatile uint32_t g_loop_count = 0;
volatile uint32_t g_run_sample = 0;

volatile uint8_t g_s1 = 0;
volatile uint8_t g_s2 = 0;
volatile uint8_t g_s3 = 0;
volatile uint8_t g_s4 = 0;
volatile uint8_t g_s5 = 0;
volatile uint8_t g_s6 = 0;
volatile uint8_t g_s7 = 0;
volatile uint8_t g_line_raw_mask = 0;
volatile uint8_t g_line_raw_high_last_mask = 0;
volatile uint8_t g_line_raw_high_or_mask = 0;
volatile uint8_t g_line_raw_high_and_mask = 0;
volatile uint8_t g_line_seen_latched = 0;
volatile uint8_t g_line_valid = 0;
volatile uint8_t g_line_active_count = 0;
volatile uint16_t g_line_lost_count = 0;
volatile uint16_t g_line_seen_latch_frames = 0;
volatile uint32_t g_line_seen_event_count = 0;
volatile uint32_t g_line_sample_call_count = 0;
volatile int16_t g_line_error = 0;
volatile int16_t g_line_last_error = 0;
volatile int32_t g_line_weighted_sum = 0;
volatile int32_t g_line_correction = 0;

volatile int32_t g_left_count = 0;
volatile int32_t g_right_count = 0;
volatile uint32_t g_left_edges = 0;
volatile uint32_t g_right_edges = 0;
volatile int32_t g_encoder_left_forward_total = 0;
volatile int32_t g_encoder_right_forward_total = 0;
volatile int32_t g_encoder_avg_forward_total = 0;
volatile int32_t g_encoder_diff_forward_total = 0;

volatile uint32_t g_left_pwm_ticks = PWM_LEFT_BASE_TICKS;
volatile uint32_t g_right_pwm_ticks = PWM_RIGHT_BASE_TICKS;
volatile uint32_t g_left_compare_ticks = PWM_PERIOD - PWM_LEFT_BASE_TICKS;
volatile uint32_t g_right_compare_ticks = PWM_PERIOD - PWM_RIGHT_BASE_TICKS;

volatile uint32_t g_start_gap_frames = 0;
volatile uint32_t g_start_gap_event_count = 0;
volatile uint32_t g_reacquire_event_count = 0;
volatile uint8_t g_force_turn_dir = 0;
volatile uint32_t g_force_turn_frames = 0;

volatile uint8_t g_startup_line_valid_snapshot = 0;
volatile uint8_t g_startup_line_raw_mask_snapshot = 0;
volatile uint8_t g_startup_line_active_count_snapshot = 0;
volatile int16_t g_startup_line_error_snapshot = 0;

volatile uint8_t g_state_switch_reason = 0;
volatile uint16_t g_state_switch_hold_frames_left = 0;
volatile uint32_t g_state_switch_hold_event_count = 0;
volatile uint32_t g_state_switch_hold_frames_total = 0;

static NOINLINE void delay_cycles_local(uint32_t cycles)
{
    while (cycles--) {
        __asm volatile("nop");
    }
}

static NOINLINE void delay_us(uint32_t us)
{
    uint32_t cycles_per_us = SYSCLK_HZ / 1000000U;
    while (us--) {
        delay_cycles_local(cycles_per_us);
    }
}

static NOINLINE void delay_ms(uint32_t ms)
{
    while (ms--) {
        delay_us(1000U);
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

static NOINLINE void led_set(bool on)
{
    if (on) {
        DL_GPIO_setPins(LED_PORT, LED_PIN);
    } else {
        DL_GPIO_clearPins(LED_PORT, LED_PIN);
    }
}

static NOINLINE void buzzer_set(bool on)
{
    if (on) {
        DL_GPIO_setPins(BUZZER_PORT, BUZZER_PIN);
    } else {
        DL_GPIO_clearPins(BUZZER_PORT, BUZZER_PIN);
    }
}

static NOINLINE void buzzer_beep_blocking(uint8_t count)
{
    for (uint8_t i = 0U; i < count; i++) {
        buzzer_set(true);
        delay_ms(90U);
        buzzer_set(false);
        delay_ms(90U);
    }
}

static NOINLINE void transition_hold(uint8_t reason, uint8_t beep_count)
{
    g_state_switch_reason = reason;
    g_state_switch_hold_event_count++;
    g_state_switch_hold_frames_left = TRANSITION_HOLD_FRAMES;

    buzzer_beep_blocking(beep_count);

    for (uint32_t i = 0U; i < TRANSITION_HOLD_FRAMES; i++) {
        led_set(((i % 10U) < 5U) ? true : false);
        delay_ms(20U);
        if (g_state_switch_hold_frames_left > 0U) {
            g_state_switch_hold_frames_left--;
        }
        g_state_switch_hold_frames_total++;
    }
    led_set(false);
}

static int32_t abs_int32(int32_t value)
{
    return (value < 0) ? -value : value;
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

static uint32_t clamp_pwm_normal(int32_t ticks)
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

static uint32_t clamp_pwm_turn(int32_t ticks)
{
    if (ticks <= 0) {
        return 0U;
    }
    if (ticks > (int32_t) PWM_MAX_TICKS) {
        return PWM_MAX_TICKS;
    }
    return (uint32_t) ticks;
}

static NOINLINE void set_pwm(uint32_t left_duty_ticks, uint32_t right_duty_ticks)
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

static NOINLINE void motors_forward(void)
{
    DL_GPIO_clearPins(MOTOR_PORT, AIN1_PIN | AIN2_PIN | BIN1_PIN | BIN2_PIN);
    DL_GPIO_setPins(MOTOR_PORT, AIN1_PIN | BIN1_PIN | STBY_PIN);
}

static NOINLINE void motors_stop(void)
{
    set_pwm(0U, 0U);
    DL_GPIO_clearPins(MOTOR_PORT, AIN1_PIN | AIN2_PIN | BIN1_PIN | BIN2_PIN);
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
    return (uint8_t) (mask & LINE_SENSOR_MASK_ALL);
}

static NOINLINE void line_sensor_update(void)
{
    uint8_t high_last = 0U;
    uint8_t high_or = 0U;
    uint8_t high_and = LINE_SENSOR_MASK_ALL;
    uint8_t logical_mask;

    for (uint8_t i = 0U; i < LINE_SAMPLE_COUNT; i++) {
        high_last = read_line_raw_high_mask_once();
        high_or |= high_last;
        high_and &= high_last;
        if (i + 1U < LINE_SAMPLE_COUNT) {
            delay_cycles_local(LINE_SAMPLE_DELAY_CYCLES);
        }
    }

    g_line_sample_call_count++;
    g_line_raw_high_last_mask = high_last;
    g_line_raw_high_or_mask = (uint8_t) (high_or & LINE_SENSOR_MASK_ALL);
    g_line_raw_high_and_mask = (uint8_t) (high_and & LINE_SENSOR_MASK_ALL);

#if LINE_BLACK_IS_HIGH
    logical_mask = g_line_raw_high_or_mask;
#else
    logical_mask = (uint8_t) ((~g_line_raw_high_and_mask) & LINE_SENSOR_MASK_ALL);
#endif

    if ((logical_mask != 0U) && (g_line_seen_latch_frames == 0U)) {
        g_line_seen_event_count++;
    }
    if (logical_mask != 0U) {
        g_line_seen_latch_frames = LINE_SEEN_LATCH_FRAMES;
    } else if (g_line_seen_latch_frames > 0U) {
        g_line_seen_latch_frames--;
    }
    g_line_seen_latched = (g_line_seen_latch_frames > 0U) ? 1U : 0U;

    g_s1 = (uint8_t) ((logical_mask >> 0) & 0x01U);
    g_s2 = (uint8_t) ((logical_mask >> 1) & 0x01U);
    g_s3 = (uint8_t) ((logical_mask >> 2) & 0x01U);
    g_s4 = (uint8_t) ((logical_mask >> 3) & 0x01U);
    g_s5 = (uint8_t) ((logical_mask >> 4) & 0x01U);
    g_s6 = (uint8_t) ((logical_mask >> 5) & 0x01U);
    g_s7 = (uint8_t) ((logical_mask >> 6) & 0x01U);

    g_line_raw_mask = logical_mask;
}

static NOINLINE void line_position_update(void)
{
    int32_t sum = 0;
    uint8_t count = 0;

    if (g_s1 != 0U) { sum += -3000; count++; }
    if (g_s2 != 0U) { sum += -2000; count++; }
    if (g_s3 != 0U) { sum += -1000; count++; }
    if (g_s4 != 0U) { sum += 0;     count++; }
    if (g_s5 != 0U) { sum += 1000;  count++; }
    if (g_s6 != 0U) { sum += 2000;  count++; }
    if (g_s7 != 0U) { sum += 3000;  count++; }

    g_line_active_count = count;
    g_line_weighted_sum = sum;

    if (count == 0U) {
        g_line_valid = 0U;
        if (g_line_lost_count < 0xFFFFU) {
            g_line_lost_count++;
        }
        g_line_error = g_line_last_error;
        return;
    }

    g_line_valid = 1U;
    g_line_lost_count = 0U;
    g_line_error = (int16_t) ((sum / (int32_t) count) - LINE_CENTER_OFFSET);
    g_line_last_error = g_line_error;
}

static NOINLINE void encoder_distance_update(void)
{
    int32_t left_count;
    int32_t right_count;

    __disable_irq();
    left_count = g_left_count;
    right_count = g_right_count;
    __enable_irq();

    g_encoder_left_forward_total = -left_count;
    g_encoder_right_forward_total = right_count;
    g_encoder_avg_forward_total =
        (g_encoder_left_forward_total + g_encoder_right_forward_total) / 2;
    g_encoder_diff_forward_total =
        g_encoder_left_forward_total - g_encoder_right_forward_total;
}

static NOINLINE void encoder_reset_counts(void)
{
    __disable_irq();
    g_left_count = 0;
    g_right_count = 0;
    g_left_edges = 0U;
    g_right_edges = 0U;
    DL_GPIO_clearInterruptStatus(ENCODER_PORT, ENCODER_INT_MASK);
    __enable_irq();

    g_encoder_left_forward_total = 0;
    g_encoder_right_forward_total = 0;
    g_encoder_avg_forward_total = 0;
    g_encoder_diff_forward_total = 0;
}

static NOINLINE void line_follow_drive(void)
{
    int32_t error = (int32_t) g_line_error;
    bool lost_recovery = (g_line_valid == 0U) && (g_line_last_error != 0);
    bool sharp_turn = (abs_int32(error) >= LINE_SHARP_ERROR) || lost_recovery;
    int32_t correction;

    if ((g_line_valid == 0U) && (g_line_last_error == 0)) {
        g_turn_mode = 3U;
        g_line_correction = 0;
        set_pwm(PWM_LEFT_BASE_TICKS, PWM_RIGHT_BASE_TICKS);
        return;
    }

    if (sharp_turn) {
        g_turn_mode = lost_recovery ? 2U : 1U;
        correction = clamp_int32(
            error / LINE_SHARP_KP_DIVISOR,
            -LINE_SHARP_CORR_LIMIT,
            LINE_SHARP_CORR_LIMIT);
        g_line_correction = correction;
        set_pwm(
            clamp_pwm_turn((int32_t) PWM_LEFT_BASE_TICKS - correction),
            clamp_pwm_turn((int32_t) PWM_RIGHT_BASE_TICKS + correction));
        return;
    }

    g_turn_mode = 0U;
    correction = clamp_int32(
        error / LINE_KP_DIVISOR,
        -LINE_CORR_LIMIT,
        LINE_CORR_LIMIT);
    g_line_correction = correction;
    set_pwm(
        clamp_pwm_normal((int32_t) PWM_LEFT_BASE_TICKS - correction),
        clamp_pwm_normal((int32_t) PWM_RIGHT_BASE_TICKS + correction));
}

static NOINLINE void bridge_drive(void)
{
    g_turn_mode = 4U;
    g_line_correction = 0;
    set_pwm(PWM_BRIDGE_LEFT_TICKS, PWM_BRIDGE_RIGHT_TICKS);
}

static NOINLINE void force_turn_drive(void)
{
    int32_t dir = (g_force_turn_dir == 0U) ? 1 : (int32_t) g_force_turn_dir;
    int32_t correction = dir * PWM_FORCE_CORR_TICKS;

    g_turn_mode = 2U;
    g_line_correction = correction;
    set_pwm(
        clamp_pwm_turn((int32_t) PWM_FORCE_BASE_TICKS - correction),
        clamp_pwm_turn((int32_t) PWM_FORCE_BASE_TICKS + correction));
}

static NOINLINE void hardware_init(void)
{
    SYSCFG_DL_init();

    init_output(IOMUX_PINCM37, LED_PORT, LED_PIN, false);
    init_output(IOMUX_PINCM28, BUZZER_PORT, BUZZER_PIN, false);

    init_output(IOMUX_PINCM12, MOTOR_PORT, AIN1_PIN, false);
    init_output(IOMUX_PINCM13, MOTOR_PORT, AIN2_PIN, false);
    init_output(IOMUX_PINCM15, MOTOR_PORT, BIN1_PIN, false);
    init_output(IOMUX_PINCM16, MOTOR_PORT, BIN2_PIN, false);
    init_output(IOMUX_PINCM17, MOTOR_PORT, STBY_PIN, false);

    init_input(IOMUX_PINCM18);
    init_input(IOMUX_PINCM23);
    init_input(IOMUX_PINCM24);
    init_input(IOMUX_PINCM14);
    init_input(IOMUX_PINCM19);
    init_input(IOMUX_PINCM26);
    init_input(IOMUX_PINCM27);

    DL_GPIO_initDigitalInputFeatures(IOMUX_PINCM29,
        DL_GPIO_INVERSION_DISABLE,
        DL_GPIO_RESISTOR_NONE,
        DL_GPIO_HYSTERESIS_ENABLE,
        DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initDigitalInputFeatures(IOMUX_PINCM30,
        DL_GPIO_INVERSION_DISABLE,
        DL_GPIO_RESISTOR_NONE,
        DL_GPIO_HYSTERESIS_ENABLE,
        DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initDigitalInputFeatures(IOMUX_PINCM31,
        DL_GPIO_INVERSION_DISABLE,
        DL_GPIO_RESISTOR_NONE,
        DL_GPIO_HYSTERESIS_ENABLE,
        DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initDigitalInputFeatures(IOMUX_PINCM32,
        DL_GPIO_INVERSION_DISABLE,
        DL_GPIO_RESISTOR_NONE,
        DL_GPIO_HYSTERESIS_ENABLE,
        DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_setLowerPinsPolarity(ENCODER_PORT,
        DL_GPIO_PIN_12_EDGE_RISE_FALL |
        DL_GPIO_PIN_14_EDGE_RISE_FALL);
    DL_GPIO_clearInterruptStatus(ENCODER_PORT, ENCODER_INT_MASK);
    DL_GPIO_enableInterrupt(ENCODER_PORT, ENCODER_INT_MASK);
    NVIC_EnableIRQ(GPIOB_INT_IRQn);

    DL_TimerG_reset(PWM_TIMER);
    DL_TimerG_enablePower(PWM_TIMER);
    delay_cycles_local(16U);

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

    motors_stop();
}

static NOINLINE void startup_indication(void)
{
    for (uint8_t i = 0U; i < 3U; i++) {
        led_set(true);
        delay_ms(80U);
        led_set(false);
        delay_ms(80U);
    }
    buzzer_beep_blocking(1U);
}

static NOINLINE void stop_with_reason(uint8_t reason)
{
    g_phase = PHASE_STOP;
    g_stop_reason = reason;
    motors_stop();
    buzzer_set(true);
    delay_ms(600U);
    buzzer_set(false);
}

int main(void)
{
    hardware_init();
    startup_indication();

    line_sensor_update();
    line_position_update();
    encoder_reset_counts();
    encoder_distance_update();

    g_startup_line_valid_snapshot = g_line_valid;
    g_startup_line_raw_mask_snapshot = g_line_raw_mask;
    g_startup_line_active_count_snapshot = g_line_active_count;
    g_startup_line_error_snapshot = g_line_error;

    if (g_line_valid != 0U) {
        g_phase = PHASE_LINE_RUN;
        transition_hold(HOLD_REASON_LINE_START, 4U);
        line_follow_drive();
        motors_forward();
    } else {
        g_phase = PHASE_START_GAP_BRIDGE;
        g_start_gap_event_count++;
        transition_hold(HOLD_REASON_BRIDGE, 2U);
        bridge_drive();
        motors_forward();
    }

    while (1) {
        g_loop_count++;
        encoder_distance_update();
        line_sensor_update();
        line_position_update();

        if (g_phase == PHASE_START_GAP_BRIDGE) {
            if ((g_line_valid != 0U) || (g_line_seen_latched != 0U)) {
                g_reacquire_event_count++;
                g_phase = PHASE_LINE_RUN;
                transition_hold(HOLD_REASON_REACQUIRE, 3U);
                line_follow_drive();
                motors_forward();
            } else if (g_start_gap_frames >= START_GAP_MAX_FRAMES) {
                stop_with_reason(STOP_REASON_START_GAP);
            } else {
                g_start_gap_frames++;
                bridge_drive();
            }
        } else if (g_phase == PHASE_LINE_RUN) {
            if (g_line_valid != 0U) {
                line_follow_drive();
                g_force_turn_dir =
                    (g_line_error >= 0) ? 1U : (uint8_t) -1;
                g_force_turn_frames = 0U;
            } else if (g_line_lost_count >= LINE_LOST_STOP_FRAMES) {
                stop_with_reason(STOP_REASON_LINE_LOST);
            } else if (g_line_lost_count >= LINE_LOST_FORCE_FRAMES) {
                g_phase = PHASE_FORCE_TURN;
                g_force_turn_frames = 0U;
                force_turn_drive();
            } else {
                line_follow_drive();
            }
            g_run_sample++;
        } else if (g_phase == PHASE_FORCE_TURN) {
            if ((g_line_valid != 0U) || (g_line_seen_latched != 0U)) {
                g_phase = PHASE_LINE_RUN;
                g_reacquire_event_count++;
                line_follow_drive();
            } else if (g_line_lost_count >= LINE_LOST_STOP_FRAMES) {
                stop_with_reason(STOP_REASON_LINE_LOST);
            } else {
                g_force_turn_frames++;
                force_turn_drive();
            }
            g_run_sample++;
        } else {
            motors_stop();
        }

        if ((g_run_sample >= RUN_MAX_FRAMES) && (g_phase != PHASE_STOP)) {
            stop_with_reason(STOP_REASON_RUN_TIMEOUT);
        }

        delay_ms(CONTROL_LOOP_DELAY_MS);
    }
}

void GPIOB_IRQHandler(void)
{
    uint32_t status =
        DL_GPIO_getEnabledInterruptStatus(ENCODER_PORT, ENCODER_INT_MASK);

    if ((status & LEFT_A_PIN) != 0U) {
        uint32_t pins =
            DL_GPIO_readPins(ENCODER_PORT, LEFT_A_PIN | LEFT_B_PIN);
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
        uint32_t pins =
            DL_GPIO_readPins(ENCODER_PORT, RIGHT_A_PIN | RIGHT_B_PIN);
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
