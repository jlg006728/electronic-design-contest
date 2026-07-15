/**
 * V49 corrected sensor polarity, suspended-state rejection, debounced line state.
 *
 * Scope:
 *   - Chassis/line-following test only, no ABCD mission logic.
 *   - Physical DO is low on white and high on black/suspended in the current wiring.
 *   - Software g_s1..g_s7 remains normalized as white=1 and black=0.
 *   - Bridge stage uses encoder distance PID to keep straight without a line.
 *   - Enter line-run on majority line detection or confirmed pulse candidates.
 *   - Exit line-run after three consecutive frames without a usable black-line bit.
 *   - All seven normalized sensors at black is treated as suspended/abnormal.
 *   - OpenMV UART + PA14/PA17 dual-servo tracking runs in parallel but does not guide chassis.
 */

#include <stdbool.h>
#include <stdint.h>
#include "ti_msp_dl_config.h"

#define NOINLINE                    __attribute__((noinline))

#define SYSCLK_HZ                   32000000U

#define MOTOR_PORT                  GPIOB
#define MOTOR_ENABLE                1U
#define AIN1_PIN                    DL_GPIO_PIN_0
#define AIN2_PIN                    DL_GPIO_PIN_1
#define BIN1_PIN                    DL_GPIO_PIN_2
#define BIN2_PIN                    DL_GPIO_PIN_3
#define STBY_PIN                    DL_GPIO_PIN_4

#define PWM_TIMER                   TIMG0
#define PWM_PERIOD                  1600U
#define PWM_LEFT_BASE_TICKS         250U
#define PWM_RIGHT_BASE_TICKS        242U
#define PWM_MIN_TICKS               180U
#define PWM_MAX_TICKS               350U

#define START_GAP_LEFT_BASE_TICKS   250U
#define START_GAP_RIGHT_BASE_TICKS  250U
#define START_GAP_LEFT_TRIM_TICKS   (-20)
#define START_GAP_RIGHT_TRIM_TICKS  20
#define START_GAP_MAX_FRAMES        900U
#define START_GAP_ENCODER_DISTANCE_KP_DIV 48
#define START_GAP_ENCODER_INTEGRAL_KI_DIV 1800
#define START_GAP_ENCODER_SPEED_KD_DIV 2
#define START_GAP_ENCODER_INTEGRAL_LIMIT 12000
#define START_GAP_ENCODER_LIMIT     130
#define START_GAP_ENCODER_SLEW_LIMIT 14

#define LINE_KP_DIVISOR             16
#define LINE_CORR_LIMIT             190
#define LINE_SHARP_ERROR            1800
#define LINE_SHARP_KP_DIVISOR       6
#define LINE_SHARP_CORR_LIMIT       450

#define SENSOR_HIGH_IS_WHITE        0
#define LINE_SAMPLE_COUNT           9U
#define LINE_WHITE_MIN_HIGH_SAMPLES 5U
#define LINE_SAMPLE_DELAY_CYCLES    120U
#define SENSOR_QUIET_SAMPLE_ENABLE  1U
#define SENSOR_QUIET_SAMPLE_US      1500U
#define LINE_ENTRY_USE_PULSE_MASK   1U
#define LINE_ENTRY_CANDIDATE_HISTORY_MASK 0x0FU
#define LINE_ENTRY_CANDIDATE_CONFIRM_MIN 2U
#define LINE_ENTRY_CONTROL_FRAMES   3U
#define LINE_START_IGNORE_ENABLE    1U
#define LINE_START_IGNORE_MAX_ACTIVE 1U
#define LINE_START_DIRECT_ENABLE    1U
#define LINE_START_DIRECT_MIN_ACTIVE 2U
#define LINE_EXIT_CONFIRM_FRAMES    3U
#define LINE_HISTORY_WIDTH          3U
#define LINE_MASK_ALL               0x7FU
#define LINE_ENTER_MIN_ACTIVE       1U
#define LINE_ENTER_MAX_ACTIVE       4U
#define SENSOR_ALL_LOW_STOP_FRAMES  3U

#define RUN_MAX_FRAMES              9000U

#define LED_PORT                    GPIOA
#define LED_PIN                     DL_GPIO_PIN_15

#define BUZZER_PORT                 GPIOB
#define BUZZER_PIN                  DL_GPIO_PIN_11
#define BUZZER_ON_FRAMES            5U
#define BUZZER_OFF_FRAMES           5U
#define BUZZER_LONG_FRAMES          45U
#define BUZZER_TRIGGER_COOLDOWN_FRAMES 25U

#define S1_PIN                      DL_GPIO_PIN_5
#define S2_PIN                      DL_GPIO_PIN_6
#define S3_PIN                      DL_GPIO_PIN_7
#define S4_PIN                      DL_GPIO_PIN_7
#define S5_PIN                      DL_GPIO_PIN_8
#define S6_PIN                      DL_GPIO_PIN_9
#define S7_PIN                      DL_GPIO_PIN_10

#define ENCODER_PORT                GPIOB
#define LEFT_A_PIN                  DL_GPIO_PIN_12
#define LEFT_B_PIN                  DL_GPIO_PIN_13
#define RIGHT_A_PIN                 DL_GPIO_PIN_14
#define RIGHT_B_PIN                 DL_GPIO_PIN_15
#define ENCODER_INT_MASK            (LEFT_A_PIN | RIGHT_A_PIN)

#define UART_OPENMV                 UART0
#define UART_BAUDRATE               115200U
#define OMV_HEADER0                 0xAAU
#define OMV_HEADER1                 0x55U
#define OMV_FOOTER                  0x5BU
#define OMV_FRAME_LENGTH            17U
#define OMV_TYPE_COLOR              3U
#define OMV_FLAG_CAMERA_OK          0x01U
#define OMV_FLAG_TARGET_DETECTED    0x02U
#define OMV_FLAG_UART_RX_SEEN       0x04U

#define IMG_WIDTH                   320
#define IMG_HEIGHT                  240
#define IMG_CENTER_X                (IMG_WIDTH / 2)
#define IMG_CENTER_Y                (IMG_HEIGHT / 2)
#define AIM_X_PULSE_SIGN            (-1)
#define AIM_Y_PULSE_SIGN            (-1)
#define AIM_DEAD_X_PIXELS           20
#define AIM_DEAD_Y_PIXELS           16
#define AIM_X_DIVISOR               2
#define AIM_Y_DIVISOR               3
#define AIM_MAX_STEP_US             60
#define AIM_MIN_PIXELS              14U
#define AIM_FILTER_DIVISOR          2
#define AIM_TARGET_CONFIRM_FRAMES   2U
#define LINK_TIMEOUT_FRAMES         120U

#define SERVO_PORT                  GPIOA
#define SERVO_X_PIN                 DL_GPIO_PIN_14
#define SERVO_Y_PIN                 DL_GPIO_PIN_17
#define SERVO_PERIOD_US             20000U
#define SERVO_X_HOME_US             1600U
#define SERVO_Y_HOME_US             1500U
#define SERVO_X_MIN_US              350U
#define SERVO_X_MAX_US              2650U
#define SERVO_Y_MIN_US              500U
#define SERVO_Y_MAX_US              2500U
#define SERVO_STARTUP_HOME_FRAMES   50U
#define SERVO_STARTUP_TEST_X_OFFSET 120
#define SERVO_STARTUP_TEST_Y_OFFSET 80
#define SERVO_STARTUP_TEST_FRAMES   10U

#define PHASE_STARTUP               1U
#define PHASE_LINE_RUN              2U
#define PHASE_DONE                  3U
#define PHASE_STOP                  4U
#define PHASE_START_GAP_BRIDGE      8U

#define STOP_REASON_NONE            0U
#define STOP_REASON_RUN_TIMEOUT     1U
#define STOP_REASON_SENSOR_ALL_LOW  3U
#define STOP_REASON_START_GAP       6U

#define SWITCH_REASON_START_BRIDGE  1U
#define SWITCH_REASON_ENTER_LINE    2U
#define SWITCH_REASON_EXIT_LINE     3U

volatile uint8_t g_s1 = 0;
volatile uint8_t g_s2 = 0;
volatile uint8_t g_s3 = 0;
volatile uint8_t g_s4 = 0;
volatile uint8_t g_s5 = 0;
volatile uint8_t g_s6 = 0;
volatile uint8_t g_s7 = 0;

volatile uint8_t g_line_raw_mask = 0;
volatile uint8_t g_black_line_mask = 0;
volatile uint8_t g_line_start_ignore_mask = 0;
volatile uint8_t g_line_effective_detect_mask = 0;
volatile uint8_t g_line_black_pulse_mask = 0;
volatile uint8_t g_line_candidate_mask = 0;
volatile uint8_t g_line_detect_mask = 0;
volatile uint8_t g_line_entry_mask = 0;
volatile uint8_t g_line_confirmed_entry_mask = 0;
volatile uint8_t g_line_entry_control_frames_left = 0;
volatile uint8_t g_line_candidate_history_mask = 0;
volatile uint8_t g_line_candidate_confirm_count = 0;
volatile uint32_t g_line_confirmed_entry_count = 0;
volatile uint8_t g_line_exit_missing_count = 0;
volatile uint32_t g_line_exit_confirmed_count = 0;
volatile uint8_t g_startup_ignore_active_count = 0;
volatile uint8_t g_startup_direct_line_allowed = 0;
volatile uint8_t g_line_history0 = 0;
volatile uint8_t g_line_history1 = 0;
volatile uint8_t g_line_history2 = 0;
volatile uint8_t g_line_history_or_mask = 0;
volatile uint8_t g_line_history_nonzero = 0;
volatile uint8_t g_line_active_count = 0;
volatile uint8_t g_line_valid = 0;
volatile uint8_t g_sensor_all_low_abnormal = 0;
volatile uint8_t g_sensor_all_low_count = 0;
volatile int16_t g_line_error = 0;
volatile int16_t g_line_last_error = 0;
volatile int32_t g_line_weighted_sum = 0;
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

volatile int32_t g_left_count = 0;
volatile int32_t g_right_count = 0;
volatile uint32_t g_left_edges = 0;
volatile uint32_t g_right_edges = 0;
volatile int32_t g_encoder_left_forward_total = 0;
volatile int32_t g_encoder_right_forward_total = 0;
volatile int32_t g_encoder_avg_forward_total = 0;
volatile int32_t g_encoder_diff_forward_total = 0;
volatile int32_t g_start_gap_encoder_integral_error = 0;
volatile int32_t g_start_gap_encoder_p_term = 0;
volatile int32_t g_start_gap_encoder_i_term = 0;
volatile int32_t g_start_gap_encoder_d_term = 0;
volatile int32_t g_start_gap_encoder_target_correction = 0;
volatile int32_t g_start_gap_encoder_correction = 0;
volatile int32_t g_start_gap_left_forward_prev = 0;
volatile int32_t g_start_gap_right_forward_prev = 0;
volatile int32_t g_start_gap_left_forward_delta = 0;
volatile int32_t g_start_gap_right_forward_delta = 0;
volatile int32_t g_start_gap_encoder_delta_diff = 0;
volatile int32_t g_start_gap_encoder_speed_term = 0;
volatile int32_t g_start_gap_encoder_distance_term = 0;
volatile uint32_t g_start_gap_left_cmd_ticks = START_GAP_LEFT_BASE_TICKS;
volatile uint32_t g_start_gap_right_cmd_ticks = START_GAP_RIGHT_BASE_TICKS;
volatile uint8_t g_start_gap_encoder_seen = 0;
volatile uint32_t g_motor_quiet_sample_count = 0;

volatile uint32_t g_left_pwm_ticks = PWM_LEFT_BASE_TICKS;
volatile uint32_t g_right_pwm_ticks = PWM_RIGHT_BASE_TICKS;
volatile uint32_t g_left_compare_ticks = PWM_PERIOD - PWM_LEFT_BASE_TICKS;
volatile uint32_t g_right_compare_ticks = PWM_PERIOD - PWM_RIGHT_BASE_TICKS;

volatile uint32_t g_run_frame = 0;
volatile uint32_t g_start_gap_frames = 0;
volatile uint32_t g_reacquire_event_count = 0;
volatile uint32_t g_firmware_version = 49U;
volatile uint8_t g_phase = PHASE_STARTUP;
volatile uint8_t g_stop_reason = STOP_REASON_NONE;
volatile uint8_t g_state_switch_reason = 0;
volatile uint32_t g_state_switch_event_count = 0;

volatile uint32_t g_omv_rx_bytes = 0;
volatile uint32_t g_omv_frame_count = 0;
volatile uint32_t g_omv_color_frame_count = 0;
volatile uint32_t g_omv_bad_checksum = 0;
volatile uint32_t g_omv_bad_footer = 0;
volatile uint32_t g_omv_sync_drop = 0;
volatile uint32_t g_omv_timeout_frames = 0;
volatile uint8_t g_omv_link_ok = 0;
volatile uint8_t g_omv_frame_type = 0;
volatile uint8_t g_omv_flags = 0;
volatile uint8_t g_omv_camera_ok = 0;
volatile uint8_t g_omv_target_detected = 0;
volatile uint8_t g_omv_uart_rx_seen = 0;
volatile uint16_t g_omv_seq = 0;
volatile int16_t g_omv_cx = -1;
volatile int16_t g_omv_cy = -1;
volatile uint16_t g_omv_pixels = 0;
volatile uint16_t g_omv_fps_x10 = 0;
volatile uint8_t g_omv_l_mean = 0;
volatile uint8_t g_omv_last_frame[OMV_FRAME_LENGTH] = {0};

volatile uint16_t g_servo_x_us = SERVO_X_HOME_US;
volatile uint16_t g_servo_y_us = SERVO_Y_HOME_US;
volatile uint32_t g_servo_frame_count = 0;
volatile uint8_t g_servo_startup_home_done = 0;
volatile uint32_t g_aim_update_count = 0;
volatile uint32_t g_no_target_frames = 0;
volatile int16_t g_aim_raw_dx = 0;
volatile int16_t g_aim_raw_dy = 0;
volatile int16_t g_aim_dx = 0;
volatile int16_t g_aim_dy = 0;
volatile int16_t g_aim_filtered_cx = IMG_CENTER_X;
volatile int16_t g_aim_filtered_cy = IMG_CENTER_Y;
volatile int16_t g_aim_x_step_us = 0;
volatile int16_t g_aim_y_step_us = 0;
volatile uint8_t g_aim_state = 0;
volatile uint8_t g_aim_filter_ready = 0;
volatile uint8_t g_aim_target_confirm_count = 0;

volatile uint8_t g_buzzer_mode = 0;
volatile uint8_t g_buzzer_output_on = 0;
volatile uint8_t g_buzzer_beeps_left = 0;
volatile uint8_t g_buzzer_last_beep_count = 0;
volatile uint16_t g_buzzer_frames_left = 0;
volatile uint16_t g_buzzer_cooldown_frames = 0;
volatile uint32_t g_buzzer_event_count = 0;
volatile uint32_t g_buzzer_suppressed_count = 0;

static uint8_t s_rx_buf[OMV_FRAME_LENGTH];
static uint8_t s_rx_idx = 0;

static NOINLINE void delay_us(uint32_t us)
{
    if (us != 0U) {
        delay_cycles(us * 32U);
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
        DL_GPIO_RESISTOR_NONE,
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
        g_buzzer_output_on = 1U;
    } else {
        DL_GPIO_clearPins(BUZZER_PORT, BUZZER_PIN);
        g_buzzer_output_on = 0U;
    }
}

static NOINLINE void buzzer_start_beeps(uint8_t count)
{
    if (count == 0U) {
        return;
    }
    if ((g_buzzer_cooldown_frames > 0U) &&
        (g_buzzer_last_beep_count == count)) {
        g_buzzer_suppressed_count++;
        return;
    }
    buzzer_set(false);
    g_buzzer_mode = 1U;
    g_buzzer_beeps_left = count;
    g_buzzer_last_beep_count = count;
    g_buzzer_frames_left = 0U;
    g_buzzer_cooldown_frames = BUZZER_TRIGGER_COOLDOWN_FRAMES;
    g_buzzer_event_count++;
}

static NOINLINE void buzzer_start_long(void)
{
    if (g_buzzer_mode == 2U) {
        return;
    }
    g_buzzer_mode = 2U;
    g_buzzer_beeps_left = 0U;
    g_buzzer_frames_left = BUZZER_LONG_FRAMES;
    g_buzzer_cooldown_frames = BUZZER_TRIGGER_COOLDOWN_FRAMES;
    buzzer_set(true);
    g_buzzer_event_count++;
}

static NOINLINE void buzzer_update_one_frame(void)
{
    if (g_buzzer_cooldown_frames > 0U) {
        g_buzzer_cooldown_frames--;
    }

    if (g_buzzer_mode == 0U) {
        buzzer_set(false);
        return;
    }

    if (g_buzzer_mode == 2U) {
        if (g_buzzer_frames_left > 0U) {
            g_buzzer_frames_left--;
            return;
        }
        buzzer_set(false);
        g_buzzer_mode = 0U;
        return;
    }

    if (g_buzzer_frames_left > 0U) {
        g_buzzer_frames_left--;
        return;
    }

    if (g_buzzer_output_on != 0U) {
        buzzer_set(false);
        g_buzzer_frames_left = BUZZER_OFF_FRAMES;
        return;
    }

    if (g_buzzer_beeps_left > 0U) {
        g_buzzer_beeps_left--;
        buzzer_set(true);
        g_buzzer_frames_left = BUZZER_ON_FRAMES;
        return;
    }

    buzzer_set(false);
    g_buzzer_mode = 0U;
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

static int16_t abs_i16(int16_t value)
{
    return (value < 0) ? (int16_t) -value : value;
}

static uint8_t count_bits7(uint8_t value)
{
    uint8_t count = 0U;
    value &= LINE_MASK_ALL;
    for (uint8_t i = 0U; i < 7U; i++) {
        count += (uint8_t) ((value >> i) & 0x01U);
    }
    return count;
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

static uint16_t clamp_servo(int32_t value, uint16_t min_value, uint16_t max_value)
{
    if (value < (int32_t) min_value) {
        return min_value;
    }
    if (value > (int32_t) max_value) {
        return max_value;
    }
    return (uint16_t) value;
}

static NOINLINE void pwm_init(void)
{
    DL_TimerG_reset(PWM_TIMER);
    DL_TimerG_enablePower(PWM_TIMER);
    delay_cycles(16U);

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
#if MOTOR_ENABLE
    DL_GPIO_clearPins(MOTOR_PORT, AIN1_PIN | AIN2_PIN | BIN1_PIN | BIN2_PIN);
    DL_GPIO_setPins(MOTOR_PORT, AIN1_PIN | BIN1_PIN | STBY_PIN);
#else
    set_pwm(0U, 0U);
    DL_GPIO_clearPins(MOTOR_PORT,
        AIN1_PIN | AIN2_PIN | BIN1_PIN | BIN2_PIN | STBY_PIN);
#endif
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

static NOINLINE void reset_line_history(void)
{
    g_line_history0 = 0U;
    g_line_history1 = 0U;
    g_line_history2 = 0U;
    g_line_history_or_mask = 0U;
    g_line_history_nonzero = 0U;
    g_line_entry_mask = 0U;
    g_line_confirmed_entry_mask = 0U;
    g_line_entry_control_frames_left = 0U;
    g_line_candidate_history_mask = 0U;
    g_line_candidate_confirm_count = 0U;
    g_line_candidate_mask = 0U;
    g_line_exit_missing_count = 0U;
}

static NOINLINE void update_line_history(void)
{
    g_line_history2 = g_line_history1;
    g_line_history1 = g_line_history0;
    g_line_history0 = g_line_entry_mask;
    g_line_history_or_mask =
        (uint8_t) (g_line_history0 | g_line_history1 | g_line_history2);
    g_line_history_nonzero = (g_line_history_or_mask != 0U) ? 1U : 0U;
}

static NOINLINE void read_line_sensors(void)
{
    uint8_t high_last = 0U;
    uint8_t high_or = 0U;
    uint8_t high_and = LINE_MASK_ALL;
    uint8_t c1 = 0U;
    uint8_t c2 = 0U;
    uint8_t c3 = 0U;
    uint8_t c4 = 0U;
    uint8_t c5 = 0U;
    uint8_t c6 = 0U;
    uint8_t c7 = 0U;
    uint8_t active_count;
    uint8_t black_pulse_mask;
    uint8_t pulse_candidate_mask;
    uint8_t pulse_candidate_seen;

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
    g_black_line_mask = (uint8_t) ((~g_line_raw_mask) & LINE_MASK_ALL);
    black_pulse_mask = (uint8_t) ((~high_and) & LINE_MASK_ALL);
    g_line_black_pulse_mask = black_pulse_mask;

    active_count = count_bits7(g_black_line_mask);
    g_sensor_all_low_abnormal = (active_count >= 7U) ? 1U : 0U;
    if (g_sensor_all_low_abnormal != 0U) {
        if (g_sensor_all_low_count < 0xFFU) {
            g_sensor_all_low_count++;
        }
    } else {
        g_sensor_all_low_count = 0U;
    }

    if ((active_count >= LINE_ENTER_MIN_ACTIVE) &&
        (active_count <= LINE_ENTER_MAX_ACTIVE)) {
        g_line_detect_mask = g_black_line_mask;
    } else {
        g_line_detect_mask = 0U;
    }

    g_line_effective_detect_mask =
        (uint8_t) (g_line_detect_mask & (uint8_t) (~g_line_start_ignore_mask));
    pulse_candidate_mask = 0U;
#if LINE_ENTRY_USE_PULSE_MASK
    if ((g_line_effective_detect_mask == 0U) &&
        (black_pulse_mask != 0U) &&
        (black_pulse_mask != LINE_MASK_ALL)) {
        pulse_candidate_mask = (uint8_t)
            (black_pulse_mask & (uint8_t) (~g_line_start_ignore_mask));
    }
#endif
    g_line_candidate_mask =
        (g_line_effective_detect_mask != 0U) ?
            g_line_effective_detect_mask :
            pulse_candidate_mask;
    pulse_candidate_seen = (g_line_candidate_mask != 0U) ? 1U : 0U;
    g_line_candidate_history_mask = (uint8_t)
        (((g_line_candidate_history_mask << 1) | pulse_candidate_seen) &
            LINE_ENTRY_CANDIDATE_HISTORY_MASK);
    g_line_candidate_confirm_count =
        count_bits7(g_line_candidate_history_mask);

    if ((g_line_candidate_mask != 0U) &&
        (g_line_candidate_confirm_count >=
        LINE_ENTRY_CANDIDATE_CONFIRM_MIN)) {
        g_line_entry_mask = g_line_candidate_mask;
        g_line_confirmed_entry_mask = g_line_candidate_mask;
    } else {
        g_line_entry_mask = 0U;
    }

    update_line_history();
}

static NOINLINE void read_line_sensors_for_control(void)
{
#if MOTOR_ENABLE
    uint32_t saved_left = g_left_pwm_ticks;
    uint32_t saved_right = g_right_pwm_ticks;

    if (SENSOR_QUIET_SAMPLE_ENABLE != 0U) {
        set_pwm(0U, 0U);
        delay_us(SENSOR_QUIET_SAMPLE_US);
        g_motor_quiet_sample_count++;
        read_line_sensors();
        set_pwm(saved_left, saved_right);
        return;
    }
#endif

    read_line_sensors();
}

static NOINLINE void update_line_position(void)
{
    int32_t sum = 0;
    uint8_t count = 0U;
    uint8_t mask = g_line_effective_detect_mask;

    if ((mask == 0U) &&
        (g_line_entry_control_frames_left > 0U) &&
        (g_line_confirmed_entry_mask != 0U)) {
        mask = g_line_confirmed_entry_mask;
    }

    if ((mask & (1U << 0)) != 0U) { sum += -3000; count++; }
    if ((mask & (1U << 1)) != 0U) { sum += -2000; count++; }
    if ((mask & (1U << 2)) != 0U) { sum += -1000; count++; }
    if ((mask & (1U << 3)) != 0U) { sum += 0;     count++; }
    if ((mask & (1U << 4)) != 0U) { sum += 1000;  count++; }
    if ((mask & (1U << 5)) != 0U) { sum += 2000;  count++; }
    if ((mask & (1U << 6)) != 0U) { sum += 3000;  count++; }

    g_line_active_count = count;
    g_line_weighted_sum = sum;

    if (count == 0U) {
        g_line_valid = 0U;
        g_line_error = g_line_last_error;
        return;
    }

    g_line_valid = 1U;
    g_line_error = (int16_t) (sum / (int32_t) count);
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
    g_start_gap_encoder_seen =
        ((g_left_edges + g_right_edges) != 0U) ? 1U : 0U;
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
    g_start_gap_encoder_integral_error = 0;
    g_start_gap_encoder_p_term = 0;
    g_start_gap_encoder_i_term = 0;
    g_start_gap_encoder_d_term = 0;
    g_start_gap_encoder_target_correction = 0;
    g_start_gap_encoder_correction = 0;
    g_start_gap_left_forward_prev = 0;
    g_start_gap_right_forward_prev = 0;
    g_start_gap_left_forward_delta = 0;
    g_start_gap_right_forward_delta = 0;
    g_start_gap_encoder_delta_diff = 0;
    g_start_gap_encoder_speed_term = 0;
    g_start_gap_encoder_distance_term = 0;
    g_start_gap_left_cmd_ticks = START_GAP_LEFT_BASE_TICKS;
    g_start_gap_right_cmd_ticks = START_GAP_RIGHT_BASE_TICKS;
    g_start_gap_encoder_seen = 0U;
}

static NOINLINE void line_follow_update(void)
{
    int32_t error = (int32_t) g_line_error;
    bool sharp_turn = (abs_int32(error) >= LINE_SHARP_ERROR);

    if (sharp_turn) {
        g_turn_mode = 1U;
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

    g_turn_mode = 0U;
    g_line_correction = clamp_int32(
        error / LINE_KP_DIVISOR,
        -LINE_CORR_LIMIT,
        LINE_CORR_LIMIT);
    set_pwm(
        clamp_pwm((int32_t) PWM_LEFT_BASE_TICKS - g_line_correction),
        clamp_pwm((int32_t) PWM_RIGHT_BASE_TICKS + g_line_correction));
}

static NOINLINE void start_gap_bridge_update(void)
{
    int32_t target_correction;
    int32_t correction_delta;
    int32_t left_cmd;
    int32_t right_cmd;

    encoder_distance_update();
    g_start_gap_left_forward_delta =
        g_encoder_left_forward_total - g_start_gap_left_forward_prev;
    g_start_gap_right_forward_delta =
        g_encoder_right_forward_total - g_start_gap_right_forward_prev;
    g_start_gap_left_forward_prev = g_encoder_left_forward_total;
    g_start_gap_right_forward_prev = g_encoder_right_forward_total;
    g_start_gap_encoder_delta_diff =
        g_start_gap_left_forward_delta - g_start_gap_right_forward_delta;

    g_start_gap_encoder_integral_error = clamp_int32(
        g_start_gap_encoder_integral_error + g_encoder_diff_forward_total,
        -START_GAP_ENCODER_INTEGRAL_LIMIT,
        START_GAP_ENCODER_INTEGRAL_LIMIT);
    g_start_gap_encoder_p_term =
        g_encoder_diff_forward_total / START_GAP_ENCODER_DISTANCE_KP_DIV;
    g_start_gap_encoder_i_term =
        g_start_gap_encoder_integral_error / START_GAP_ENCODER_INTEGRAL_KI_DIV;
    g_start_gap_encoder_d_term =
        g_start_gap_encoder_delta_diff / START_GAP_ENCODER_SPEED_KD_DIV;
    g_start_gap_encoder_speed_term = g_start_gap_encoder_d_term;
    g_start_gap_encoder_distance_term =
        g_start_gap_encoder_p_term + g_start_gap_encoder_i_term;

    target_correction = clamp_int32(
        g_start_gap_encoder_p_term +
            g_start_gap_encoder_i_term +
            g_start_gap_encoder_d_term,
        -START_GAP_ENCODER_LIMIT,
        START_GAP_ENCODER_LIMIT);
    g_start_gap_encoder_target_correction = target_correction;
    correction_delta = clamp_int32(
        target_correction - g_start_gap_encoder_correction,
        -START_GAP_ENCODER_SLEW_LIMIT,
        START_GAP_ENCODER_SLEW_LIMIT);
    g_start_gap_encoder_correction += correction_delta;

    left_cmd = (int32_t) START_GAP_LEFT_BASE_TICKS +
        START_GAP_LEFT_TRIM_TICKS - g_start_gap_encoder_correction;
    right_cmd = (int32_t) START_GAP_RIGHT_BASE_TICKS +
        START_GAP_RIGHT_TRIM_TICKS + g_start_gap_encoder_correction;
    g_start_gap_left_cmd_ticks = clamp_pwm(left_cmd);
    g_start_gap_right_cmd_ticks = clamp_pwm(right_cmd);
    g_turn_mode = 4U;
    g_line_correction = g_start_gap_encoder_correction;
    set_pwm(
        g_start_gap_left_cmd_ticks,
        g_start_gap_right_cmd_ticks);
}

static NOINLINE void stop_with_reason(uint8_t reason)
{
    g_phase = PHASE_STOP;
    g_stop_reason = reason;
    motors_stop();
    buzzer_start_long();
}

static uint8_t omv_checksum(const uint8_t *buf)
{
    uint8_t value = 0U;
    for (uint8_t i = 2U; i <= 14U; i++) {
        value ^= buf[i];
    }
    return value;
}

static int16_t decode_s16(uint8_t low, uint8_t high)
{
    return (int16_t) ((uint16_t) low | ((uint16_t) high << 8));
}

static uint16_t decode_u16(uint8_t low, uint8_t high)
{
    return (uint16_t) low | ((uint16_t) high << 8);
}

static NOINLINE void omv_accept_frame(void)
{
    for (uint8_t i = 0U; i < OMV_FRAME_LENGTH; i++) {
        g_omv_last_frame[i] = s_rx_buf[i];
    }

    if (s_rx_buf[16] != OMV_FOOTER) {
        g_omv_bad_footer++;
        return;
    }

    if (s_rx_buf[15] != omv_checksum(s_rx_buf)) {
        g_omv_bad_checksum++;
        return;
    }

    g_omv_frame_type = s_rx_buf[2];
    g_omv_seq = decode_u16(s_rx_buf[3], s_rx_buf[4]);
    g_omv_flags = s_rx_buf[5];
    g_omv_camera_ok = (g_omv_flags & OMV_FLAG_CAMERA_OK) ? 1U : 0U;
    g_omv_target_detected = (g_omv_flags & OMV_FLAG_TARGET_DETECTED) ? 1U : 0U;
    g_omv_uart_rx_seen = (g_omv_flags & OMV_FLAG_UART_RX_SEEN) ? 1U : 0U;
    g_omv_cx = decode_s16(s_rx_buf[6], s_rx_buf[7]);
    g_omv_cy = decode_s16(s_rx_buf[8], s_rx_buf[9]);
    g_omv_pixels = decode_u16(s_rx_buf[10], s_rx_buf[11]);
    g_omv_fps_x10 = decode_u16(s_rx_buf[12], s_rx_buf[13]);
    g_omv_l_mean = s_rx_buf[14];

    g_omv_frame_count++;
    g_omv_link_ok = 1U;
    g_omv_timeout_frames = 0U;

    if (g_omv_frame_type == OMV_TYPE_COLOR) {
        g_omv_color_frame_count++;
    }
}

static NOINLINE void omv_parse_byte(uint8_t byte)
{
    g_omv_rx_bytes++;

    if (s_rx_idx == 0U) {
        if (byte == OMV_HEADER0) {
            s_rx_buf[s_rx_idx++] = byte;
        } else {
            g_omv_sync_drop++;
        }
        return;
    }

    if (s_rx_idx == 1U) {
        if (byte == OMV_HEADER1) {
            s_rx_buf[s_rx_idx++] = byte;
        } else {
            s_rx_idx = 0U;
            g_omv_sync_drop++;
            if (byte == OMV_HEADER0) {
                s_rx_buf[s_rx_idx++] = byte;
            }
        }
        return;
    }

    s_rx_buf[s_rx_idx++] = byte;
    if (s_rx_idx >= OMV_FRAME_LENGTH) {
        s_rx_idx = 0U;
        omv_accept_frame();
    }
}

void UART0_IRQHandler(void)
{
    switch (DL_UART_Main_getPendingInterrupt(UART_OPENMV)) {
        case DL_UART_MAIN_IIDX_RX:
            while (!DL_UART_Main_isRXFIFOEmpty(UART_OPENMV)) {
                omv_parse_byte(DL_UART_Main_receiveData(UART_OPENMV));
            }
            break;
        default:
            break;
    }
}

static NOINLINE void uart0_openmv_init(void)
{
    DL_UART_Main_reset(UART_OPENMV);
    DL_UART_Main_enablePower(UART_OPENMV);
    delay_cycles(16U);

    DL_GPIO_initPeripheralOutputFunction(IOMUX_PINCM21, IOMUX_PINCM21_PF_UART0_TX);
    DL_GPIO_initPeripheralInputFunction(IOMUX_PINCM22, IOMUX_PINCM22_PF_UART0_RX);

    DL_UART_Main_ClockConfig clock_cfg = {
        .clockSel = DL_UART_MAIN_CLOCK_BUSCLK,
        .divideRatio = DL_UART_MAIN_CLOCK_DIVIDE_RATIO_1,
    };
    DL_UART_Main_setClockConfig(UART_OPENMV, &clock_cfg);

    DL_UART_Main_Config uart_cfg = {
        .mode = DL_UART_MAIN_MODE_NORMAL,
        .direction = DL_UART_MAIN_DIRECTION_TX_RX,
        .flowControl = DL_UART_MAIN_FLOW_CONTROL_NONE,
        .parity = DL_UART_MAIN_PARITY_NONE,
        .wordLength = DL_UART_MAIN_WORD_LENGTH_8_BITS,
        .stopBits = DL_UART_MAIN_STOP_BITS_ONE,
    };
    DL_UART_Main_init(UART_OPENMV, &uart_cfg);
    DL_UART_Main_configBaudRate(UART_OPENMV, SYSCLK_HZ, UART_BAUDRATE);
    DL_UART_Main_setRXFIFOThreshold(UART_OPENMV, DL_UART_MAIN_RX_FIFO_LEVEL_ONE_ENTRY);
    DL_UART_Main_enableInterrupt(UART_OPENMV, DL_UART_MAIN_INTERRUPT_RX);
    DL_UART_Main_enable(UART_OPENMV);

    NVIC_ClearPendingIRQ(UART0_INT_IRQn);
    NVIC_EnableIRQ(UART0_INT_IRQn);
}

static int16_t aim_step_from_error(
    int16_t error,
    int16_t dead_pixels,
    int16_t divisor,
    int16_t max_step)
{
    int16_t magnitude = abs_i16(error);
    int16_t step;

    if (magnitude <= dead_pixels) {
        return 0;
    }

    step = (int16_t) (((magnitude - dead_pixels) / divisor) + 1);
    if (step > max_step) {
        step = max_step;
    }

    return (error > 0) ? step : (int16_t) -step;
}

static NOINLINE void aiming_update_from_latest_color_frame(void)
{
    int16_t dx;
    int16_t dy;
    int16_t raw_x_step;
    int16_t raw_y_step;
    int32_t next_x;
    int32_t next_y;

    if (g_omv_link_ok == 0U) {
        g_aim_state = 0U;
        g_aim_filter_ready = 0U;
        g_aim_target_confirm_count = 0U;
        g_aim_x_step_us = 0;
        g_aim_y_step_us = 0;
        return;
    }

    if ((g_omv_camera_ok == 0U) ||
        (g_omv_target_detected == 0U) ||
        (g_omv_pixels < AIM_MIN_PIXELS)) {
        g_no_target_frames++;
        g_aim_state = 1U;
        g_aim_filter_ready = 0U;
        g_aim_target_confirm_count = 0U;
        g_aim_x_step_us = 0;
        g_aim_y_step_us = 0;
        return;
    }

    g_no_target_frames = 0U;
    g_aim_raw_dx = (int16_t) (g_omv_cx - IMG_CENTER_X);
    g_aim_raw_dy = (int16_t) (g_omv_cy - IMG_CENTER_Y);

    if (g_aim_target_confirm_count < AIM_TARGET_CONFIRM_FRAMES) {
        g_aim_target_confirm_count++;
        g_aim_state = 7U;
        g_aim_x_step_us = 0;
        g_aim_y_step_us = 0;
        return;
    }

    if (g_aim_filter_ready == 0U) {
        g_aim_filtered_cx = g_omv_cx;
        g_aim_filtered_cy = g_omv_cy;
        g_aim_filter_ready = 1U;
    } else {
        g_aim_filtered_cx = (int16_t) (g_aim_filtered_cx +
            ((g_omv_cx - g_aim_filtered_cx) / AIM_FILTER_DIVISOR));
        g_aim_filtered_cy = (int16_t) (g_aim_filtered_cy +
            ((g_omv_cy - g_aim_filtered_cy) / AIM_FILTER_DIVISOR));
    }

    dx = (int16_t) (g_aim_filtered_cx - IMG_CENTER_X);
    dy = (int16_t) (g_aim_filtered_cy - IMG_CENTER_Y);
    g_aim_dx = dx;
    g_aim_dy = dy;

    raw_x_step = aim_step_from_error(
        dx, AIM_DEAD_X_PIXELS, AIM_X_DIVISOR, AIM_MAX_STEP_US);
    raw_y_step = aim_step_from_error(
        dy, AIM_DEAD_Y_PIXELS, AIM_Y_DIVISOR, AIM_MAX_STEP_US);

    g_aim_x_step_us = (int16_t) (AIM_X_PULSE_SIGN * raw_x_step);
    g_aim_y_step_us = (int16_t) (AIM_Y_PULSE_SIGN * raw_y_step);
    next_x = (int32_t) g_servo_x_us + g_aim_x_step_us;
    next_y = (int32_t) g_servo_y_us + g_aim_y_step_us;
    g_servo_x_us = clamp_servo(next_x, SERVO_X_MIN_US, SERVO_X_MAX_US);
    g_servo_y_us = clamp_servo(next_y, SERVO_Y_MIN_US, SERVO_Y_MAX_US);

    g_aim_state =
        ((g_aim_x_step_us == 0) && (g_aim_y_step_us == 0)) ? 3U : 2U;
    g_aim_update_count++;
}

static NOINLINE void servo_output_one_frame(void)
{
    uint16_t x_us = g_servo_x_us;
    uint16_t y_us = g_servo_y_us;

    DL_GPIO_setPins(SERVO_PORT, SERVO_X_PIN | SERVO_Y_PIN);

    if (x_us == y_us) {
        delay_us(x_us);
        DL_GPIO_clearPins(SERVO_PORT, SERVO_X_PIN | SERVO_Y_PIN);
        delay_us(SERVO_PERIOD_US - x_us);
    } else if (x_us < y_us) {
        delay_us(x_us);
        DL_GPIO_clearPins(SERVO_PORT, SERVO_X_PIN);
        delay_us((uint32_t) y_us - x_us);
        DL_GPIO_clearPins(SERVO_PORT, SERVO_Y_PIN);
        delay_us(SERVO_PERIOD_US - y_us);
    } else {
        delay_us(y_us);
        DL_GPIO_clearPins(SERVO_PORT, SERVO_Y_PIN);
        delay_us((uint32_t) x_us - y_us);
        DL_GPIO_clearPins(SERVO_PORT, SERVO_X_PIN);
        delay_us(SERVO_PERIOD_US - x_us);
    }

    g_servo_frame_count++;
}

static NOINLINE void servo_hold_home_on_startup(void)
{
    g_servo_x_us = SERVO_X_HOME_US;
    g_servo_y_us = SERVO_Y_HOME_US;
    g_aim_state = 4U;

    for (uint32_t i = 0U; i < SERVO_STARTUP_HOME_FRAMES / 2U; i++) {
        servo_output_one_frame();
        buzzer_update_one_frame();
    }

    g_servo_x_us = clamp_servo(
        (int32_t) SERVO_X_HOME_US + SERVO_STARTUP_TEST_X_OFFSET,
        SERVO_X_MIN_US,
        SERVO_X_MAX_US);
    g_servo_y_us = clamp_servo(
        (int32_t) SERVO_Y_HOME_US + SERVO_STARTUP_TEST_Y_OFFSET,
        SERVO_Y_MIN_US,
        SERVO_Y_MAX_US);
    for (uint32_t i = 0U; i < SERVO_STARTUP_TEST_FRAMES; i++) {
        servo_output_one_frame();
        buzzer_update_one_frame();
    }

    g_servo_x_us = clamp_servo(
        (int32_t) SERVO_X_HOME_US - SERVO_STARTUP_TEST_X_OFFSET,
        SERVO_X_MIN_US,
        SERVO_X_MAX_US);
    g_servo_y_us = clamp_servo(
        (int32_t) SERVO_Y_HOME_US - SERVO_STARTUP_TEST_Y_OFFSET,
        SERVO_Y_MIN_US,
        SERVO_Y_MAX_US);
    for (uint32_t i = 0U; i < SERVO_STARTUP_TEST_FRAMES; i++) {
        servo_output_one_frame();
        buzzer_update_one_frame();
    }

    g_servo_x_us = SERVO_X_HOME_US;
    g_servo_y_us = SERVO_Y_HOME_US;
    for (uint32_t i = 0U; i < SERVO_STARTUP_HOME_FRAMES / 2U; i++) {
        servo_output_one_frame();
        buzzer_update_one_frame();
    }
    g_servo_startup_home_done = 1U;
}

static NOINLINE void encoder_input_init(void)
{
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
    NVIC_ClearPendingIRQ(GPIOB_INT_IRQn);
    NVIC_EnableIRQ(GPIOB_INT_IRQn);
}

static NOINLINE void hardware_init(void)
{
    SYSCFG_DL_init();

    init_output(IOMUX_PINCM12, MOTOR_PORT, AIN1_PIN, false);
    init_output(IOMUX_PINCM13, MOTOR_PORT, AIN2_PIN, false);
    init_output(IOMUX_PINCM15, MOTOR_PORT, BIN1_PIN, false);
    init_output(IOMUX_PINCM16, MOTOR_PORT, BIN2_PIN, false);
    init_output(IOMUX_PINCM17, MOTOR_PORT, STBY_PIN, true);
    init_output(IOMUX_PINCM37, LED_PORT, LED_PIN, false);
    init_output(IOMUX_PINCM28, BUZZER_PORT, BUZZER_PIN, false);
    init_output(IOMUX_PINCM36, SERVO_PORT, SERVO_X_PIN, false);
    init_output(IOMUX_PINCM39, SERVO_PORT, SERVO_Y_PIN, false);

    init_input(IOMUX_PINCM18);
    init_input(IOMUX_PINCM23);
    init_input(IOMUX_PINCM24);
    init_input(IOMUX_PINCM14);
    init_input(IOMUX_PINCM19);
    init_input(IOMUX_PINCM26);
    init_input(IOMUX_PINCM27);

    encoder_input_init();
    pwm_init();
    uart0_openmv_init();
    motors_stop();
}

static NOINLINE void enter_bridge(uint8_t reason)
{
    g_phase = PHASE_START_GAP_BRIDGE;
    g_state_switch_reason = reason;
    g_state_switch_event_count++;
    g_start_gap_frames = 0U;
    reset_line_history();
    encoder_reset_counts();
    led_set(false);
    buzzer_start_beeps(2U);
    start_gap_bridge_update();
    motors_forward();
}

static NOINLINE void enter_line_run(void)
{
    uint8_t entry_mask =
        (g_line_entry_mask != 0U) ?
            (uint8_t) (g_line_entry_mask &
                (uint8_t) (~g_line_start_ignore_mask)) :
            g_line_effective_detect_mask;

    g_phase = PHASE_LINE_RUN;
    g_state_switch_reason = SWITCH_REASON_ENTER_LINE;
    g_state_switch_event_count++;
    g_reacquire_event_count++;
    g_line_confirmed_entry_count++;
    reset_line_history();
    g_line_confirmed_entry_mask = entry_mask;
    g_line_entry_control_frames_left = LINE_ENTRY_CONTROL_FRAMES;
    led_set(true);
    buzzer_start_beeps(3U);
    line_follow_update();
    motors_forward();
}

static NOINLINE void update_openmv_and_servo(uint32_t *last_color_count, uint32_t *last_frame_count)
{
    servo_output_one_frame();
    buzzer_update_one_frame();

    if (g_omv_color_frame_count != *last_color_count) {
        *last_color_count = g_omv_color_frame_count;
        aiming_update_from_latest_color_frame();
    }

    if (g_omv_frame_count != *last_frame_count) {
        *last_frame_count = g_omv_frame_count;
        g_omv_timeout_frames = 0U;
    } else if (g_omv_timeout_frames < 0xFFFFFFFFU) {
        g_omv_timeout_frames++;
        if (g_omv_timeout_frames > LINK_TIMEOUT_FRAMES) {
            g_omv_link_ok = 0U;
            g_aim_state = 0U;
        }
    }
}

int main(void)
{
    uint32_t last_color_count = 0U;
    uint32_t last_frame_count = 0U;

    hardware_init();
    g_phase = PHASE_STARTUP;
    buzzer_start_beeps(1U);
    servo_hold_home_on_startup();

    read_line_sensors();
    g_startup_ignore_active_count = count_bits7(g_line_detect_mask);
#if LINE_START_IGNORE_ENABLE
    if ((g_line_detect_mask != 0U) &&
        (g_startup_ignore_active_count <= LINE_START_IGNORE_MAX_ACTIVE)) {
        g_line_start_ignore_mask = g_line_detect_mask;
    } else {
        g_line_start_ignore_mask = 0U;
    }
#endif
    g_line_effective_detect_mask =
        (uint8_t) (g_line_detect_mask & (uint8_t) (~g_line_start_ignore_mask));
    g_startup_direct_line_allowed =
        ((LINE_START_DIRECT_ENABLE != 0U) &&
         (count_bits7(g_line_effective_detect_mask) >=
            LINE_START_DIRECT_MIN_ACTIVE)) ? 1U : 0U;
    update_line_position();
    if (g_startup_direct_line_allowed != 0U) {
        enter_line_run();
    } else {
        enter_bridge(SWITCH_REASON_START_BRIDGE);
    }

    last_color_count = g_omv_color_frame_count;
    last_frame_count = g_omv_frame_count;

    while (1) {
        update_openmv_and_servo(&last_color_count, &last_frame_count);

        if (g_phase == PHASE_STOP || g_phase == PHASE_DONE) {
            motors_stop();
            continue;
        }

        read_line_sensors_for_control();
        update_line_position();

        if ((SENSOR_ALL_LOW_STOP_FRAMES != 0U) &&
            (g_sensor_all_low_count >= SENSOR_ALL_LOW_STOP_FRAMES)) {
            stop_with_reason(STOP_REASON_SENSOR_ALL_LOW);
            continue;
        }

        if (g_phase == PHASE_START_GAP_BRIDGE) {
            if (g_line_entry_mask != 0U) {
                enter_line_run();
                continue;
            }

            if (g_start_gap_frames >= START_GAP_MAX_FRAMES) {
                stop_with_reason(STOP_REASON_START_GAP);
                continue;
            }

            g_start_gap_frames++;
            start_gap_bridge_update();
            motors_forward();
        } else if (g_phase == PHASE_LINE_RUN) {
            if ((g_line_effective_detect_mask == 0U) &&
                (g_line_entry_control_frames_left == 0U)) {
                if (g_line_exit_missing_count < 0xFFU) {
                    g_line_exit_missing_count++;
                }
            } else {
                g_line_exit_missing_count = 0U;
            }

            if (g_line_exit_missing_count >= LINE_EXIT_CONFIRM_FRAMES) {
                g_line_exit_confirmed_count++;
                enter_bridge(SWITCH_REASON_EXIT_LINE);
                continue;
            }

            line_follow_update();
            motors_forward();
            if (g_line_effective_detect_mask != 0U) {
                g_line_entry_control_frames_left = 0U;
            } else if (g_line_entry_control_frames_left > 0U) {
                g_line_entry_control_frames_left--;
            }
            g_run_frame++;

            if (g_run_frame >= RUN_MAX_FRAMES) {
                g_phase = PHASE_DONE;
                g_stop_reason = STOP_REASON_RUN_TIMEOUT;
                motors_stop();
                continue;
            }
        } else {
            enter_bridge(SWITCH_REASON_START_BRIDGE);
        }
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
