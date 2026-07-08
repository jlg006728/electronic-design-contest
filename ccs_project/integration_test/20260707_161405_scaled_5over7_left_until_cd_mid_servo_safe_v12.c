/*
 * MSPM0G3507 + line following + OpenMV dual-servo aiming integration test.
 *
 * Goal:
 *   Keep the verified line-following chassis running while the PA14/PA17
 *   gimbal tracks red target coordinates streamed by OpenMV.
 *
 * OpenMV script:
 *   ccs_project/openmv_test/
 *   20260706_163540_openmv_uart_stream_edge_guard_rot180.py
 *
 * Wiring:
 *   MSPM0 PA10 / UART0_TX -> OpenMV P5 / UART3_RX
 *   MSPM0 PA11 / UART0_RX <- OpenMV P4 / UART3_TX
 *   MSPM0 PA14            -> horizontal/yaw servo signal
 *   MSPM0 PA17            -> pitch servo signal
 *   MSPM0 GND             -> OpenMV GND, servo GND, TB6612 GND
 *
 * Power:
 *   Servos: stable 5V from TB6612 5V output or dedicated 5V supply.
 *   OpenMV: USB or stable 5V VIN. Do not power OpenMV from MSPM0 3V3.
 *
 * Safety:
 *   First integration version only combines line following and gimbal aiming.
 *   No firing/laser output is controlled here.
 *
 * If direction is reversed:
 *   Change AIM_X_PULSE_SIGN or AIM_Y_PULSE_SIGN below from 1 to -1, or back.
 */

#include <stdbool.h>
#include <stdint.h>
#include "ti_msp_dl_config.h"

#define NOINLINE                    __attribute__((noinline))

#define SYSCLK_HZ                   32000000U
#define UART_OPENMV                 UART0
#define UART_BAUDRATE               115200U

#define OMV_HEADER0                 0xAAU
#define OMV_HEADER1                 0x55U
#define OMV_FOOTER                  0x5BU
#define OMV_FRAME_LENGTH            17U

#define OMV_CMD_COLOR               ((uint8_t) 'C')
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
#define AIM_EDGE_MARGIN_X           18
#define AIM_EDGE_MARGIN_Y           10
#define AIM_SEARCH_START_FRAMES     6U
#define AIM_SEARCH_STEP_US          35
#define AIM_SEARCH_Y_HOME_STEP_US   4U

#define SERVO_PERIOD_US             20000U
#define SERVO_X_HOME_US             1600U
#define SERVO_Y_HOME_US             1500U
#define SERVO_STARTUP_HOME_FRAMES   100U
/*
 * Keep runtime limits aligned with the calibration range. The previous
 * X minimum of 900us clipped the calibrated X home of 850us.
 */
#define SERVO_X_MIN_US              350U
#define SERVO_X_MAX_US              2650U
#define SERVO_Y_MIN_US              500U
#define SERVO_Y_MAX_US              2500U
#define SERVO_X_CALIB_MIN_US        500U
#define SERVO_X_CALIB_MAX_US        2500U
#define SERVO_Y_CALIB_MIN_US        500U
#define SERVO_Y_CALIB_MAX_US        2500U

#define SERVO_CALIBRATION_MODE         0U
#define OPENMV_ENABLE_REQUESTS         0U
#define OPENMV_REQUEST_INTERVAL_FRAMES 4U
#define LINK_TIMEOUT_FRAMES            120U
#define LOST_RECENTER_FRAMES           40U

#define DIAG_STAGE_NO_LINK             0U
#define DIAG_STAGE_FRAME_RX            1U
#define DIAG_STAGE_NO_TARGET           2U
#define DIAG_STAGE_SMALL_TARGET        3U
#define DIAG_STAGE_CONFIRM_WAIT        4U
#define DIAG_STAGE_DEADBAND            5U
#define DIAG_STAGE_SERVO_UPDATE        6U
#define DIAG_STAGE_LIMITED             7U
#define DIAG_STAGE_EDGE_TARGET         8U
#define DIAG_STAGE_SEARCH_TARGET       9U
#define DIAG_STAGE_TURN_HOLD           10U

#define LED_PORT                    GPIOA
#define LED_PIN                     DL_GPIO_PIN_15

#define BUZZER_PORT                 GPIOB
#define BUZZER_PIN                  DL_GPIO_PIN_11
#define BUZZER_ON_FRAMES            5U
#define BUZZER_OFF_FRAMES           5U
#define BUZZER_LONG_FRAMES          50U

#define SERVO_PORT                  GPIOA
#define SERVO_X_PIN                 DL_GPIO_PIN_14
#define SERVO_Y_PIN                 DL_GPIO_PIN_17

#define MOTOR_PORT                  GPIOB
#define AIN1_PIN                    DL_GPIO_PIN_0
#define AIN2_PIN                    DL_GPIO_PIN_1
#define BIN1_PIN                    DL_GPIO_PIN_2
#define BIN2_PIN                    DL_GPIO_PIN_3
#define STBY_PIN                    DL_GPIO_PIN_4

#define PWM_TIMER                   TIMG0
#define PWM_PERIOD                  1600U
#define PWM_LEFT_BASE_TICKS         403U
#define PWM_RIGHT_BASE_TICKS        403U
#define PWM_LEFT_BOOST_TICKS        403U
#define PWM_RIGHT_BOOST_TICKS       403U
#define PWM_MIN_TICKS               287U
#define PWM_MAX_TICKS               567U
#define PWM_SHARP_LEFT_BASE_TICKS   403U
#define PWM_SHARP_RIGHT_BASE_TICKS  403U
#define PWM_SHARP_MAX_TICKS         567U
#define START_BOOST_SAMPLES         0U
#define LINE_KP_DIVISOR             20
#define LINE_CORR_LIMIT             150
#define LINE_CENTER_OFFSET          0
#define LINE_SHARP_ERROR            1800
#define LINE_SHARP_KP_DIVISOR       8
#define LINE_SHARP_CORR_LIMIT       380
#define LINE_RUN_SAMPLES            4500U
#define LINE_LOST_STOP_SAMPLES      100U
#define MISSION_ENABLE_B_AIM        0U
#define MISSION_ENABLE_POINT_PROMPTS 0U
#define MISSION_ENABLE_POINT_STOPS  1U
#define FULL_TRACK_SEARCH_DURING_TURN 1U
#define FULL_TRACK_ACCEPT_EDGE_TARGET 1U
#define COURSE_PREAIM_ENABLE        1U
#define COURSE_PREAIM_LEFT_HOLD     1U
#define COURSE_PREAIM_RETURN_HOME   2U
#define COURSE_LAP_SAMPLES          914U
#define COURSE_LEFT_HOLD_SAMPLE     559U
#define COURSE_LEFT_TARGET_US       2550U
#define COURSE_HOME_TARGET_US       SERVO_X_HOME_US
#define COURSE_PREAIM_STEP_US       35
#define COURSE_POINT_HOLD_FRAMES    100U
#define COURSE_POINT_B_SAMPLE       203U
#define COURSE_POINT_C_SAMPLE       457U
#define COURSE_POINT_D_SAMPLE       660U
#define COURSE_POINT_A_SAMPLE       COURSE_LAP_SAMPLES
#define MISSION_B_STOP_SAMPLE       180U
#define MISSION_C_PROMPT_SAMPLE     360U
#define MISSION_D_PROMPT_SAMPLE     540U
#define MISSION_A_FINISH_SAMPLE     720U
#define MISSION_AIM_MAX_FRAMES      350U
#define MISSION_AIM_STABLE_FRAMES   20U
#define MISSION_POINT_PROMPT_FRAMES 40U
#define MISSION_POINT_TOGGLE_FRAMES 5U
#define MISSION_B_AIM_X_PRESET_US   2450U
#define MISSION_B_AIM_Y_PRESET_US   SERVO_Y_HOME_US
#define MISSION_B_AIM_SEARCH_DIR    1
#define PHASE_LINE_RUN              2U
#define PHASE_MISSION_AIM           6U
#define PHASE_POINT_HOLD            7U

#define S1_PIN                      DL_GPIO_PIN_5
#define S2_PIN                      DL_GPIO_PIN_6
#define S3_PIN                      DL_GPIO_PIN_7
#define S4_PIN                      DL_GPIO_PIN_7
#define S5_PIN                      DL_GPIO_PIN_8
#define S6_PIN                      DL_GPIO_PIN_9
#define S7_PIN                      DL_GPIO_PIN_10

#define LINE_BLACK_IS_HIGH          1

volatile uint32_t g_omv_tx_requests = 0;
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

volatile uint16_t g_servo_x_home_us = SERVO_X_HOME_US;
volatile uint16_t g_servo_y_home_us = SERVO_Y_HOME_US;
volatile uint16_t g_servo_x_us = SERVO_X_HOME_US;
volatile uint16_t g_servo_y_us = SERVO_Y_HOME_US;
volatile uint32_t g_servo_frame_count = 0;
volatile uint32_t g_servo_startup_home_frames = 0;
volatile uint32_t g_aim_update_count = 0;
volatile uint32_t g_no_target_frames = 0;
volatile uint32_t g_aim_rejected_small_target = 0;
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
volatile uint8_t g_aim_confident_target = 0;
volatile uint8_t g_aim_search_active = 0;
volatile int8_t g_aim_search_dir = 1;
volatile uint8_t g_servo_startup_home_done = 0;
volatile uint8_t g_servo_calibrate_enable = 0U;
volatile uint8_t g_led_state = 0;
volatile uint8_t g_diag_stage = DIAG_STAGE_NO_LINK;
volatile uint8_t g_diag_x_limited = 0;
volatile uint8_t g_diag_y_limited = 0;
volatile uint32_t g_diag_no_link_count = 0;
volatile uint32_t g_diag_no_target_count = 0;
volatile uint32_t g_diag_small_target_count = 0;
volatile uint32_t g_diag_confirm_wait_count = 0;
volatile uint32_t g_diag_deadband_count = 0;
volatile uint32_t g_diag_servo_update_count = 0;
volatile uint32_t g_diag_limited_count = 0;
volatile uint32_t g_diag_edge_target_count = 0;
volatile uint32_t g_diag_search_target_count = 0;
volatile uint32_t g_diag_turn_hold_count = 0;

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
volatile uint8_t g_start_boost_active = 0;
volatile uint8_t g_phase = 0;
volatile uint8_t g_stop_reason = 0;
volatile uint8_t g_mission_phase = 0;
volatile uint8_t g_mission_b_done = 0;
volatile uint8_t g_mission_c_done = 0;
volatile uint8_t g_mission_d_done = 0;
volatile uint8_t g_mission_a_done = 0;
volatile uint8_t g_mission_aim_success = 0;
volatile uint8_t g_mission_b_aim_preset_applied = 0;
volatile uint8_t g_mission_last_point = 0;
volatile uint32_t g_mission_aim_frames = 0;
volatile uint32_t g_mission_aim_stable_frames = 0;
volatile uint32_t g_mission_prompt_frames = 0;
volatile uint8_t g_course_preaim_phase = COURSE_PREAIM_LEFT_HOLD;
volatile uint16_t g_course_preaim_x_us = SERVO_X_HOME_US;
volatile uint32_t g_course_lap_sample = 0;
volatile int16_t g_course_aim_offset_us = 0;
volatile uint8_t g_point_stop_next_index = 0;
volatile uint8_t g_point_hold_index = 0;
volatile uint8_t g_point_hold_id = 0;
volatile uint16_t g_point_hold_frames_left = 0;
volatile uint32_t g_point_stop_lap_base = 0;
volatile uint32_t g_point_stop_target_sample = 0;
volatile uint16_t g_point_forced_x_us = SERVO_X_HOME_US;
volatile uint8_t g_point_hold_target_locked = 0;
volatile uint8_t g_buzzer_mode = 0;
volatile uint8_t g_buzzer_output_on = 0;
volatile uint8_t g_buzzer_beeps_left = 0;
volatile uint16_t g_buzzer_frames_left = 0;
volatile uint32_t g_buzzer_event_count = 0;

static uint8_t s_rx_buf[OMV_FRAME_LENGTH];
static uint8_t s_rx_idx = 0;

static NOINLINE void servo_output_one_frame(void);

static NOINLINE void delay_us(uint32_t us)
{
    if (us == 0U) {
        return;
    }
    delay_cycles(us * 32U);
}

static NOINLINE void delay_ms(uint32_t ms)
{
    for (uint32_t i = 0; i < ms; i++) {
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

static NOINLINE void safe_motor_outputs_init(void)
{
    init_output(IOMUX_PINCM12, MOTOR_PORT, AIN1_PIN, false);
    init_output(IOMUX_PINCM13, MOTOR_PORT, AIN2_PIN, false);
    init_output(IOMUX_PINCM15, MOTOR_PORT, BIN1_PIN, false);
    init_output(IOMUX_PINCM16, MOTOR_PORT, BIN2_PIN, false);
    init_output(IOMUX_PINCM17, MOTOR_PORT, STBY_PIN, false);
}

static NOINLINE void line_inputs_init(void)
{
    init_input(IOMUX_PINCM18);
    init_input(IOMUX_PINCM23);
    init_input(IOMUX_PINCM24);
    init_input(IOMUX_PINCM14);
    init_input(IOMUX_PINCM19);
    init_input(IOMUX_PINCM26);
    init_input(IOMUX_PINCM27);
}

static NOINLINE void servo_outputs_init(void)
{
    init_output(IOMUX_PINCM36, SERVO_PORT, SERVO_X_PIN, false);
    init_output(IOMUX_PINCM39, SERVO_PORT, SERVO_Y_PIN, false);
}

static NOINLINE void buzzer_output_init(void)
{
    init_output(IOMUX_PINCM28, BUZZER_PORT, BUZZER_PIN, false);
}

static NOINLINE void led_set(bool on)
{
    if (on) {
        DL_GPIO_setPins(LED_PORT, LED_PIN);
        g_led_state = 1U;
    } else {
        DL_GPIO_clearPins(LED_PORT, LED_PIN);
        g_led_state = 0U;
    }
}

static NOINLINE void led_toggle(void)
{
    DL_GPIO_togglePins(LED_PORT, LED_PIN);
    g_led_state = (g_led_state == 0U) ? 1U : 0U;
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

    buzzer_set(false);
    g_buzzer_mode = 1U;
    g_buzzer_beeps_left = count;
    g_buzzer_frames_left = 0U;
    g_buzzer_event_count++;
}

static NOINLINE void buzzer_start_long(void)
{
    g_buzzer_mode = 2U;
    g_buzzer_beeps_left = 0U;
    g_buzzer_frames_left = BUZZER_LONG_FRAMES;
    buzzer_set(true);
    g_buzzer_event_count++;
}

static NOINLINE void buzzer_update_one_frame(void)
{
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

static NOINLINE void startup_blink(void)
{
    for (uint8_t i = 0; i < 4U; i++) {
        led_set(true);
        delay_ms(70U);
        led_set(false);
        delay_ms(70U);
    }
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

static uint32_t clamp_pwm_sharp(int32_t ticks)
{
    if (ticks <= 0) {
        return 0U;
    }
    if (ticks > (int32_t) PWM_SHARP_MAX_TICKS) {
        return PWM_SHARP_MAX_TICKS;
    }
    return (uint32_t) ticks;
}

static uint32_t current_left_base_ticks(void)
{
    return (g_run_sample < START_BOOST_SAMPLES) ?
        PWM_LEFT_BOOST_TICKS : PWM_LEFT_BASE_TICKS;
}

static uint32_t current_right_base_ticks(void)
{
    return (g_run_sample < START_BOOST_SAMPLES) ?
        PWM_RIGHT_BOOST_TICKS : PWM_RIGHT_BASE_TICKS;
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
    g_left_pwm_ticks = left_duty_ticks;
    g_right_pwm_ticks = right_duty_ticks;
    g_left_compare_ticks = PWM_PERIOD - left_duty_ticks;
    g_right_compare_ticks = PWM_PERIOD - right_duty_ticks;

    DL_TimerG_setCaptureCompareValue(PWM_TIMER, g_left_compare_ticks, DL_TIMER_CC_0_INDEX);
    DL_TimerG_setCaptureCompareValue(PWM_TIMER, g_right_compare_ticks, DL_TIMER_CC_1_INDEX);
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

static NOINLINE void read_line_sensors(void)
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
    g_line_raw_mask |= (uint8_t) (g_s1 << 0);
    g_line_raw_mask |= (uint8_t) (g_s2 << 1);
    g_line_raw_mask |= (uint8_t) (g_s3 << 2);
    g_line_raw_mask |= (uint8_t) (g_s4 << 3);
    g_line_raw_mask |= (uint8_t) (g_s5 << 4);
    g_line_raw_mask |= (uint8_t) (g_s6 << 5);
    g_line_raw_mask |= (uint8_t) (g_s7 << 6);
}

static NOINLINE void update_line_position(void)
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
    g_line_error = (int16_t) ((sum / (int32_t) count) - LINE_CENTER_OFFSET);
    g_line_last_error = g_line_error;
}

static NOINLINE void line_follow_update(void)
{
    int32_t error = (int32_t) g_line_error;
    bool lost_recovery = (g_line_valid == 0U) && (g_line_last_error != 0);
    bool sharp_turn = (abs_int32(error) >= LINE_SHARP_ERROR) || lost_recovery;
    uint32_t left_base = current_left_base_ticks();
    uint32_t right_base = current_right_base_ticks();

    g_start_boost_active = (g_run_sample < START_BOOST_SAMPLES) ? 1U : 0U;

    if ((g_line_valid == 0U) && (g_line_last_error == 0)) {
        g_turn_mode = 3U;
        g_line_correction = 0;
        set_pwm(left_base, right_base);
        return;
    }

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
            clamp_pwm_sharp((int32_t) PWM_SHARP_LEFT_BASE_TICKS - g_line_correction),
            clamp_pwm_sharp((int32_t) PWM_SHARP_RIGHT_BASE_TICKS + g_line_correction));
        return;
    }

    g_line_correction = clamp_int32(
        error / LINE_KP_DIVISOR,
        -LINE_CORR_LIMIT,
        LINE_CORR_LIMIT);

    set_pwm(
        clamp_pwm((int32_t) left_base - g_line_correction),
        clamp_pwm((int32_t) right_base + g_line_correction));
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
        g_diag_stage = DIAG_STAGE_FRAME_RX;
        led_toggle();
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

static NOINLINE void omv_send_color_request(void)
{
    DL_UART_Main_transmitDataBlocking(UART_OPENMV, OMV_CMD_COLOR);
    g_omv_tx_requests++;
}

static int16_t abs_i16(int16_t value)
{
    return (value < 0) ? (int16_t) -value : value;
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

static bool aim_target_near_edge(void)
{
    return (g_omv_cx <= AIM_EDGE_MARGIN_X) ||
        (g_omv_cx >= (IMG_WIDTH - AIM_EDGE_MARGIN_X)) ||
        (g_omv_cy <= AIM_EDGE_MARGIN_Y) ||
        (g_omv_cy >= (IMG_HEIGHT - AIM_EDGE_MARGIN_Y));
}

static void aim_update_search_dir_from_dx(int16_t dx)
{
    if (dx > 0) {
        g_aim_search_dir = (int8_t) AIM_X_PULSE_SIGN;
    } else if (dx < 0) {
        g_aim_search_dir = (int8_t) (-AIM_X_PULSE_SIGN);
    }
}

static bool aim_should_hold_for_chassis_turn(void)
{
    if (FULL_TRACK_SEARCH_DURING_TURN != 0U) {
        return false;
    }

    if (g_phase == PHASE_MISSION_AIM) {
        return false;
    }

    return (g_turn_mode == 1U) || (g_turn_mode == 2U);
}

static uint16_t clamp_course_preaim_x(int32_t value)
{
    int32_t home = (int32_t) COURSE_HOME_TARGET_US;
    int32_t left = (int32_t) COURSE_LEFT_TARGET_US;
    int32_t low = (home < left) ? home : left;
    int32_t high = (home > left) ? home : left;

    return clamp_servo(value, (uint16_t) low, (uint16_t) high);
}

static uint8_t course_preaim_phase_from_sample(void)
{
    uint32_t lap_sample = g_run_sample % COURSE_LAP_SAMPLES;

    g_course_lap_sample = lap_sample;
    if (lap_sample < COURSE_LEFT_HOLD_SAMPLE) {
        return COURSE_PREAIM_LEFT_HOLD;
    }
    return COURSE_PREAIM_RETURN_HOME;
}

static uint16_t course_preaim_target_from_lap_sample(uint32_t lap_sample)
{
    int32_t home = (int32_t) COURSE_HOME_TARGET_US;
    int32_t left = (int32_t) COURSE_LEFT_TARGET_US;
    int32_t offset = left - home;
    int32_t target = left;

    if (lap_sample >= COURSE_LEFT_HOLD_SAMPLE) {
        uint32_t return_span = COURSE_LAP_SAMPLES - COURSE_LEFT_HOLD_SAMPLE;
        uint32_t return_sample = lap_sample - COURSE_LEFT_HOLD_SAMPLE;
        int32_t remaining = 0;

        if (return_span == 0U) {
            target = home;
            offset = 0;
        } else {
            if (return_sample < return_span) {
                remaining = (int32_t) (return_span - return_sample);
            }
            offset = (offset * remaining) / (int32_t) return_span;
            target = home + offset;
        }
    }

    return clamp_course_preaim_x(target);
}

static uint16_t course_preaim_target_x_us(void)
{
    uint8_t phase = course_preaim_phase_from_sample();
    int32_t home = (int32_t) COURSE_HOME_TARGET_US;

    g_course_preaim_phase = phase;
    g_course_preaim_x_us = course_preaim_target_from_lap_sample(g_course_lap_sample);
    g_course_aim_offset_us = (int16_t) ((int32_t) g_course_preaim_x_us - home);
    return g_course_preaim_x_us;
}

static NOINLINE void aim_search_for_lost_target(void)
{
    uint16_t old_x = g_servo_x_us;
    uint16_t old_y = g_servo_y_us;
    int32_t next_x;
    int32_t next_y = (int32_t) g_servo_y_us;

    if (COURSE_PREAIM_ENABLE != 0U) {
        uint16_t target_x = (g_phase == PHASE_POINT_HOLD) ?
            g_point_forced_x_us : course_preaim_target_x_us();

        if (g_servo_x_us + COURSE_PREAIM_STEP_US < target_x) {
            next_x = (int32_t) g_servo_x_us + COURSE_PREAIM_STEP_US;
        } else if (g_servo_x_us > target_x + COURSE_PREAIM_STEP_US) {
            next_x = (int32_t) g_servo_x_us - COURSE_PREAIM_STEP_US;
        } else {
            next_x = target_x;
        }

        if (g_servo_y_us + AIM_SEARCH_Y_HOME_STEP_US < g_servo_y_home_us) {
            next_y = (int32_t) g_servo_y_us + AIM_SEARCH_Y_HOME_STEP_US;
        } else if (g_servo_y_us > g_servo_y_home_us + AIM_SEARCH_Y_HOME_STEP_US) {
            next_y = (int32_t) g_servo_y_us - AIM_SEARCH_Y_HOME_STEP_US;
        } else {
            next_y = g_servo_y_home_us;
        }

        g_servo_x_us = clamp_servo(next_x, SERVO_X_MIN_US, SERVO_X_MAX_US);
        g_servo_y_us = clamp_servo(next_y, SERVO_Y_MIN_US, SERVO_Y_MAX_US);
        g_aim_x_step_us = (int16_t) ((int32_t) g_servo_x_us - old_x);
        g_aim_y_step_us = (int16_t) ((int32_t) g_servo_y_us - old_y);
        if (g_aim_x_step_us > 0) {
            g_aim_search_dir = 1;
        } else if (g_aim_x_step_us < 0) {
            g_aim_search_dir = -1;
        }
        g_aim_search_active = 1U;
        g_aim_state = 9U;
        g_diag_stage = DIAG_STAGE_SEARCH_TARGET;
        g_diag_search_target_count++;
        return;
    }

    if (g_aim_search_dir == 0) {
        g_aim_search_dir = 1;
    }

    next_x = (int32_t) g_servo_x_us +
        ((int32_t) g_aim_search_dir * AIM_SEARCH_STEP_US);

    if (next_x <= (int32_t) SERVO_X_MIN_US) {
        next_x = SERVO_X_MIN_US;
        g_aim_search_dir = 1;
    } else if (next_x >= (int32_t) SERVO_X_MAX_US) {
        next_x = SERVO_X_MAX_US;
        g_aim_search_dir = -1;
    }

    if (g_servo_y_us + AIM_SEARCH_Y_HOME_STEP_US < g_servo_y_home_us) {
        next_y = (int32_t) g_servo_y_us + AIM_SEARCH_Y_HOME_STEP_US;
    } else if (g_servo_y_us > g_servo_y_home_us + AIM_SEARCH_Y_HOME_STEP_US) {
        next_y = (int32_t) g_servo_y_us - AIM_SEARCH_Y_HOME_STEP_US;
    } else {
        next_y = g_servo_y_home_us;
    }

    g_servo_x_us = clamp_servo(next_x, SERVO_X_MIN_US, SERVO_X_MAX_US);
    g_servo_y_us = clamp_servo(next_y, SERVO_Y_MIN_US, SERVO_Y_MAX_US);
    g_aim_x_step_us = (int16_t) ((int32_t) g_servo_x_us - old_x);
    g_aim_y_step_us = (int16_t) ((int32_t) g_servo_y_us - old_y);
    g_aim_search_active = 1U;
    g_aim_state = 9U;
    g_diag_stage = DIAG_STAGE_SEARCH_TARGET;
    g_diag_search_target_count++;
}

static NOINLINE void aim_handle_unusable_target(uint8_t hold_state, uint8_t hold_diag)
{
    g_no_target_frames++;
    g_aim_target_confirm_count = 0U;
    g_aim_confident_target = 0U;
    g_aim_x_step_us = 0;
    g_aim_y_step_us = 0;

    if (g_no_target_frames > LOST_RECENTER_FRAMES) {
        g_aim_filter_ready = 0U;
    }

    if (aim_should_hold_for_chassis_turn()) {
        g_aim_search_active = 0U;
        g_aim_state = 10U;
        g_diag_stage = DIAG_STAGE_TURN_HOLD;
        g_diag_turn_hold_count++;
        return;
    }

    if (g_no_target_frames < AIM_SEARCH_START_FRAMES) {
        g_aim_search_active = 0U;
        g_aim_state = hold_state;
        g_diag_stage = hold_diag;
        return;
    }

    aim_search_for_lost_target();
}

static NOINLINE void clamp_home_position(void)
{
    if (SERVO_CALIBRATION_MODE != 0U) {
        g_servo_x_home_us = clamp_servo(
            g_servo_x_home_us, SERVO_X_CALIB_MIN_US, SERVO_X_CALIB_MAX_US);
        g_servo_y_home_us = clamp_servo(
            g_servo_y_home_us, SERVO_Y_CALIB_MIN_US, SERVO_Y_CALIB_MAX_US);
    } else {
        g_servo_x_home_us = clamp_servo(
            g_servo_x_home_us, SERVO_X_MIN_US, SERVO_X_MAX_US);
        g_servo_y_home_us = clamp_servo(
            g_servo_y_home_us, SERVO_Y_MIN_US, SERVO_Y_MAX_US);
    }
}

static NOINLINE void servo_go_home_now(void)
{
    clamp_home_position();
    g_servo_x_us = g_servo_x_home_us;
    g_servo_y_us = g_servo_y_home_us;
    g_aim_raw_dx = 0;
    g_aim_raw_dy = 0;
    g_aim_dx = 0;
    g_aim_dy = 0;
    g_aim_filtered_cx = IMG_CENTER_X;
    g_aim_filtered_cy = IMG_CENTER_Y;
    g_aim_x_step_us = 0;
    g_aim_y_step_us = 0;
    g_aim_filter_ready = 0U;
    g_aim_target_confirm_count = 0U;
    g_aim_confident_target = 0U;
    g_aim_search_active = 0U;
    g_aim_search_dir = 1;
    g_no_target_frames = 0U;
}

static NOINLINE void servo_hold_home_on_startup(void)
{
    servo_go_home_now();
    g_aim_state = 4U;
    led_set(true);

    for (uint32_t i = 0U; i < SERVO_STARTUP_HOME_FRAMES; i++) {
        servo_output_one_frame();
        buzzer_update_one_frame();
        g_servo_startup_home_frames++;
    }

    led_set(false);
    g_servo_startup_home_done = 1U;
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
        g_diag_stage = DIAG_STAGE_NO_LINK;
        g_diag_no_link_count++;
        g_aim_search_active = 0U;
        g_aim_target_confirm_count = 0U;
        g_aim_confident_target = 0U;
        g_aim_x_step_us = 0;
        g_aim_y_step_us = 0;
        g_aim_filter_ready = 0U;
        return;
    }

    if ((g_omv_camera_ok == 0U) || (g_omv_target_detected == 0U)) {
        g_diag_no_target_count++;
        aim_handle_unusable_target(1U, DIAG_STAGE_NO_TARGET);
        return;
    }

    g_aim_raw_dx = (int16_t) (g_omv_cx - IMG_CENTER_X);
    g_aim_raw_dy = (int16_t) (g_omv_cy - IMG_CENTER_Y);

    if ((FULL_TRACK_ACCEPT_EDGE_TARGET == 0U) && aim_target_near_edge()) {
        aim_update_search_dir_from_dx(g_aim_raw_dx);
        g_diag_edge_target_count++;
        aim_handle_unusable_target(8U, DIAG_STAGE_EDGE_TARGET);
        return;
    }

    if (g_omv_pixels < AIM_MIN_PIXELS) {
        aim_update_search_dir_from_dx(g_aim_raw_dx);
        g_diag_small_target_count++;
        g_aim_rejected_small_target++;
        aim_handle_unusable_target(6U, DIAG_STAGE_SMALL_TARGET);
        return;
    }

    g_no_target_frames = 0U;
    g_aim_search_active = 0U;

    if (g_aim_target_confirm_count < AIM_TARGET_CONFIRM_FRAMES) {
        g_aim_target_confirm_count++;
        g_aim_confident_target = 0U;
        g_aim_state = 7U;
        g_diag_stage = DIAG_STAGE_CONFIRM_WAIT;
        g_diag_confirm_wait_count++;
        g_aim_x_step_us = 0;
        g_aim_y_step_us = 0;
        return;
    }

    g_aim_confident_target = 1U;
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
    aim_update_search_dir_from_dx(dx);

    if ((g_aim_x_step_us == 0) && (g_aim_y_step_us == 0)) {
        g_aim_state = 3U;
        g_diag_stage = DIAG_STAGE_DEADBAND;
        g_diag_deadband_count++;
    } else {
        g_aim_state = 2U;
    }

    next_x = (int32_t) g_servo_x_us + g_aim_x_step_us;
    next_y = (int32_t) g_servo_y_us + g_aim_y_step_us;
    g_servo_x_us = clamp_servo(next_x, SERVO_X_MIN_US, SERVO_X_MAX_US);
    g_servo_y_us = clamp_servo(next_y, SERVO_Y_MIN_US, SERVO_Y_MAX_US);
    g_diag_x_limited = ((int32_t) g_servo_x_us != next_x) ? 1U : 0U;
    g_diag_y_limited = ((int32_t) g_servo_y_us != next_y) ? 1U : 0U;
    if ((g_diag_x_limited != 0U) || (g_diag_y_limited != 0U)) {
        g_diag_stage = DIAG_STAGE_LIMITED;
        g_diag_limited_count++;
    } else if ((g_aim_x_step_us != 0) || (g_aim_y_step_us != 0)) {
        g_diag_stage = DIAG_STAGE_SERVO_UPDATE;
        g_diag_servo_update_count++;
    }
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

static uint32_t course_point_relative_sample(uint8_t point_index)
{
    switch (point_index) {
    case 0U:
        return COURSE_POINT_B_SAMPLE;
    case 1U:
        return COURSE_POINT_C_SAMPLE;
    case 2U:
        return COURSE_POINT_D_SAMPLE;
    default:
        return COURSE_POINT_A_SAMPLE;
    }
}

static uint8_t course_point_id_from_index(uint8_t point_index)
{
    switch (point_index) {
    case 0U:
        return 2U;
    case 1U:
        return 3U;
    case 2U:
        return 4U;
    default:
        return 1U;
    }
}

static uint16_t course_point_forced_x_us(uint8_t point_index)
{
    if (point_index >= 3U) {
        return COURSE_HOME_TARGET_US;
    }

    return course_preaim_target_from_lap_sample(
        course_point_relative_sample(point_index));
}

static NOINLINE void aim_reset_after_forced_point(void)
{
    g_aim_raw_dx = 0;
    g_aim_raw_dy = 0;
    g_aim_dx = 0;
    g_aim_dy = 0;
    g_aim_filtered_cx = IMG_CENTER_X;
    g_aim_filtered_cy = IMG_CENTER_Y;
    g_aim_x_step_us = 0;
    g_aim_y_step_us = 0;
    g_aim_filter_ready = 0U;
    g_aim_target_confirm_count = 0U;
    g_aim_confident_target = 0U;
    g_aim_search_active = 0U;
    g_no_target_frames = 0U;
}

static NOINLINE void mission_enter_point_hold(void)
{
    uint8_t point_index = g_point_stop_next_index;
    uint32_t relative_sample = course_point_relative_sample(point_index);

    motors_stop();
    led_set(true);
    g_turn_mode = 0U;
    g_line_lost_count = 0U;
    g_line_correction = 0;
    g_phase = PHASE_POINT_HOLD;
    g_stop_reason = 5U;
    g_mission_phase = 4U;
    g_point_hold_index = point_index;
    g_point_hold_id = course_point_id_from_index(point_index);
    g_mission_last_point = g_point_hold_id;
    g_point_hold_frames_left = COURSE_POINT_HOLD_FRAMES;
    g_point_stop_target_sample = g_point_stop_lap_base + relative_sample;
    g_point_forced_x_us = course_point_forced_x_us(point_index);
    g_point_hold_target_locked = 0U;
    g_servo_x_us = g_point_forced_x_us;
    g_servo_y_us = g_servo_y_home_us;
    g_course_preaim_x_us = g_point_forced_x_us;
    g_course_aim_offset_us =
        (int16_t) ((int32_t) g_point_forced_x_us - COURSE_HOME_TARGET_US);
    aim_reset_after_forced_point();

    if (g_point_hold_id == 2U) {
        buzzer_start_beeps(2U);
    } else if (g_point_hold_id == 3U) {
        buzzer_start_beeps(4U);
    } else if (g_point_hold_id == 4U) {
        buzzer_start_beeps(5U);
    } else {
        buzzer_start_beeps(1U);
    }
}

static bool mission_try_enter_point_hold(void)
{
    uint32_t target_sample;

    if (MISSION_ENABLE_POINT_STOPS == 0U) {
        return false;
    }

    target_sample = g_point_stop_lap_base +
        course_point_relative_sample(g_point_stop_next_index);

    if (g_run_sample < target_sample) {
        return false;
    }

    mission_enter_point_hold();
    return true;
}

static NOINLINE void mission_resume_after_point_hold(void)
{
    led_set(false);
    g_stop_reason = 0U;

    if (g_point_hold_target_locked == 0U) {
        buzzer_start_long();
    }

    if (g_point_hold_index >= 3U) {
        g_point_stop_next_index = 0U;
        g_point_stop_lap_base += COURSE_LAP_SAMPLES;
    } else {
        g_point_stop_next_index = (uint8_t) (g_point_hold_index + 1U);
    }

    read_line_sensors();
    update_line_position();
    line_follow_update();
    motors_forward();
    g_phase = PHASE_LINE_RUN;
}

static NOINLINE void mission_update_point_hold(void)
{
    motors_stop();

    if ((g_point_hold_target_locked == 0U) &&
        (g_omv_target_detected != 0U) &&
        (g_aim_state == 3U)) {
        g_point_hold_target_locked = 1U;
        buzzer_start_beeps(3U);
    }

    if (g_point_hold_frames_left > 0U) {
        g_point_hold_frames_left--;
        return;
    }

    mission_resume_after_point_hold();
}

static NOINLINE void mission_start_b_aim(void)
{
    motors_stop();
    led_set(true);
    g_turn_mode = 0U;
    g_line_lost_count = 0U;
    g_line_correction = 0;
    g_phase = PHASE_MISSION_AIM;
    g_stop_reason = 3U;
    g_mission_phase = 1U;
    g_mission_last_point = 2U;
    g_mission_aim_frames = 0U;
    g_mission_aim_stable_frames = 0U;
    g_mission_aim_success = 0U;
    g_mission_b_aim_preset_applied = 1U;

    g_servo_x_us = clamp_servo(
        MISSION_B_AIM_X_PRESET_US, SERVO_X_MIN_US, SERVO_X_MAX_US);
    g_servo_y_us = clamp_servo(
        MISSION_B_AIM_Y_PRESET_US, SERVO_Y_MIN_US, SERVO_Y_MAX_US);
    g_aim_raw_dx = 0;
    g_aim_raw_dy = 0;
    g_aim_dx = 0;
    g_aim_dy = 0;
    g_aim_filtered_cx = IMG_CENTER_X;
    g_aim_filtered_cy = IMG_CENTER_Y;
    g_aim_x_step_us = 0;
    g_aim_y_step_us = 0;
    g_aim_filter_ready = 0U;
    g_aim_target_confirm_count = 0U;
    g_aim_confident_target = 0U;
    g_aim_search_active = 0U;
    g_aim_search_dir = MISSION_B_AIM_SEARCH_DIR;
    g_no_target_frames = 0U;
}

static NOINLINE void mission_start_point_prompt(uint8_t point_id)
{
    g_mission_last_point = point_id;
    g_mission_prompt_frames = MISSION_POINT_PROMPT_FRAMES;
    led_set(true);
}

static NOINLINE void mission_update_point_prompt(void)
{
    if (g_mission_prompt_frames == 0U) {
        return;
    }

    if ((g_mission_prompt_frames % MISSION_POINT_TOGGLE_FRAMES) == 0U) {
        led_toggle();
    }

    g_mission_prompt_frames--;
    if (g_mission_prompt_frames == 0U) {
        led_set(false);
    }
}

static bool mission_b_aim_finished(void)
{
    g_mission_aim_frames++;

    if (g_aim_state == 3U) {
        g_mission_aim_stable_frames++;
    } else {
        g_mission_aim_stable_frames = 0U;
    }

    if (g_mission_aim_stable_frames >= MISSION_AIM_STABLE_FRAMES) {
        g_mission_aim_success = 1U;
        return true;
    }

    if (g_mission_aim_frames >= MISSION_AIM_MAX_FRAMES) {
        g_mission_aim_success = 0U;
        return true;
    }

    return false;
}

static NOINLINE void mission_resume_after_b_aim(void)
{
    led_set(false);
    g_mission_b_done = 1U;
    g_mission_phase = 2U;
    g_stop_reason = 0U;

    read_line_sensors();
    update_line_position();
    line_follow_update();
    motors_forward();
    g_phase = PHASE_LINE_RUN;
}

static bool mission_handle_key_points(void)
{
    if ((g_mission_b_done != 0U) &&
        (g_mission_c_done == 0U) &&
        (g_run_sample >= MISSION_C_PROMPT_SAMPLE)) {
        g_mission_c_done = 1U;
        mission_start_point_prompt(3U);
    }

    if ((g_mission_b_done != 0U) &&
        (g_mission_d_done == 0U) &&
        (g_run_sample >= MISSION_D_PROMPT_SAMPLE)) {
        g_mission_d_done = 1U;
        mission_start_point_prompt(4U);
    }

    if ((g_mission_b_done != 0U) &&
        (g_mission_a_done == 0U) &&
        (g_run_sample >= MISSION_A_FINISH_SAMPLE)) {
        g_mission_a_done = 1U;
        g_mission_phase = 3U;
        g_stop_reason = 4U;
        g_phase = 3U;
        motors_stop();
        led_set(true);
        g_mission_last_point = 1U;
        return true;
    }

    return false;
}

int main(void)
{
    uint32_t last_color_count;
    uint32_t last_frame_count;

    SYSCFG_DL_init();

    safe_motor_outputs_init();
    servo_outputs_init();
    init_output(IOMUX_PINCM37, LED_PORT, LED_PIN, false);
    buzzer_output_init();
    line_inputs_init();
    pwm_init();
    motors_stop();
    uart0_openmv_init();

    startup_blink();
    buzzer_start_beeps(1U);
    servo_hold_home_on_startup();

    read_line_sensors();
    update_line_position();
    line_follow_update();
    motors_forward();
    g_phase = PHASE_LINE_RUN;

    last_color_count = g_omv_color_frame_count;
    last_frame_count = g_omv_frame_count;

    while (1) {
        if (SERVO_CALIBRATION_MODE != 0U) {
            motors_stop();
            g_phase = 5U;
            g_servo_calibrate_enable = 1U;
            servo_go_home_now();
            g_aim_state = 5U;
            servo_output_one_frame();
            continue;
        }

        if ((OPENMV_ENABLE_REQUESTS != 0U) &&
            ((g_servo_frame_count % OPENMV_REQUEST_INTERVAL_FRAMES) == 0U)) {
            omv_send_color_request();
        }

        if (g_servo_calibrate_enable != 0U) {
            g_servo_calibrate_enable = 0U;
        }

        servo_output_one_frame();
        buzzer_update_one_frame();
        if (COURSE_PREAIM_ENABLE != 0U) {
            if (g_phase == PHASE_POINT_HOLD) {
                g_course_preaim_x_us = g_point_forced_x_us;
                g_course_aim_offset_us = (int16_t)
                    ((int32_t) g_point_forced_x_us - COURSE_HOME_TARGET_US);
            } else {
                (void) course_preaim_target_x_us();
            }
        }

        if (g_omv_color_frame_count != last_color_count) {
            last_color_count = g_omv_color_frame_count;
            aiming_update_from_latest_color_frame();
        }

        if (g_omv_frame_count != last_frame_count) {
            last_frame_count = g_omv_frame_count;
            g_omv_timeout_frames = 0U;
        } else if (g_omv_timeout_frames < 0xFFFFFFFFU) {
            g_omv_timeout_frames++;
            if (g_omv_timeout_frames > LINK_TIMEOUT_FRAMES) {
                g_omv_link_ok = 0U;
                g_aim_state = 0U;
                g_diag_stage = DIAG_STAGE_NO_LINK;
                g_diag_no_link_count++;
                led_set(false);
            }
        }

        if (g_phase == PHASE_MISSION_AIM) {
            if (mission_b_aim_finished()) {
                mission_resume_after_b_aim();
            }
            continue;
        }

        if (g_phase == PHASE_POINT_HOLD) {
            mission_update_point_hold();
            continue;
        }

        if (g_phase == PHASE_LINE_RUN) {
            if (mission_try_enter_point_hold()) {
                continue;
            }

            if ((MISSION_ENABLE_B_AIM != 0U) &&
                (g_mission_b_done == 0U) &&
                (g_run_sample >= MISSION_B_STOP_SAMPLE)) {
                mission_start_b_aim();
                continue;
            }

            if (MISSION_ENABLE_POINT_PROMPTS != 0U) {
                mission_update_point_prompt();
                if (mission_handle_key_points()) {
                    continue;
                }
            }

            read_line_sensors();
            update_line_position();

            if ((g_line_lost_count >= LINE_LOST_STOP_SAMPLES) &&
                (g_line_last_error != 0)) {
                g_phase = 4U;
                g_stop_reason = 2U;
                motors_stop();
            } else {
                line_follow_update();
                g_run_sample++;
                if (g_run_sample >= LINE_RUN_SAMPLES) {
                    g_phase = 3U;
                    g_stop_reason = 1U;
                    motors_stop();
                }
            }
        }
    }
}
