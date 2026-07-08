/*
 * MSPM0G3507 + OpenMV + dual-servo static aiming test.
 *
 * Goal:
 *   Keep the car still, initialize the PA14/PA17 gimbal to a calibrated
 *   home position, then read red target coordinates from OpenMV and move
 *   the gimbal so the target returns to image center.
 *
 * OpenMV script:
 *   C:/Users/1/Desktop/ELECTRIC/../电赛/ccs_project/openmv_test/
 *   20260704_135927_openmv_uart_stream_fast_rot180.py
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
 *   Motors are disabled. PB4/STBY is held LOW.
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

#define IMG_CENTER_X                160
#define IMG_CENTER_Y                120

#define AIM_X_PULSE_SIGN            (-1)
#define AIM_Y_PULSE_SIGN            (-1)

#define AIM_DEAD_X_PIXELS           14
#define AIM_DEAD_Y_PIXELS           12
#define AIM_X_DIVISOR               2
#define AIM_Y_DIVISOR               2
#define AIM_MAX_STEP_US             70
#define AIM_MIN_PIXELS              25U
#define AIM_FILTER_DIVISOR          2
#define AIM_TARGET_CONFIRM_FRAMES   1U
#define AIM_LOST_PREDICT_FRAMES     8U
#define AIM_LOST_PREDICT_DIVISOR    2

#define SERVO_PERIOD_US             20000U
#define SERVO_X_HOME_US             1500U
#define SERVO_Y_HOME_US             1500U
#define SERVO_STARTUP_HOME_FRAMES   75U
#define SERVO_X_MIN_US              1050U
#define SERVO_X_MAX_US              1950U
#define SERVO_Y_MIN_US              500U
#define SERVO_Y_MAX_US              2100U
#define SERVO_X_CALIB_MIN_US        500U
#define SERVO_X_CALIB_MAX_US        2500U
#define SERVO_Y_CALIB_MIN_US        500U
#define SERVO_Y_CALIB_MAX_US        2500U
#define SERVO_SLEW_STEP_US          32U
#define SERVO_X_LIMIT_MARGIN_US     35U
#define SERVO_X_LIMIT_BACKOFF_US    70U

#define SERVO_CALIBRATION_MODE         0U
#define OPENMV_ENABLE_REQUESTS         0U
#define OPENMV_REQUEST_INTERVAL_FRAMES 4U
#define LINK_TIMEOUT_FRAMES            120U
#define LOST_RECENTER_FRAMES           40U

/* Diagnostic build switches. Enable only one diagnostic mode at a time. */
#define DIAG_FREEZE_YAW_SERVO          0U
#define DIAG_YAW_SWEEP_MODE            0U
#define DIAG_YAW_SWEEP_MIN_US          1300U
#define DIAG_YAW_SWEEP_MAX_US          1700U
#define DIAG_YAW_SWEEP_STEP_US         3U
#define DIAG_SINGLE_PIN_SWEEP_MODE     1U
#define DIAG_SINGLE_PIN_IS_PA14        0U
#define DIAG_SINGLE_SWEEP_MIN_US       1100U
#define DIAG_SINGLE_SWEEP_MAX_US       1900U
#define DIAG_SINGLE_SWEEP_STEP_US      8U
#define DIAG_STOP_ALL_SERVOS_MODE      0U

#define LED_PORT                    GPIOA
#define LED_PIN                     DL_GPIO_PIN_15

#define SERVO_PORT                  GPIOA
#define SERVO_X_PIN                 DL_GPIO_PIN_14
#define SERVO_Y_PIN                 DL_GPIO_PIN_17

#define MOTOR_PORT                  GPIOB
#define AIN1_PIN                    DL_GPIO_PIN_0
#define AIN2_PIN                    DL_GPIO_PIN_1
#define BIN1_PIN                    DL_GPIO_PIN_2
#define BIN2_PIN                    DL_GPIO_PIN_3
#define STBY_PIN                    DL_GPIO_PIN_4

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
volatile uint16_t g_servo_x_goal_us = SERVO_X_HOME_US;
volatile uint16_t g_servo_y_goal_us = SERVO_Y_HOME_US;
volatile uint32_t g_servo_frame_count = 0;
volatile uint32_t g_servo_startup_home_frames = 0;
volatile uint32_t g_aim_update_count = 0;
volatile uint32_t g_no_target_frames = 0;
volatile uint32_t g_aim_rejected_small_target = 0;
volatile uint32_t g_servo_x_limit_guard_count = 0;
volatile uint32_t g_diag_yaw_sweep_turn_count = 0;
volatile int16_t g_diag_yaw_sweep_direction = 1;
volatile uint32_t g_diag_single_sweep_turn_count = 0;
volatile uint16_t g_diag_single_pulse_us = SERVO_X_HOME_US;
volatile int16_t g_diag_single_sweep_direction = 1;
volatile int16_t g_aim_raw_dx = 0;
volatile int16_t g_aim_raw_dy = 0;
volatile int16_t g_aim_dx = 0;
volatile int16_t g_aim_dy = 0;
volatile int16_t g_aim_filtered_cx = IMG_CENTER_X;
volatile int16_t g_aim_filtered_cy = IMG_CENTER_Y;
volatile int16_t g_aim_x_step_us = 0;
volatile int16_t g_aim_y_step_us = 0;
volatile int16_t g_aim_last_x_step_us = 0;
volatile int16_t g_aim_last_y_step_us = 0;
volatile uint8_t g_aim_state = 0;
volatile uint8_t g_tracking_stop_reason = 0;
volatile uint8_t g_aim_filter_ready = 0;
volatile uint8_t g_aim_target_confirm_count = 0;
volatile uint8_t g_aim_confident_target = 0;
volatile uint8_t g_servo_startup_home_done = 0;
volatile uint8_t g_servo_calibrate_enable = 0U;
volatile uint8_t g_led_state = 0;

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

static NOINLINE void safe_motor_outputs_init(void)
{
    init_output(IOMUX_PINCM12, MOTOR_PORT, AIN1_PIN, false);
    init_output(IOMUX_PINCM13, MOTOR_PORT, AIN2_PIN, false);
    init_output(IOMUX_PINCM15, MOTOR_PORT, BIN1_PIN, false);
    init_output(IOMUX_PINCM16, MOTOR_PORT, BIN2_PIN, false);
    init_output(IOMUX_PINCM17, MOTOR_PORT, STBY_PIN, false);
}

static NOINLINE void servo_outputs_init(void)
{
    init_output(IOMUX_PINCM36, SERVO_PORT, SERVO_X_PIN, false);
    init_output(IOMUX_PINCM39, SERVO_PORT, SERVO_Y_PIN, false);
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

static NOINLINE void startup_blink(void)
{
    for (uint8_t i = 0; i < 4U; i++) {
        led_set(true);
        delay_ms(70U);
        led_set(false);
        delay_ms(70U);
    }
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

static uint16_t slew_toward_u16(uint16_t current, uint16_t target, uint16_t max_step)
{
    if (current + max_step < target) {
        return (uint16_t) (current + max_step);
    }
    if (current > target + max_step) {
        return (uint16_t) (current - max_step);
    }
    return target;
}

static NOINLINE void servo_slew_toward_goal(void)
{
    g_servo_x_us = slew_toward_u16(g_servo_x_us, g_servo_x_goal_us, SERVO_SLEW_STEP_US);
    g_servo_y_us = slew_toward_u16(g_servo_y_us, g_servo_y_goal_us, SERVO_SLEW_STEP_US);
}

static NOINLINE void servo_goal_add_step(int16_t x_step, int16_t y_step)
{
    int32_t next_x = (int32_t) g_servo_x_goal_us + x_step;
    int32_t next_y = (int32_t) g_servo_y_goal_us + y_step;

    if (DIAG_FREEZE_YAW_SERVO != 0U) {
        next_x = (int32_t) g_servo_x_home_us;
        x_step = 0;
    } else if ((g_servo_x_goal_us <= (SERVO_X_MIN_US + SERVO_X_LIMIT_MARGIN_US)) &&
        (x_step < 0)) {
        next_x = (int32_t) SERVO_X_MIN_US + SERVO_X_LIMIT_BACKOFF_US;
        g_tracking_stop_reason = 4U;
        g_servo_x_limit_guard_count++;
    } else if ((g_servo_x_goal_us >=
        (SERVO_X_MAX_US - SERVO_X_LIMIT_MARGIN_US)) && (x_step > 0)) {
        next_x = (int32_t) SERVO_X_MAX_US - SERVO_X_LIMIT_BACKOFF_US;
        g_tracking_stop_reason = 4U;
        g_servo_x_limit_guard_count++;
    }

    g_servo_x_goal_us = clamp_servo(next_x, SERVO_X_MIN_US, SERVO_X_MAX_US);
    g_servo_y_goal_us = clamp_servo(next_y, SERVO_Y_MIN_US, SERVO_Y_MAX_US);
}

static NOINLINE void diag_yaw_sweep_update_goal(void)
{
    int32_t next_x = (int32_t) g_servo_x_goal_us +
        ((int32_t) g_diag_yaw_sweep_direction * DIAG_YAW_SWEEP_STEP_US);

    g_servo_y_goal_us = g_servo_y_home_us;

    if (next_x >= (int32_t) DIAG_YAW_SWEEP_MAX_US) {
        next_x = DIAG_YAW_SWEEP_MAX_US;
        g_diag_yaw_sweep_direction = -1;
        g_diag_yaw_sweep_turn_count++;
    } else if (next_x <= (int32_t) DIAG_YAW_SWEEP_MIN_US) {
        next_x = DIAG_YAW_SWEEP_MIN_US;
        g_diag_yaw_sweep_direction = 1;
        g_diag_yaw_sweep_turn_count++;
    }

    g_servo_x_goal_us = (uint16_t) next_x;
    g_aim_state = 9U;
    g_tracking_stop_reason = 5U;
}

static NOINLINE void diag_single_pin_sweep_update(void)
{
    int32_t next_us = (int32_t) g_diag_single_pulse_us +
        ((int32_t) g_diag_single_sweep_direction * DIAG_SINGLE_SWEEP_STEP_US);

    if (next_us >= (int32_t) DIAG_SINGLE_SWEEP_MAX_US) {
        next_us = DIAG_SINGLE_SWEEP_MAX_US;
        g_diag_single_sweep_direction = -1;
        g_diag_single_sweep_turn_count++;
    } else if (next_us <= (int32_t) DIAG_SINGLE_SWEEP_MIN_US) {
        next_us = DIAG_SINGLE_SWEEP_MIN_US;
        g_diag_single_sweep_direction = 1;
        g_diag_single_sweep_turn_count++;
    }

    g_diag_single_pulse_us = (uint16_t) next_us;
    g_aim_state = 10U;
    g_tracking_stop_reason = 6U;
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
    g_servo_x_goal_us = g_servo_x_home_us;
    g_servo_y_goal_us = g_servo_y_home_us;
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
    g_aim_last_x_step_us = 0;
    g_aim_last_y_step_us = 0;
    g_aim_filter_ready = 0U;
    g_aim_target_confirm_count = 0U;
    g_aim_confident_target = 0U;
    g_no_target_frames = 0U;
    g_tracking_stop_reason = 0U;
}

static NOINLINE void servo_hold_home_on_startup(void)
{
    servo_go_home_now();
    g_aim_state = 4U;
    led_set(true);

    for (uint32_t i = 0U; i < SERVO_STARTUP_HOME_FRAMES; i++) {
        servo_slew_toward_goal();
        servo_output_one_frame();
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

    if (g_omv_link_ok == 0U) {
        g_aim_state = 0U;
        g_tracking_stop_reason = 1U;
        g_aim_x_step_us = 0;
        g_aim_y_step_us = 0;
        g_aim_filter_ready = 0U;
        return;
    }

    if ((g_omv_camera_ok == 0U) || (g_omv_target_detected == 0U)) {
        g_aim_state = 1U;
        g_tracking_stop_reason = 2U;
        g_no_target_frames++;
        g_aim_target_confirm_count = 0U;
        g_aim_confident_target = 0U;
        if ((g_no_target_frames <= AIM_LOST_PREDICT_FRAMES) &&
            ((g_aim_last_x_step_us != 0) || (g_aim_last_y_step_us != 0))) {
            g_aim_x_step_us = (int16_t) (g_aim_last_x_step_us / AIM_LOST_PREDICT_DIVISOR);
            g_aim_y_step_us = (int16_t) (g_aim_last_y_step_us / AIM_LOST_PREDICT_DIVISOR);
            servo_goal_add_step(g_aim_x_step_us, g_aim_y_step_us);
            g_aim_state = 8U;
        } else {
            g_aim_x_step_us = 0;
            g_aim_y_step_us = 0;
        }
        if (g_no_target_frames > LOST_RECENTER_FRAMES) {
            g_aim_filter_ready = 0U;
            g_aim_last_x_step_us = 0;
            g_aim_last_y_step_us = 0;
        }
        return;
    }

    if (g_omv_pixels < AIM_MIN_PIXELS) {
        g_aim_state = 6U;
        g_tracking_stop_reason = 3U;
        g_aim_rejected_small_target++;
        g_aim_target_confirm_count = 0U;
        g_aim_confident_target = 0U;
        g_aim_x_step_us = 0;
        g_aim_y_step_us = 0;
        return;
    }

    g_no_target_frames = 0U;
    g_tracking_stop_reason = 0U;
    g_aim_raw_dx = (int16_t) (g_omv_cx - IMG_CENTER_X);
    g_aim_raw_dy = (int16_t) (g_omv_cy - IMG_CENTER_Y);

    if (g_aim_target_confirm_count < AIM_TARGET_CONFIRM_FRAMES) {
        g_aim_target_confirm_count++;
        if (g_aim_target_confirm_count < AIM_TARGET_CONFIRM_FRAMES) {
            g_aim_confident_target = 0U;
            g_aim_state = 7U;
            g_aim_x_step_us = 0;
            g_aim_y_step_us = 0;
            return;
        }
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
    if (DIAG_FREEZE_YAW_SERVO != 0U) {
        g_aim_x_step_us = 0;
    }

    if ((g_aim_x_step_us == 0) && (g_aim_y_step_us == 0)) {
        g_aim_state = 3U;
        g_aim_last_x_step_us = 0;
        g_aim_last_y_step_us = 0;
    } else {
        g_aim_state = 2U;
        g_aim_last_x_step_us = g_aim_x_step_us;
        g_aim_last_y_step_us = g_aim_y_step_us;
    }

    servo_goal_add_step(g_aim_x_step_us, g_aim_y_step_us);
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

static NOINLINE void servo_output_single_pin_frame(void)
{
    uint32_t active_pin = (DIAG_SINGLE_PIN_IS_PA14 != 0U) ?
        SERVO_X_PIN : SERVO_Y_PIN;
    uint16_t pulse_us = g_diag_single_pulse_us;

    DL_GPIO_clearPins(SERVO_PORT, SERVO_X_PIN | SERVO_Y_PIN);
    DL_GPIO_setPins(SERVO_PORT, active_pin);
    delay_us(pulse_us);
    DL_GPIO_clearPins(SERVO_PORT, active_pin);
    delay_us(SERVO_PERIOD_US - pulse_us);

    g_servo_frame_count++;
}

int main(void)
{
    uint32_t last_color_count;
    uint32_t last_frame_count;

    SYSCFG_DL_init();

    safe_motor_outputs_init();
    servo_outputs_init();
    init_output(IOMUX_PINCM37, LED_PORT, LED_PIN, false);
    if (DIAG_SINGLE_PIN_SWEEP_MODE == 0U) {
        uart0_openmv_init();
    }

    startup_blink();
    servo_hold_home_on_startup();

    last_color_count = g_omv_color_frame_count;
    last_frame_count = g_omv_frame_count;

    while (1) {
        if (DIAG_STOP_ALL_SERVOS_MODE != 0U) {
            DL_GPIO_clearPins(SERVO_PORT, SERVO_X_PIN | SERVO_Y_PIN);
            led_set(false);
            delay_ms(100U);
            continue;
        }

        if (DIAG_SINGLE_PIN_SWEEP_MODE != 0U) {
            diag_single_pin_sweep_update();
            servo_output_single_pin_frame();
            continue;
        }

        if (SERVO_CALIBRATION_MODE != 0U) {
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

        if (DIAG_YAW_SWEEP_MODE != 0U) {
            diag_yaw_sweep_update_goal();
            servo_slew_toward_goal();
            servo_output_one_frame();

            if (g_omv_frame_count != last_frame_count) {
                last_frame_count = g_omv_frame_count;
                g_omv_timeout_frames = 0U;
            } else if (g_omv_timeout_frames < 0xFFFFFFFFU) {
                g_omv_timeout_frames++;
                if (g_omv_timeout_frames > LINK_TIMEOUT_FRAMES) {
                    g_omv_link_ok = 0U;
                    led_set(false);
                }
            }
            continue;
        }

        servo_slew_toward_goal();
        servo_output_one_frame();

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
                g_tracking_stop_reason = 1U;
                led_set(false);
            }
        }
    }
}
