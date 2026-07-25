/*
 * V77 MSPM0G3507 + reduced line-loss left nudge.
 *
 * Wiring:
 *   PB5 -> AD0, PB6 -> AD1, PB7 -> AD2, PA7 <- OUT.
 *
 * Scope:
 *   The grayscale array, TB6612 motors and MPU gyro are active.
 *   OpenMV, servos, encoder control and mission logic remain disabled.
 *   The low-trigger buzzer module is held HIGH (silent).
 *   OpenMV UART is not initialized.
 *
 * Startup:
 *   Place X1..X8 on the same white background before reset. The first
 *   50 scans establish a white reference without assuming OUT polarity.
 */

#include <stdbool.h>
#include <stdint.h>
#include "ti_msp_dl_config.h"

#define NOINLINE                         __attribute__((noinline))

#define FIRMWARE_VERSION                 81U
#define MPU_ENABLE                       0U
#define GRAY_CHANNEL_COUNT               8U
#define GRAY_SAMPLE_COUNT                9U
#define GRAY_HIGH_MAJORITY               5U
#define GRAY_ADDRESS_SETTLE_US           40U
#define GRAY_SAMPLE_INTERVAL_US          10U
#define GRAY_BASELINE_SCAN_COUNT         50U
#define GRAY_MAIN_LOOP_MS                20U
#define GRAY_POWERUP_DELAY_MS            300U
#define DELAY_LOOPS_PER_US               8U

#define PHASE_INIT                       0U
#define PHASE_CALIBRATION                1U
#define PHASE_LINE_RUN                   2U
#define PHASE_COMPLETE                   3U
#define PHASE_STOP                       4U
#define PHASE_START_BRIDGE               8U

#define STOP_REASON_NONE                 0U
#define STOP_REASON_TEST_COMPLETE        1U
#define STOP_REASON_BASELINE_INVALID     2U
#define STOP_REASON_BRIDGE_TIMEOUT       3U
#define STOP_REASON_LINE_LOST_TIMEOUT    4U
#define STOP_REASON_CURVE_TIMEOUT        5U
#define STOP_REASON_GAP_TIMEOUT          6U

#define PWM_TIMER                        TIMG0
#define PWM_PERIOD                       1600U
#define PWM_LEFT_BASE_TICKS              425U
#define PWM_RIGHT_BASE_TICKS             435U
#define PWM_MIN_TICKS                    250U
#define PWM_MAX_TICKS                    657U
#define START_BOOST_LEFT_TICKS           561U
#define START_BOOST_RIGHT_TICKS          573U
#define START_BOOST_FRAMES               20U
#define START_BRIDGE_LEFT_TICKS          423U
#define START_BRIDGE_RIGHT_TICKS         431U
#define START_BRIDGE_GYRO_FILTER_DIVISOR 4
#define START_BRIDGE_GYRO_RATE_DEADBAND_MDPS 600
#define START_BRIDGE_GYRO_RATE_DIVISOR   1500
#define START_BRIDGE_GYRO_RATE_LIMIT     30
#define START_BRIDGE_HEADING_DEADBAND_MDEG 500
#define START_BRIDGE_HEADING_DIVISOR     400
#define START_BRIDGE_HEADING_LIMIT       35
#define START_BRIDGE_LEFT_CORR_LIMIT     72
#define START_BRIDGE_RIGHT_CORR_LIMIT    18
#define START_BRIDGE_RIGHT_CONFIRM_FRAMES 5U
#define START_BRIDGE_LEFT_SLEW_TICKS     7
#define START_BRIDGE_RIGHT_SLEW_TICKS    2
#define START_BRIDGE_RELEASE_SLEW_TICKS  6
#define LINE_KP_DIVISOR                  36
#define LINE_CORR_LIMIT                  70
#define LINE_SHARP_ERROR                 1800
#define LINE_SHARP_KP_DIVISOR            8
#define LINE_SHARP_CORR_LIMIT            240
#define LINE_CENTER_MASK                 0x18U
#define LINE_CENTER_ERROR_MAX            600
#define LINE_CENTER_CONFIRM_FRAMES       2U
#define LINE_LOST_STRONG_FRAMES          10U
#define LINE_LOST_MEDIUM_FRAMES          25U
#define LINE_LOST_MEDIUM_CORR_LIMIT      100
#define LINE_LOST_LONG_CORR_LIMIT        45
#define LINE_MAX_ACTIVE                  6U
#define LINE_ENTRY_CONFIRM_FRAMES        2U
#define START_BRIDGE_MAX_FRAMES          250U
#define LINE_LOST_MAX_FRAMES             100U
#define RUN_MAX_FRAMES                   1500U
#define START_READY_HOLD_MS              800U

#define MPU_I2C                          I2C0
#define MPU_I2C_ADDRESS                  0x68U
#define MPU_I2C_TIMEOUT                  200000U
#define MPU_I2C_TIMER_PERIOD_100KHZ      31U
#define MPU6050_WHO_AM_I                 0x68U
#define MPU6500_WHO_AM_I                 0x70U
#define MPU_REG_SMPLRT_DIV               0x19U
#define MPU_REG_CONFIG                   0x1AU
#define MPU_REG_GYRO_CONFIG              0x1BU
#define MPU_REG_GYRO_XOUT_H              0x43U
#define MPU_REG_PWR_MGMT_1               0x6BU
#define MPU_REG_WHO_AM_I                 0x75U
#define MPU_GYRO_FS_500DPS               0x08U
#define MPU_CALIBRATION_SAMPLES          500U

#define TURN_EXIT_HOLD_FRAMES            28U
#define GYRO_LOST_ENABLE_FRAMES          5U
#define GYRO_RATE_DIVISOR                1500
#define GYRO_RATE_CORR_LIMIT             50
#define GYRO_HEADING_DIVISOR             1000
#define GYRO_HEADING_CORR_LIMIT          30
#define GYRO_TOTAL_CORR_LIMIT            65

#define TURN_EXIT_RIGHT_HISTORY_MASK     0x70U
#define TURN_EXIT_LEFT_HISTORY_MASK      0x0EU
#define TURN_EXIT_CONFIRM_LOST_FRAMES    5U
#define TURN_EXIT_BRAKE_CORR_TICKS       0
#define TURN_EXIT_BRAKE_MIN_FRAMES       3U
#define TURN_EXIT_BRAKE_MAX_FRAMES       15U
#define TURN_EXIT_BRAKE_NO_MPU_FRAMES    8U
#define TURN_EXIT_BRAKE_STOP_RATE_MDPS   3000
#define TURN_EXIT_BRAKE_START_RATE_MDPS  1000
#define TURN_EXIT_LEFT_NUDGE_TICKS       10
#define TURN_EXIT_LEFT_NUDGE_FRAMES      4U

#define CURVE_RIGHT_FEEDFORWARD_TICKS    30
#define CURVE_FEEDFORWARD_STEP_TICKS     3
#define CURVE_TARGET_ANGLE_MDEG          180000
#define CURVE_EXIT_MIN_ANGLE_MDEG        165000
#define CURVE_REENTRY_CONFIRM_FRAMES     2U
#define CURVE_NO_LINE_MAX_FRAMES         180U
#define GAP_REENTRY_MIN_FRAMES           25U
#define GAP_REENTRY_MAX_FRAMES           300U
#define GRAY_CURVE_ENTRY_ERROR           1200
#define GRAY_CURVE_ENTRY_CONFIRM_FRAMES  2U

#define MOTOR_PORT                       GPIOB
#define AIN1_PIN                         DL_GPIO_PIN_0
#define AIN2_PIN                         DL_GPIO_PIN_1
#define BIN1_PIN                         DL_GPIO_PIN_2
#define BIN2_PIN                         DL_GPIO_PIN_3
#define STBY_PIN                         DL_GPIO_PIN_4

#define PWM_PORT                         GPIOA
#define PWMA_PIN                         DL_GPIO_PIN_12
#define PWMB_PIN                         DL_GPIO_PIN_13

#define GRAY_ADDRESS_PORT                GPIOB
#define GRAY_AD0_PIN                     DL_GPIO_PIN_5
#define GRAY_AD1_PIN                     DL_GPIO_PIN_6
#define GRAY_AD2_PIN                     DL_GPIO_PIN_7
#define GRAY_ADDRESS_MASK                (GRAY_AD0_PIN | GRAY_AD1_PIN | GRAY_AD2_PIN)

#define GRAY_OUT_PORT                    GPIOA
#define GRAY_OUT_PIN                     DL_GPIO_PIN_7

#define SERVO_PORT                       GPIOA
#define SERVO_X_PIN                      DL_GPIO_PIN_14
#define SERVO_Y_PIN                      DL_GPIO_PIN_17

#define LED_PORT                         GPIOA
#define LED_PIN                          DL_GPIO_PIN_15

#define BUZZER_PORT                      GPIOB
#define BUZZER_PIN                       DL_GPIO_PIN_11

volatile uint32_t g_firmware_version = FIRMWARE_VERSION;
volatile uint8_t g_motor_disabled = 0U;
volatile uint8_t g_buzzer_active_low = 1U;
volatile uint8_t g_buzzer_idle_level = 1U;
volatile uint8_t g_gray_test_state = 0U;
volatile uint8_t g_gray_selected_channel = 0U;
volatile uint8_t g_gray_live_out = 0U;

volatile uint8_t g_gray_raw_high_mask = 0U;
volatile uint8_t g_gray_unstable_mask = 0U;
volatile uint8_t g_gray_white_baseline_mask = 0U;
volatile uint8_t g_gray_changed_from_white_mask = 0U;
volatile uint8_t g_gray_previous_change_mask = 0U;

volatile uint8_t g_gray_x1 = 0U;
volatile uint8_t g_gray_x2 = 0U;
volatile uint8_t g_gray_x3 = 0U;
volatile uint8_t g_gray_x4 = 0U;
volatile uint8_t g_gray_x5 = 0U;
volatile uint8_t g_gray_x6 = 0U;
volatile uint8_t g_gray_x7 = 0U;
volatile uint8_t g_gray_x8 = 0U;

volatile uint8_t g_gray_x1_high_count = 0U;
volatile uint8_t g_gray_x2_high_count = 0U;
volatile uint8_t g_gray_x3_high_count = 0U;
volatile uint8_t g_gray_x4_high_count = 0U;
volatile uint8_t g_gray_x5_high_count = 0U;
volatile uint8_t g_gray_x6_high_count = 0U;
volatile uint8_t g_gray_x7_high_count = 0U;
volatile uint8_t g_gray_x8_high_count = 0U;

volatile uint8_t g_gray_all_low = 0U;
volatile uint8_t g_gray_all_high = 0U;
volatile uint8_t g_gray_mixed = 0U;
volatile uint8_t g_gray_baseline_ready = 0U;
volatile uint8_t g_gray_baseline_uniform = 0U;
volatile uint8_t g_gray_white_level_code = 2U;
volatile uint8_t g_gray_changed_count = 0U;
volatile uint8_t g_gray_line_valid = 0U;
volatile int16_t g_gray_line_error = 0;
volatile uint8_t g_gray_line_mask = 0U;
volatile uint8_t g_gray_line_abnormal = 0U;
volatile int16_t g_line_last_error = 0;
volatile int16_t g_line_last_nonzero_error = 0;
volatile int32_t g_line_correction = 0;
volatile uint8_t g_turn_mode = 0U;
volatile uint32_t g_line_lost_count = 0U;
volatile uint8_t g_line_center_confirm_count = 0U;
volatile int32_t g_line_lost_correction_limit = LINE_SHARP_CORR_LIMIT;

volatile uint32_t g_left_pwm_ticks = 0U;
volatile uint32_t g_right_pwm_ticks = 0U;
volatile uint32_t g_left_compare_ticks = PWM_PERIOD;
volatile uint32_t g_right_compare_ticks = PWM_PERIOD;
volatile uint8_t g_phase = PHASE_INIT;
volatile uint8_t g_stop_reason = STOP_REASON_NONE;
volatile uint16_t g_line_entry_confirm_count = 0U;
volatile uint32_t g_start_bridge_frame_count = 0U;
volatile uint8_t g_start_boost_active = 0U;
volatile uint8_t g_start_bridge_heading_active = 0U;
volatile int32_t g_start_bridge_heading_target_mdeg = 0;
volatile int32_t g_start_bridge_heading_error_mdeg = 0;
volatile int32_t g_start_bridge_filtered_gyro_z_mdps = 0;
volatile int32_t g_start_bridge_gyro_rate_correction = 0;
volatile int32_t g_start_bridge_heading_correction = 0;
volatile int32_t g_start_bridge_desired_correction = 0;
volatile int32_t g_start_bridge_mpu_correction = 0;
volatile uint8_t g_start_bridge_right_confirm_count = 0U;
volatile uint8_t g_start_bridge_correction_mode = 0U;
volatile uint32_t g_start_bridge_left_cmd_ticks = 0U;
volatile uint32_t g_start_bridge_right_cmd_ticks = 0U;
volatile uint32_t g_run_frame = 0U;

volatile uint16_t g_gray_baseline_scan_count = 0U;
volatile uint16_t g_gray_baseline_votes[GRAY_CHANNEL_COUNT] = {0U};
volatile uint8_t g_gray_channel_high_count[GRAY_CHANNEL_COUNT] = {0U};
volatile uint32_t g_gray_scan_count = 0U;
volatile uint32_t g_gray_change_event_count = 0U;
volatile uint8_t g_led_state = 0U;

volatile uint8_t g_mpu_state = 0U;
volatile uint8_t g_mpu_error = 0U;
volatile uint8_t g_mpu_link_ok = 0U;
volatile uint8_t g_mpu_who_am_i = 0U;
volatile uint8_t g_mpu_device_type = 0U;
volatile uint32_t g_mpu_sample_count = 0U;
volatile uint32_t g_mpu_read_fail_count = 0U;
volatile uint32_t g_i2c_error_count = 0U;
volatile int16_t g_gyro_x_raw = 0;
volatile int16_t g_gyro_y_raw = 0;
volatile int16_t g_gyro_z_raw = 0;
volatile int32_t g_gyro_z_bias_raw = 0;
volatile int32_t g_gyro_z_corrected_raw = 0;
volatile int32_t g_gyro_z_mdps = 0;
volatile int32_t g_yaw_mdeg = 0;

volatile uint8_t g_turn_was_active = 0U;
volatile uint32_t g_turn_exit_hold_count = 0U;
volatile int32_t g_turn_exit_heading_target_mdeg = 0;
volatile int32_t g_gyro_rate_correction = 0;
volatile int32_t g_gyro_heading_correction = 0;
volatile int32_t g_mpu_control_correction = 0;
volatile int32_t g_total_control_correction = 0;

volatile uint8_t g_line_mask_history_0 = 0U;
volatile uint8_t g_line_mask_history_1 = 0U;
volatile uint8_t g_line_mask_history_2 = 0U;
volatile int16_t g_line_error_history_0 = 0;
volatile int16_t g_line_error_history_1 = 0;
volatile int16_t g_line_error_history_2 = 0;
volatile uint8_t g_turn_exit_brake_active = 0U;
volatile int8_t g_turn_exit_brake_direction = 0;
volatile uint32_t g_turn_exit_brake_frame_count = 0U;
volatile uint32_t g_turn_exit_brake_trigger_count = 0U;
volatile uint8_t g_turn_exit_brake_stop_reason = 0U;
volatile int8_t g_turn_exit_candidate_direction = 0;
volatile uint32_t g_turn_exit_candidate_lost_count = 0U;
volatile uint32_t g_turn_exit_candidate_confirm_count = 0U;
volatile uint32_t g_turn_exit_candidate_cancel_count = 0U;
volatile uint8_t g_turn_exit_left_nudge_active = 0U;
volatile uint8_t g_turn_exit_left_nudge_remaining = 0U;
volatile uint32_t g_turn_exit_left_nudge_trigger_count = 0U;

volatile uint8_t g_curve_active = 0U;
volatile uint8_t g_gap_heading_active = 0U;
volatile uint8_t g_curve_reentry_confirm_count = 0U;
volatile uint8_t g_curve_entry_candidate_count = 0U;
volatile uint32_t g_curve_start_count = 0U;
volatile uint32_t g_curve_exit_count = 0U;
volatile uint32_t g_curve_forced_candidate_count = 0U;
volatile uint32_t g_gap_frame_count = 0U;
volatile uint32_t g_mpu_runtime_degrade_count = 0U;
volatile uint8_t g_course_previous_mpu_link_ok = 0U;
volatile int32_t g_curve_entry_yaw_mdeg = 0;
volatile int32_t g_curve_target_yaw_mdeg = 0;
volatile int32_t g_curve_angle_mdeg = 0;
volatile int32_t g_curve_exit_angle_mdeg = 0;
volatile int32_t g_curve_feedforward_correction = 0;

static NOINLINE void delay_cycles_rough(uint32_t cycles)
{
    volatile uint32_t i;

    for (i = 0U; i < cycles; i++) {
        __asm volatile("nop");
    }
}

static NOINLINE void delay_us_rough(uint32_t us)
{
    delay_cycles_rough(us * DELAY_LOOPS_PER_US);
}

static NOINLINE void delay_ms_rough(uint32_t ms)
{
    for (uint32_t i = 0U; i < ms; i++) {
        delay_us_rough(1000U);
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
        g_led_state = 1U;
    } else {
        DL_GPIO_clearPins(LED_PORT, LED_PIN);
        g_led_state = 0U;
    }
}

static NOINLINE void safe_outputs_init(void)
{
    init_output(IOMUX_PINCM12, MOTOR_PORT, AIN1_PIN, false);
    init_output(IOMUX_PINCM13, MOTOR_PORT, AIN2_PIN, false);
    init_output(IOMUX_PINCM15, MOTOR_PORT, BIN1_PIN, false);
    init_output(IOMUX_PINCM16, MOTOR_PORT, BIN2_PIN, false);
    init_output(IOMUX_PINCM17, MOTOR_PORT, STBY_PIN, false);

    init_output(IOMUX_PINCM34, PWM_PORT, PWMA_PIN, false);
    init_output(IOMUX_PINCM35, PWM_PORT, PWMB_PIN, false);
    init_output(IOMUX_PINCM36, SERVO_PORT, SERVO_X_PIN, false);
    init_output(IOMUX_PINCM39, SERVO_PORT, SERVO_Y_PIN, false);
    init_output(IOMUX_PINCM28, BUZZER_PORT, BUZZER_PIN, true);
    init_output(IOMUX_PINCM37, LED_PORT, LED_PIN, false);
}

#if MPU_ENABLE
static NOINLINE void i2c0_init_100khz(void)
{
    static const DL_I2C_ClockConfig clock_config = {
        .clockSel = DL_I2C_CLOCK_BUSCLK,
        .divideRatio = DL_I2C_CLOCK_DIVIDE_1,
    };

    DL_I2C_reset(MPU_I2C);
    DL_I2C_enablePower(MPU_I2C);
    delay_cycles(16U);

    DL_GPIO_initPeripheralInputFunctionFeatures(
        IOMUX_PINCM1,
        IOMUX_PINCM1_PF_I2C0_SDA,
        DL_GPIO_INVERSION_DISABLE,
        DL_GPIO_RESISTOR_NONE,
        DL_GPIO_HYSTERESIS_DISABLE,
        DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initPeripheralInputFunctionFeatures(
        IOMUX_PINCM2,
        IOMUX_PINCM2_PF_I2C0_SCL,
        DL_GPIO_INVERSION_DISABLE,
        DL_GPIO_RESISTOR_NONE,
        DL_GPIO_HYSTERESIS_DISABLE,
        DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_enableHiZ(IOMUX_PINCM1);
    DL_GPIO_enableHiZ(IOMUX_PINCM2);

    DL_I2C_setClockConfig(MPU_I2C, &clock_config);
    DL_I2C_disableAnalogGlitchFilter(MPU_I2C);
    DL_I2C_resetControllerTransfer(MPU_I2C);
    DL_I2C_setTimerPeriod(MPU_I2C, MPU_I2C_TIMER_PERIOD_100KHZ);
    DL_I2C_setControllerTXFIFOThreshold(
        MPU_I2C, DL_I2C_TX_FIFO_LEVEL_BYTES_1);
    DL_I2C_setControllerRXFIFOThreshold(
        MPU_I2C, DL_I2C_RX_FIFO_LEVEL_BYTES_1);
    DL_I2C_enableControllerClockStretching(MPU_I2C);
    DL_I2C_enableController(MPU_I2C);
}

static NOINLINE bool i2c_wait_idle(void)
{
    uint32_t timeout = MPU_I2C_TIMEOUT;

    while (timeout > 0U) {
        if ((DL_I2C_getControllerStatus(MPU_I2C) &
                DL_I2C_CONTROLLER_STATUS_IDLE) != 0U) {
            return true;
        }
        timeout--;
    }
    return false;
}

static NOINLINE bool i2c_wait_not_busy(void)
{
    uint32_t timeout = MPU_I2C_TIMEOUT;

    while (timeout > 0U) {
        if ((DL_I2C_getControllerStatus(MPU_I2C) &
                DL_I2C_CONTROLLER_STATUS_BUSY) == 0U) {
            return true;
        }
        timeout--;
    }
    return false;
}

static NOINLINE void i2c_recover_after_error(void)
{
    g_i2c_error_count++;
    DL_I2C_resetControllerTransfer(MPU_I2C);
    DL_I2C_flushControllerTXFIFO(MPU_I2C);
    DL_I2C_flushControllerRXFIFO(MPU_I2C);
}

static NOINLINE bool i2c_write_bytes(
    uint8_t address,
    const uint8_t *data,
    uint8_t length)
{
    if ((length == 0U) || (length > 8U) || !i2c_wait_idle()) {
        i2c_recover_after_error();
        return false;
    }

    DL_I2C_flushControllerTXFIFO(MPU_I2C);
    if (DL_I2C_fillControllerTXFIFO(MPU_I2C, data, length) != length) {
        i2c_recover_after_error();
        return false;
    }

    DL_I2C_startControllerTransfer(
        MPU_I2C, address, DL_I2C_CONTROLLER_DIRECTION_TX, length);
    delay_cycles(3U);
    if (!i2c_wait_not_busy() ||
        ((DL_I2C_getControllerStatus(MPU_I2C) &
            DL_I2C_CONTROLLER_STATUS_ERROR) != 0U)) {
        i2c_recover_after_error();
        return false;
    }
    return i2c_wait_idle();
}

static NOINLINE bool i2c_read_bytes(
    uint8_t address,
    uint8_t *data,
    uint8_t length)
{
    if ((length == 0U) || (length > 8U) || !i2c_wait_idle()) {
        i2c_recover_after_error();
        return false;
    }

    DL_I2C_flushControllerRXFIFO(MPU_I2C);
    DL_I2C_startControllerTransfer(
        MPU_I2C, address, DL_I2C_CONTROLLER_DIRECTION_RX, length);
    delay_cycles(3U);

    for (uint8_t i = 0U; i < length; i++) {
        uint32_t timeout = MPU_I2C_TIMEOUT;
        while (DL_I2C_isControllerRXFIFOEmpty(MPU_I2C) && (timeout > 0U)) {
            timeout--;
        }
        if (timeout == 0U) {
            i2c_recover_after_error();
            return false;
        }
        data[i] = DL_I2C_receiveControllerData(MPU_I2C);
    }

    if (!i2c_wait_not_busy() ||
        ((DL_I2C_getControllerStatus(MPU_I2C) &
            DL_I2C_CONTROLLER_STATUS_ERROR) != 0U)) {
        i2c_recover_after_error();
        return false;
    }
    return true;
}

static NOINLINE bool mpu_write_register(uint8_t reg, uint8_t value)
{
    uint8_t packet[2] = {reg, value};
    return i2c_write_bytes(MPU_I2C_ADDRESS, packet, 2U);
}

static NOINLINE bool mpu_read_registers(
    uint8_t first_reg,
    uint8_t *data,
    uint8_t length)
{
    if (!i2c_write_bytes(MPU_I2C_ADDRESS, &first_reg, 1U)) {
        return false;
    }
    delay_cycles(100U);
    return i2c_read_bytes(MPU_I2C_ADDRESS, data, length);
}

static NOINLINE bool mpu_read_gyro(void)
{
    uint8_t data[6];

    if (!mpu_read_registers(MPU_REG_GYRO_XOUT_H, data, 6U)) {
        return false;
    }
    g_gyro_x_raw = (int16_t) (((uint16_t) data[0] << 8) | data[1]);
    g_gyro_y_raw = (int16_t) (((uint16_t) data[2] << 8) | data[3]);
    g_gyro_z_raw = (int16_t) (((uint16_t) data[4] << 8) | data[5]);
    return true;
}

static NOINLINE bool mpu_init(void)
{
    uint8_t who_am_i = 0U;

    if (!mpu_read_registers(MPU_REG_WHO_AM_I, &who_am_i, 1U)) {
        g_mpu_error = 1U;
        return false;
    }
    g_mpu_who_am_i = who_am_i;
    if (who_am_i == MPU6050_WHO_AM_I) {
        g_mpu_device_type = 1U;
    } else if (who_am_i == MPU6500_WHO_AM_I) {
        g_mpu_device_type = 2U;
    } else {
        g_mpu_error = 2U;
        return false;
    }

    g_mpu_state = 1U;
    if (!mpu_write_register(MPU_REG_PWR_MGMT_1, 0x80U)) {
        g_mpu_error = 3U;
        return false;
    }
    delay_ms_rough(100U);

    if (!mpu_write_register(MPU_REG_PWR_MGMT_1, 0x01U) ||
        !mpu_write_register(MPU_REG_SMPLRT_DIV, 0x04U) ||
        !mpu_write_register(MPU_REG_CONFIG, 0x03U) ||
        !mpu_write_register(MPU_REG_GYRO_CONFIG, MPU_GYRO_FS_500DPS)) {
        g_mpu_error = 3U;
        return false;
    }
    delay_ms_rough(50U);
    g_mpu_link_ok = 1U;
    g_mpu_error = 0U;
    return true;
}

static NOINLINE bool mpu_calibrate_gyro_z(void)
{
    int64_t sum = 0;

    g_mpu_state = 2U;
    for (uint32_t i = 0U; i < MPU_CALIBRATION_SAMPLES; i++) {
        if (!mpu_read_gyro()) {
            g_mpu_error = 4U;
            return false;
        }
        sum += g_gyro_z_raw;
        led_set(((i / 50U) & 0x01U) != 0U);
        delay_ms_rough(4U);
    }

    g_gyro_z_bias_raw = (int32_t) (sum / MPU_CALIBRATION_SAMPLES);
    g_yaw_mdeg = 0;
    g_mpu_state = 3U;
    g_mpu_error = 0U;
    return true;
}
#endif

static NOINLINE void mpu_update_sample(void)
{
#if !MPU_ENABLE
    g_gyro_z_mdps = 0;
    return;
#else
    if (g_mpu_link_ok == 0U) {
        g_gyro_z_mdps = 0;
        return;
    }

    if (!mpu_read_gyro()) {
        g_mpu_read_fail_count++;
        g_mpu_error = 4U;
        if (g_mpu_read_fail_count >= 5U) {
            g_mpu_link_ok = 0U;
            g_mpu_state = 4U;
        }
        return;
    }

    g_mpu_read_fail_count = 0U;
    g_mpu_sample_count++;
    g_gyro_z_corrected_raw =
        (int32_t) g_gyro_z_raw - g_gyro_z_bias_raw;
    g_gyro_z_mdps = (g_gyro_z_corrected_raw * 2000) / 131;
    g_yaw_mdeg += (int32_t) (
        ((int64_t) g_gyro_z_mdps * GRAY_MAIN_LOOP_MS) / 1000);
#endif
}

static NOINLINE int32_t clamp_int32(
    int32_t value,
    int32_t minimum,
    int32_t maximum)
{
    if (value < minimum) {
        return minimum;
    }
    if (value > maximum) {
        return maximum;
    }
    return value;
}

static NOINLINE int32_t abs_int32(int32_t value)
{
    return (value < 0) ? -value : value;
}

static NOINLINE uint32_t clamp_pwm(int32_t ticks)
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

static NOINLINE uint32_t clamp_pwm_allow_slow_inner(int32_t ticks)
{
    if (ticks <= 0) {
        return 0U;
    }
    if (ticks > (int32_t) PWM_MAX_TICKS) {
        return PWM_MAX_TICKS;
    }
    return (uint32_t) ticks;
}

static NOINLINE void pwm_init(void)
{
    DL_TimerG_reset(PWM_TIMER);
    DL_TimerG_enablePower(PWM_TIMER);
    delay_cycles(16U);

    DL_GPIO_initPeripheralOutputFunction(
        IOMUX_PINCM34, IOMUX_PINCM34_PF_TIMG0_CCP0);
    DL_GPIO_enableOutput(PWM_PORT, PWMA_PIN);
    DL_GPIO_initPeripheralOutputFunction(
        IOMUX_PINCM35, IOMUX_PINCM35_PF_TIMG0_CCP1);
    DL_GPIO_enableOutput(PWM_PORT, PWMB_PIN);

    DL_TimerG_ClockConfig clock_config = {
        .clockSel = DL_TIMER_CLOCK_BUSCLK,
        .divideRatio = DL_TIMER_CLOCK_DIVIDE_1,
        .prescale = 0U,
    };
    DL_TimerG_setClockConfig(PWM_TIMER, &clock_config);

    DL_TimerG_PWMConfig pwm_config = {
        .pwmMode = DL_TIMER_PWM_MODE_EDGE_ALIGN,
        .period = PWM_PERIOD,
        .isTimerWithFourCC = false,
        .startTimer = DL_TIMER_STOP,
    };
    DL_TimerG_initPWMMode(PWM_TIMER, &pwm_config);

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

static NOINLINE void set_pwm(uint32_t left_ticks, uint32_t right_ticks)
{
    g_left_pwm_ticks = left_ticks;
    g_right_pwm_ticks = right_ticks;
    g_left_compare_ticks = PWM_PERIOD - left_ticks;
    g_right_compare_ticks = PWM_PERIOD - right_ticks;

    DL_TimerG_setCaptureCompareValue(
        PWM_TIMER, g_left_compare_ticks, DL_TIMER_CC_0_INDEX);
    DL_TimerG_setCaptureCompareValue(
        PWM_TIMER, g_right_compare_ticks, DL_TIMER_CC_1_INDEX);
}

static NOINLINE void motors_forward(void)
{
    DL_GPIO_clearPins(
        MOTOR_PORT, AIN1_PIN | AIN2_PIN | BIN1_PIN | BIN2_PIN);
    DL_GPIO_setPins(MOTOR_PORT, AIN1_PIN | BIN1_PIN | STBY_PIN);
}

static NOINLINE void motors_stop(void)
{
    set_pwm(0U, 0U);
    DL_GPIO_clearPins(MOTOR_PORT,
        AIN1_PIN | AIN2_PIN | BIN1_PIN | BIN2_PIN | STBY_PIN);
}

static NOINLINE void gray_sensor_init(void)
{
    init_output(IOMUX_PINCM18, GRAY_ADDRESS_PORT, GRAY_AD0_PIN, false);
    init_output(IOMUX_PINCM23, GRAY_ADDRESS_PORT, GRAY_AD1_PIN, false);
    init_output(IOMUX_PINCM24, GRAY_ADDRESS_PORT, GRAY_AD2_PIN, false);
    init_input(IOMUX_PINCM14);
}

static NOINLINE void gray_select_channel(uint8_t channel)
{
    uint32_t set_mask = 0U;

    DL_GPIO_clearPins(GRAY_ADDRESS_PORT, GRAY_ADDRESS_MASK);
    if ((channel & 0x01U) != 0U) {
        set_mask |= GRAY_AD0_PIN;
    }
    if ((channel & 0x02U) != 0U) {
        set_mask |= GRAY_AD1_PIN;
    }
    if ((channel & 0x04U) != 0U) {
        set_mask |= GRAY_AD2_PIN;
    }
    if (set_mask != 0U) {
        DL_GPIO_setPins(GRAY_ADDRESS_PORT, set_mask);
    }

    g_gray_selected_channel = (uint8_t) (channel + 1U);
    delay_us_rough(GRAY_ADDRESS_SETTLE_US);
}

static NOINLINE uint8_t gray_read_selected_channel(void)
{
    uint8_t high_count = 0U;

    for (uint8_t sample = 0U; sample < GRAY_SAMPLE_COUNT; sample++) {
        uint8_t level =
            ((DL_GPIO_readPins(GRAY_OUT_PORT, GRAY_OUT_PIN) & GRAY_OUT_PIN) != 0U)
            ? 1U
            : 0U;

        g_gray_live_out = level;
        high_count = (uint8_t) (high_count + level);
        delay_us_rough(GRAY_SAMPLE_INTERVAL_US);
    }

    return high_count;
}

static NOINLINE void gray_publish_channel_values(void)
{
    g_gray_x1 = ((g_gray_raw_high_mask & 0x01U) != 0U) ? 1U : 0U;
    g_gray_x2 = ((g_gray_raw_high_mask & 0x02U) != 0U) ? 1U : 0U;
    g_gray_x3 = ((g_gray_raw_high_mask & 0x04U) != 0U) ? 1U : 0U;
    g_gray_x4 = ((g_gray_raw_high_mask & 0x08U) != 0U) ? 1U : 0U;
    g_gray_x5 = ((g_gray_raw_high_mask & 0x10U) != 0U) ? 1U : 0U;
    g_gray_x6 = ((g_gray_raw_high_mask & 0x20U) != 0U) ? 1U : 0U;
    g_gray_x7 = ((g_gray_raw_high_mask & 0x40U) != 0U) ? 1U : 0U;
    g_gray_x8 = ((g_gray_raw_high_mask & 0x80U) != 0U) ? 1U : 0U;

    g_gray_x1_high_count = g_gray_channel_high_count[0];
    g_gray_x2_high_count = g_gray_channel_high_count[1];
    g_gray_x3_high_count = g_gray_channel_high_count[2];
    g_gray_x4_high_count = g_gray_channel_high_count[3];
    g_gray_x5_high_count = g_gray_channel_high_count[4];
    g_gray_x6_high_count = g_gray_channel_high_count[5];
    g_gray_x7_high_count = g_gray_channel_high_count[6];
    g_gray_x8_high_count = g_gray_channel_high_count[7];
}

static NOINLINE void gray_scan_all_channels(void)
{
    uint8_t high_mask = 0U;
    uint8_t unstable_mask = 0U;

    for (uint8_t channel = 0U; channel < GRAY_CHANNEL_COUNT; channel++) {
        uint8_t high_count;

        gray_select_channel(channel);
        high_count = gray_read_selected_channel();
        g_gray_channel_high_count[channel] = high_count;

        if (high_count >= GRAY_HIGH_MAJORITY) {
            high_mask |= (uint8_t) (1U << channel);
        }
        if ((high_count != 0U) && (high_count != GRAY_SAMPLE_COUNT)) {
            unstable_mask |= (uint8_t) (1U << channel);
        }
    }

    g_gray_raw_high_mask = high_mask;
    g_gray_unstable_mask = unstable_mask;
    g_gray_all_low = (high_mask == 0x00U) ? 1U : 0U;
    g_gray_all_high = (high_mask == 0xFFU) ? 1U : 0U;
    g_gray_mixed = ((high_mask != 0x00U) && (high_mask != 0xFFU)) ? 1U : 0U;
    gray_publish_channel_values();
    g_gray_scan_count++;
}

static NOINLINE void gray_update_line_position(uint8_t changed_mask)
{
    static const int16_t weights[GRAY_CHANNEL_COUNT] = {
        -3500, -2500, -1500, -500, 500, 1500, 2500, 3500
    };
    int32_t weighted_sum = 0;
    uint8_t count = 0U;

    for (uint8_t channel = 0U; channel < GRAY_CHANNEL_COUNT; channel++) {
        if ((changed_mask & (uint8_t) (1U << channel)) != 0U) {
            weighted_sum += weights[channel];
            count++;
        }
    }

    g_gray_changed_count = count;
    g_gray_line_abnormal = (count > LINE_MAX_ACTIVE) ? 1U : 0U;
    if ((count == 0U) || (count > LINE_MAX_ACTIVE)) {
        g_gray_line_valid = 0U;
        g_gray_line_mask = 0U;
        g_gray_line_error = g_line_last_error;
        g_line_center_confirm_count = 0U;
        return;
    }

    g_gray_line_valid = 1U;
    g_gray_line_mask = changed_mask;
    g_gray_line_error = (int16_t) (weighted_sum / (int32_t) count);
    g_line_last_error = g_gray_line_error;

    if (((changed_mask & LINE_CENTER_MASK) != 0U) &&
        (abs_int32(g_gray_line_error) <= LINE_CENTER_ERROR_MAX)) {
        if (g_line_center_confirm_count < LINE_CENTER_CONFIRM_FRAMES) {
            g_line_center_confirm_count++;
        }
    } else {
        g_line_center_confirm_count = 0U;
    }

    if (g_line_center_confirm_count >= LINE_CENTER_CONFIRM_FRAMES) {
        g_line_last_nonzero_error = 0;
    } else if (g_gray_line_error != 0) {
        g_line_last_nonzero_error = g_gray_line_error;
    }
}

static NOINLINE void gray_update_baseline_and_changes(void)
{
    if (g_gray_baseline_ready == 0U) {
        for (uint8_t channel = 0U; channel < GRAY_CHANNEL_COUNT; channel++) {
            if ((g_gray_raw_high_mask & (uint8_t) (1U << channel)) != 0U) {
                g_gray_baseline_votes[channel]++;
            }
        }

        g_gray_baseline_scan_count++;
        if (g_gray_baseline_scan_count >= GRAY_BASELINE_SCAN_COUNT) {
            uint8_t baseline_mask = 0U;

            for (uint8_t channel = 0U; channel < GRAY_CHANNEL_COUNT; channel++) {
                if (g_gray_baseline_votes[channel] >=
                    ((GRAY_BASELINE_SCAN_COUNT + 1U) / 2U)) {
                    baseline_mask |= (uint8_t) (1U << channel);
                }
            }

            g_gray_white_baseline_mask = baseline_mask;
            g_gray_baseline_uniform =
                ((baseline_mask == 0x00U) || (baseline_mask == 0xFFU)) ? 1U : 0U;
            g_gray_white_level_code =
                (baseline_mask == 0x00U) ? 0U :
                (baseline_mask == 0xFFU) ? 1U : 2U;
            g_gray_baseline_ready = 1U;
            g_gray_test_state = (g_gray_baseline_uniform != 0U) ? 2U : 3U;
            g_gray_changed_from_white_mask = 0U;
            g_gray_previous_change_mask = 0U;
        }
        return;
    }

    g_gray_changed_from_white_mask =
        (uint8_t) (g_gray_raw_high_mask ^ g_gray_white_baseline_mask);
    if (g_gray_changed_from_white_mask != g_gray_previous_change_mask) {
        g_gray_change_event_count++;
        g_gray_previous_change_mask = g_gray_changed_from_white_mask;
    }
    gray_update_line_position(g_gray_changed_from_white_mask);

    g_line_mask_history_2 = g_line_mask_history_1;
    g_line_mask_history_1 = g_line_mask_history_0;
    g_line_mask_history_0 = g_gray_line_mask;
    g_line_error_history_2 = g_line_error_history_1;
    g_line_error_history_1 = g_line_error_history_0;
    g_line_error_history_0 = g_gray_line_error;
}

static NOINLINE int32_t mpu_compute_control_correction(bool heading_enabled)
{
    int32_t heading_error = 0;

    if (g_mpu_link_ok == 0U) {
        g_gyro_rate_correction = 0;
        g_gyro_heading_correction = 0;
        g_mpu_control_correction = 0;
        return 0;
    }

    /* Installed orientation: positive Z rate means the vehicle turns left. */
    g_gyro_rate_correction = clamp_int32(
        g_gyro_z_mdps / GYRO_RATE_DIVISOR,
        -GYRO_RATE_CORR_LIMIT,
        GYRO_RATE_CORR_LIMIT);

    if (heading_enabled) {
        heading_error =
            g_yaw_mdeg - g_turn_exit_heading_target_mdeg;
        g_gyro_heading_correction = clamp_int32(
            heading_error / GYRO_HEADING_DIVISOR,
            -GYRO_HEADING_CORR_LIMIT,
            GYRO_HEADING_CORR_LIMIT);
    } else {
        g_gyro_heading_correction = 0;
    }

    g_mpu_control_correction = clamp_int32(
        g_gyro_rate_correction + g_gyro_heading_correction,
        -GYRO_TOTAL_CORR_LIMIT,
        GYRO_TOTAL_CORR_LIMIT);
    return g_mpu_control_correction;
}

static NOINLINE int32_t start_bridge_compute_mpu_correction(void)
{
    int32_t desired_correction;
    int32_t current_correction;
    int32_t correction_delta;
    bool releasing = false;

    if ((g_start_bridge_heading_active == 0U) ||
        (g_mpu_link_ok == 0U)) {
        g_start_bridge_heading_active = 0U;
        g_start_bridge_heading_error_mdeg = 0;
        g_start_bridge_filtered_gyro_z_mdps = 0;
        g_start_bridge_gyro_rate_correction = 0;
        g_start_bridge_heading_correction = 0;
        g_start_bridge_desired_correction = 0;
        g_start_bridge_mpu_correction = 0;
        g_start_bridge_right_confirm_count = 0U;
        g_start_bridge_correction_mode = 0U;
        return 0;
    }

    g_start_bridge_heading_error_mdeg =
        g_yaw_mdeg - g_start_bridge_heading_target_mdeg;
    g_start_bridge_filtered_gyro_z_mdps =
        ((START_BRIDGE_GYRO_FILTER_DIVISOR - 1) *
            g_start_bridge_filtered_gyro_z_mdps + g_gyro_z_mdps) /
        START_BRIDGE_GYRO_FILTER_DIVISOR;

    if (g_start_bridge_filtered_gyro_z_mdps >
        START_BRIDGE_GYRO_RATE_DEADBAND_MDPS) {
        g_start_bridge_gyro_rate_correction = clamp_int32(
            (g_start_bridge_filtered_gyro_z_mdps -
                START_BRIDGE_GYRO_RATE_DEADBAND_MDPS) /
                START_BRIDGE_GYRO_RATE_DIVISOR,
            0,
            START_BRIDGE_GYRO_RATE_LIMIT);
    } else if (g_start_bridge_filtered_gyro_z_mdps <
        -START_BRIDGE_GYRO_RATE_DEADBAND_MDPS) {
        g_start_bridge_gyro_rate_correction = clamp_int32(
            (g_start_bridge_filtered_gyro_z_mdps +
                START_BRIDGE_GYRO_RATE_DEADBAND_MDPS) /
                START_BRIDGE_GYRO_RATE_DIVISOR,
            -START_BRIDGE_GYRO_RATE_LIMIT,
            0);
    } else {
        g_start_bridge_gyro_rate_correction = 0;
    }

    if (g_start_bridge_heading_error_mdeg >
        START_BRIDGE_HEADING_DEADBAND_MDEG) {
        g_start_bridge_heading_correction = clamp_int32(
            (g_start_bridge_heading_error_mdeg -
                START_BRIDGE_HEADING_DEADBAND_MDEG) /
                START_BRIDGE_HEADING_DIVISOR,
            0,
            START_BRIDGE_HEADING_LIMIT);
        if (g_start_bridge_right_confirm_count <
            START_BRIDGE_RIGHT_CONFIRM_FRAMES) {
            g_start_bridge_right_confirm_count++;
        }
    } else if (g_start_bridge_heading_error_mdeg <
        -START_BRIDGE_HEADING_DEADBAND_MDEG) {
        g_start_bridge_heading_correction = clamp_int32(
            (g_start_bridge_heading_error_mdeg +
                START_BRIDGE_HEADING_DEADBAND_MDEG) /
                START_BRIDGE_HEADING_DIVISOR,
            -START_BRIDGE_HEADING_LIMIT,
            0);
        g_start_bridge_right_confirm_count = 0U;
    } else {
        g_start_bridge_heading_correction = 0;
        g_start_bridge_right_confirm_count = 0U;
    }

    desired_correction = clamp_int32(
        g_start_bridge_gyro_rate_correction +
            g_start_bridge_heading_correction,
        -START_BRIDGE_LEFT_CORR_LIMIT,
        START_BRIDGE_RIGHT_CORR_LIMIT);
    if ((desired_correction > 0) &&
        (g_start_bridge_right_confirm_count <
         START_BRIDGE_RIGHT_CONFIRM_FRAMES)) {
        desired_correction = 0;
    }
    g_start_bridge_desired_correction = desired_correction;

    current_correction = g_start_bridge_mpu_correction;
    if ((current_correction < 0) && (desired_correction >= 0)) {
        correction_delta = clamp_int32(
            -current_correction,
            0,
            START_BRIDGE_RELEASE_SLEW_TICKS);
        current_correction += correction_delta;
        releasing = true;
    } else if ((current_correction > 0) && (desired_correction <= 0)) {
        correction_delta = clamp_int32(
            current_correction,
            0,
            START_BRIDGE_RELEASE_SLEW_TICKS);
        current_correction -= correction_delta;
        releasing = true;
    } else if (current_correction < 0) {
        correction_delta = clamp_int32(
            desired_correction - current_correction,
            -START_BRIDGE_LEFT_SLEW_TICKS,
            START_BRIDGE_RELEASE_SLEW_TICKS);
        current_correction += correction_delta;
    } else if (current_correction > 0) {
        correction_delta = clamp_int32(
            desired_correction - current_correction,
            -START_BRIDGE_RELEASE_SLEW_TICKS,
            START_BRIDGE_RIGHT_SLEW_TICKS);
        current_correction += correction_delta;
    } else if (desired_correction < 0) {
        current_correction = clamp_int32(
            desired_correction,
            -START_BRIDGE_LEFT_SLEW_TICKS,
            0);
    } else if (desired_correction > 0) {
        current_correction = clamp_int32(
            desired_correction,
            0,
            START_BRIDGE_RIGHT_SLEW_TICKS);
    }

    g_start_bridge_mpu_correction = current_correction;
    if (releasing) {
        g_start_bridge_correction_mode = 2U;
    } else if (current_correction < 0) {
        g_start_bridge_correction_mode = 1U;
    } else if (current_correction > 0) {
        g_start_bridge_correction_mode = 3U;
    } else {
        g_start_bridge_correction_mode = 0U;
    }
    return g_start_bridge_mpu_correction;
}

static NOINLINE bool mask_is_right_exit(uint8_t mask)
{
    return ((mask & TURN_EXIT_RIGHT_HISTORY_MASK) != 0U) &&
        ((mask & (uint8_t) ~TURN_EXIT_RIGHT_HISTORY_MASK) == 0U);
}

static NOINLINE bool mask_is_left_exit(uint8_t mask)
{
    return ((mask & TURN_EXIT_LEFT_HISTORY_MASK) != 0U) &&
        ((mask & (uint8_t) ~TURN_EXIT_LEFT_HISTORY_MASK) == 0U);
}

static NOINLINE void curve_start(void)
{
    g_curve_active = 1U;
    g_gap_heading_active = 0U;
    g_gap_frame_count = 0U;
    g_curve_reentry_confirm_count = 0U;
    g_curve_entry_candidate_count = 0U;
    g_curve_entry_yaw_mdeg = g_yaw_mdeg;
    g_curve_target_yaw_mdeg =
        g_curve_entry_yaw_mdeg - CURVE_TARGET_ANGLE_MDEG;
    g_curve_angle_mdeg = 0;
    g_curve_feedforward_correction = 0;
    g_turn_exit_hold_count = 0U;
    g_turn_was_active = 0U;
    g_turn_exit_left_nudge_active = 0U;
    g_turn_exit_left_nudge_remaining = 0U;
    g_curve_start_count++;
}

static NOINLINE void curve_update_measurement(void)
{
    if ((g_curve_active != 0U) && (g_mpu_link_ok != 0U)) {
        /* The fixed A-B-C-D-A direction uses two clockwise semicircles. */
        g_curve_angle_mdeg = g_curve_entry_yaw_mdeg - g_yaw_mdeg;
    }
}

static NOINLINE void curve_update_entry_without_mpu(void)
{
    if ((g_mpu_link_ok != 0U) || (g_curve_active != 0U) ||
        (g_gap_heading_active != 0U)) {
        g_curve_entry_candidate_count = 0U;
        return;
    }

    if ((g_gray_line_valid != 0U) &&
        (abs_int32(g_gray_line_error) >= GRAY_CURVE_ENTRY_ERROR)) {
        if (g_curve_entry_candidate_count <
            GRAY_CURVE_ENTRY_CONFIRM_FRAMES) {
            g_curve_entry_candidate_count++;
        }
    } else {
        g_curve_entry_candidate_count = 0U;
    }

    if (g_curve_entry_candidate_count >=
        GRAY_CURVE_ENTRY_CONFIRM_FRAMES) {
        curve_start();
    }
}

static NOINLINE void curve_update_reentry(void)
{
    if (g_gap_heading_active == 0U) {
        g_curve_reentry_confirm_count = 0U;
        return;
    }

    if (g_gap_frame_count < GAP_REENTRY_MIN_FRAMES) {
        g_curve_reentry_confirm_count = 0U;
        return;
    }

    if ((g_gray_line_valid != 0U) &&
        (g_line_center_confirm_count >= LINE_CENTER_CONFIRM_FRAMES)) {
        if (g_curve_reentry_confirm_count <
            CURVE_REENTRY_CONFIRM_FRAMES) {
            g_curve_reentry_confirm_count++;
        }
    } else {
        g_curve_reentry_confirm_count = 0U;
    }

    if (g_curve_reentry_confirm_count >=
        CURVE_REENTRY_CONFIRM_FRAMES) {
        curve_start();
    }
}

static NOINLINE void curve_finish(void)
{
    bool had_curve = (g_curve_active != 0U);

    g_curve_exit_angle_mdeg = g_curve_angle_mdeg;
    g_curve_active = 0U;
    g_curve_feedforward_correction = 0;
    g_curve_reentry_confirm_count = 0U;
    g_gap_frame_count = 0U;

    if (had_curve) {
        g_turn_exit_heading_target_mdeg = g_curve_target_yaw_mdeg;
        g_gap_heading_active = 1U;
        g_curve_exit_count++;
    } else {
        g_turn_exit_heading_target_mdeg = g_yaw_mdeg;
        g_gap_heading_active = 0U;
    }

    g_turn_exit_hold_count = TURN_EXIT_HOLD_FRAMES;
    g_turn_was_active = 0U;
    g_turn_exit_left_nudge_active = 0U;
    g_turn_exit_left_nudge_remaining = 0U;
    g_curve_entry_candidate_count = 0U;
}

static NOINLINE void course_handle_mpu_runtime_loss(void)
{
    if ((g_course_previous_mpu_link_ok != 0U) &&
        (g_mpu_link_ok == 0U)) {
        g_curve_active = 0U;
        g_gap_heading_active = 0U;
        g_curve_reentry_confirm_count = 0U;
        g_gap_frame_count = 0U;
        g_curve_feedforward_correction = 0;
        g_turn_exit_candidate_direction = 0;
        g_turn_exit_candidate_lost_count = 0U;
        g_turn_exit_brake_active = 0U;
        g_turn_exit_brake_frame_count = 0U;
        g_turn_exit_hold_count = 0U;
        g_turn_was_active = 0U;
        g_turn_exit_left_nudge_active = 0U;
        g_turn_exit_left_nudge_remaining = 0U;
        g_mpu_control_correction = 0;
        g_total_control_correction = 0;
        g_mpu_runtime_degrade_count++;
    }
    g_course_previous_mpu_link_ok = g_mpu_link_ok;
}

static NOINLINE bool turn_exit_brake_handle(bool lost_recovery)
{
    bool right_history;
    bool left_history;
    bool no_black = (g_gray_changed_count == 0U);
    bool angle_ready = (g_curve_active == 0U) ||
        (g_mpu_link_ok == 0U) ||
        (g_curve_angle_mdeg >= CURVE_EXIT_MIN_ANGLE_MDEG);
    bool rate_reached_zero = false;
    bool timed_out = false;

    if (!no_black) {
        if (g_turn_exit_candidate_direction != 0) {
            g_turn_exit_candidate_cancel_count++;
        }
        g_turn_exit_candidate_direction = 0;
        g_turn_exit_candidate_lost_count = 0U;
    } else if (g_turn_exit_brake_active == 0U) {
        if (g_turn_exit_candidate_direction == 0) {
            right_history =
                mask_is_right_exit(g_line_mask_history_1) &&
                mask_is_right_exit(g_line_mask_history_2);
            left_history =
                mask_is_left_exit(g_line_mask_history_1) &&
                mask_is_left_exit(g_line_mask_history_2);

            g_turn_exit_candidate_direction = 0;
            g_turn_exit_candidate_lost_count = 0U;
            if (right_history &&
                ((g_mpu_link_ok == 0U) ||
                 (g_gyro_z_mdps < -TURN_EXIT_BRAKE_START_RATE_MDPS))) {
                g_turn_exit_candidate_direction = -1;
                g_turn_exit_candidate_lost_count = 1U;
            } else if (left_history &&
                ((g_mpu_link_ok == 0U) ||
                 (g_gyro_z_mdps > TURN_EXIT_BRAKE_START_RATE_MDPS))) {
                g_turn_exit_candidate_direction = 1;
                g_turn_exit_candidate_lost_count = 1U;
            }
            if ((g_turn_exit_candidate_direction == 0) &&
                (g_curve_active != 0U) &&
                (g_mpu_link_ok != 0U) &&
                (g_curve_angle_mdeg >= CURVE_EXIT_MIN_ANGLE_MDEG)) {
                g_turn_exit_candidate_direction = -1;
                g_turn_exit_candidate_lost_count = 1U;
                g_curve_forced_candidate_count++;
            }
        } else {
            g_turn_exit_candidate_lost_count++;
        }

        if ((g_turn_exit_candidate_direction != 0) &&
            (g_turn_exit_candidate_lost_count >=
             TURN_EXIT_CONFIRM_LOST_FRAMES) && angle_ready) {
            g_turn_exit_brake_active = 1U;
            g_turn_exit_brake_direction =
                g_turn_exit_candidate_direction;
            g_turn_exit_candidate_direction = 0;
            g_turn_exit_candidate_lost_count = 0U;
            g_turn_exit_candidate_confirm_count++;
            g_turn_exit_brake_frame_count = 0U;
            g_turn_exit_brake_trigger_count++;
            g_turn_exit_brake_stop_reason = 0U;
            g_turn_was_active = 1U;
        }
    }

    if (g_turn_exit_brake_active == 0U) {
        return false;
    }

    if (!lost_recovery) {
        g_turn_exit_brake_active = 0U;
        g_turn_exit_brake_stop_reason = 3U;
        return false;
    }

    g_turn_exit_brake_frame_count++;
    g_turn_mode = 3U;
    g_line_correction = 0;
    g_mpu_control_correction = 0;
    g_total_control_correction =
        (int32_t) g_turn_exit_brake_direction *
        TURN_EXIT_BRAKE_CORR_TICKS;
    set_pwm(
        clamp_pwm(
            (int32_t) PWM_LEFT_BASE_TICKS + g_total_control_correction),
        clamp_pwm(
            (int32_t) PWM_RIGHT_BASE_TICKS - g_total_control_correction));

    if (g_turn_exit_brake_frame_count >= TURN_EXIT_BRAKE_MIN_FRAMES) {
        if (g_mpu_link_ok != 0U) {
            if ((g_turn_exit_brake_direction < 0) &&
                (g_gyro_z_mdps >= -TURN_EXIT_BRAKE_STOP_RATE_MDPS)) {
                rate_reached_zero = true;
            } else if ((g_turn_exit_brake_direction > 0) &&
                (g_gyro_z_mdps <= TURN_EXIT_BRAKE_STOP_RATE_MDPS)) {
                rate_reached_zero = true;
            }
        } else if (g_turn_exit_brake_frame_count >=
            TURN_EXIT_BRAKE_NO_MPU_FRAMES) {
            rate_reached_zero = true;
        }
    }
    timed_out =
        (g_turn_exit_brake_frame_count >= TURN_EXIT_BRAKE_MAX_FRAMES);

    if (rate_reached_zero || timed_out) {
        g_turn_exit_brake_active = 0U;
        g_turn_exit_brake_stop_reason = rate_reached_zero ? 1U : 2U;
        g_line_last_nonzero_error = 0;
        curve_finish();
        g_total_control_correction = 0;
        set_pwm(PWM_LEFT_BASE_TICKS, PWM_RIGHT_BASE_TICKS);
    }
    return true;
}

static NOINLINE void line_follow_update(void)
{
    bool lost_recovery = (g_gray_line_valid == 0U) ||
        (g_gap_heading_active != 0U);
    bool center_locked =
        (!lost_recovery) &&
        (g_line_center_confirm_count >= LINE_CENTER_CONFIRM_FRAMES);
    bool gyro_enabled = false;
    bool heading_enabled = false;
    int32_t gyro_correction = 0;
    int32_t curve_feedforward = 0;
    int32_t sharp_correction_limit = LINE_SHARP_CORR_LIMIT;
    int32_t error = lost_recovery
        ? (int32_t) g_line_last_nonzero_error
        : (int32_t) g_gray_line_error;
    bool sharp_turn =
        (abs_int32(error) >= LINE_SHARP_ERROR) ||
        (lost_recovery && (error != 0));

    if (g_curve_active != 0U) {
        if (!lost_recovery && (g_curve_feedforward_correction <
            CURVE_RIGHT_FEEDFORWARD_TICKS)) {
            g_curve_feedforward_correction +=
                CURVE_FEEDFORWARD_STEP_TICKS;
            if (g_curve_feedforward_correction >
                CURVE_RIGHT_FEEDFORWARD_TICKS) {
                g_curve_feedforward_correction =
                    CURVE_RIGHT_FEEDFORWARD_TICKS;
            }
        }
        curve_feedforward = g_curve_feedforward_correction;
    }

    if (lost_recovery && (g_curve_active != 0U) &&
        (g_mpu_link_ok != 0U) &&
        (g_curve_angle_mdeg >= CURVE_EXIT_MIN_ANGLE_MDEG)) {
        /* Near the tangent, stop extending the last right-turn error. */
        error = 0;
        sharp_turn = false;
        curve_feedforward = 0;
    }

    if (turn_exit_brake_handle(lost_recovery)) {
        return;
    }

    if (g_gray_changed_count != 0U) {
        g_turn_exit_left_nudge_active = 0U;
        g_turn_exit_left_nudge_remaining = 0U;
    } else if ((g_line_lost_count == 1U) &&
        ((mask_is_right_exit(g_line_mask_history_1) &&
          mask_is_right_exit(g_line_mask_history_2)) ||
         (g_line_last_nonzero_error > 0))) {
        g_turn_exit_left_nudge_active = 1U;
        g_turn_exit_left_nudge_remaining = TURN_EXIT_LEFT_NUDGE_FRAMES;
        g_turn_exit_left_nudge_trigger_count++;
        g_line_last_nonzero_error = 0;
    }

    if (lost_recovery &&
        (g_turn_exit_left_nudge_remaining > 0U)) {
        g_turn_mode = 4U;
        g_turn_was_active = 1U;
        g_line_correction = -TURN_EXIT_LEFT_NUDGE_TICKS;
        g_gyro_rate_correction = 0;
        g_gyro_heading_correction = 0;
        g_mpu_control_correction = 0;
        g_total_control_correction = g_line_correction;
        set_pwm(
            clamp_pwm(
                (int32_t) PWM_LEFT_BASE_TICKS +
                    g_total_control_correction),
            clamp_pwm(
                (int32_t) PWM_RIGHT_BASE_TICKS -
                    g_total_control_correction));
        g_turn_exit_left_nudge_remaining--;
        if (g_turn_exit_left_nudge_remaining == 0U) {
            g_turn_exit_left_nudge_active = 0U;
        }
        return;
    }

    if (lost_recovery) {
        g_turn_mode = 2U;
        g_turn_was_active = 1U;
        if (g_line_lost_count > LINE_LOST_MEDIUM_FRAMES) {
            sharp_correction_limit = LINE_LOST_LONG_CORR_LIMIT;
        } else if (g_line_lost_count > LINE_LOST_STRONG_FRAMES) {
            sharp_correction_limit = LINE_LOST_MEDIUM_CORR_LIMIT;
        }
        g_line_lost_correction_limit = sharp_correction_limit;
    } else if (sharp_turn) {
        g_turn_mode = 1U;
        g_turn_was_active = 1U;
        g_turn_exit_hold_count = 0U;
        g_line_lost_correction_limit = LINE_SHARP_CORR_LIMIT;
    } else {
        g_turn_mode = 0U;
        g_line_lost_correction_limit = LINE_SHARP_CORR_LIMIT;
        if (center_locked && (g_turn_was_active != 0U) &&
            (g_curve_active == 0U)) {
            g_turn_exit_heading_target_mdeg = g_yaw_mdeg;
            g_turn_exit_hold_count = TURN_EXIT_HOLD_FRAMES;
            g_turn_was_active = 0U;
        }
    }

    if (g_mpu_link_ok != 0U) {
        if (center_locked && (g_curve_active == 0U)) {
            gyro_enabled = true;
        }
        if ((g_turn_exit_hold_count > 0U) &&
            (g_curve_active == 0U)) {
            gyro_enabled = true;
            heading_enabled = true;
        }
        if (g_gap_heading_active != 0U) {
            gyro_enabled = true;
            heading_enabled = true;
        }
        if (lost_recovery &&
            (g_line_lost_count > GYRO_LOST_ENABLE_FRAMES) &&
            (g_curve_active == 0U)) {
            gyro_enabled = true;
        }
    }
    if (gyro_enabled) {
        gyro_correction = mpu_compute_control_correction(heading_enabled);
    } else {
        g_gyro_rate_correction = 0;
        g_gyro_heading_correction = 0;
        g_mpu_control_correction = 0;
    }

    if (g_turn_exit_hold_count > 0U) {
        g_turn_exit_hold_count--;
    }

    if (lost_recovery && (error == 0)) {
        g_line_correction = 0;
        g_total_control_correction =
            curve_feedforward + gyro_correction;
        set_pwm(
            clamp_pwm(
                (int32_t) PWM_LEFT_BASE_TICKS + g_total_control_correction),
            clamp_pwm(
                (int32_t) PWM_RIGHT_BASE_TICKS - g_total_control_correction));
        return;
    }

    if (sharp_turn) {
        g_line_correction = clamp_int32(
            error / LINE_SHARP_KP_DIVISOR,
            -sharp_correction_limit,
            sharp_correction_limit);
        g_total_control_correction =
            g_line_correction + curve_feedforward + gyro_correction;
        set_pwm(
            clamp_pwm_allow_slow_inner(
                (int32_t) PWM_LEFT_BASE_TICKS +
                    g_total_control_correction),
            clamp_pwm_allow_slow_inner(
                (int32_t) PWM_RIGHT_BASE_TICKS -
                    g_total_control_correction));
        return;
    }

    g_line_correction = clamp_int32(
        error / LINE_KP_DIVISOR,
        -LINE_CORR_LIMIT,
        LINE_CORR_LIMIT);
    g_total_control_correction = clamp_int32(
        g_line_correction + curve_feedforward + gyro_correction,
        -(LINE_CORR_LIMIT + GYRO_TOTAL_CORR_LIMIT),
        LINE_CORR_LIMIT + GYRO_TOTAL_CORR_LIMIT);
    set_pwm(
        clamp_pwm(
            (int32_t) PWM_LEFT_BASE_TICKS + g_total_control_correction),
        clamp_pwm(
            (int32_t) PWM_RIGHT_BASE_TICKS - g_total_control_correction));
}

static NOINLINE void gray_update_led(void)
{
    if (g_phase == PHASE_CALIBRATION) {
        led_set(((g_gray_scan_count / 10U) & 0x01U) != 0U);
        return;
    }
    if (g_gray_test_state == 3U) {
        led_set(((g_gray_scan_count / 5U) & 0x01U) != 0U);
        return;
    }

    if (g_phase == PHASE_START_BRIDGE) {
        led_set(g_line_entry_confirm_count != 0U);
        return;
    }
    if (g_phase == PHASE_LINE_RUN) {
        if (g_turn_mode == 2U) {
            led_set(((g_run_frame / 5U) & 0x01U) != 0U);
        } else {
            led_set(true);
        }
        return;
    }

    led_set(false);
}

int main(void)
{
    SYSCFG_DL_init();
    g_firmware_version = FIRMWARE_VERSION;
    safe_outputs_init();
    gray_sensor_init();
    pwm_init();
    motors_stop();
#if MPU_ENABLE
    i2c0_init_100khz();
#endif
#if MPU_ENABLE
    if (!mpu_init() || !mpu_calibrate_gyro_z()) {
        g_mpu_state = 4U;
        g_mpu_link_ok = 0U;
    }
#else
    g_mpu_state = 4U;
    g_mpu_error = 0U;
    g_mpu_link_ok = 0U;
#endif
    g_course_previous_mpu_link_ok = g_mpu_link_ok;

    g_gray_test_state = 0U;
    g_phase = PHASE_INIT;
    led_set(true);
    delay_ms_rough(100U);
    led_set(false);
    delay_ms_rough(GRAY_POWERUP_DELAY_MS - 100U);
    g_gray_test_state = 1U;
    g_phase = PHASE_CALIBRATION;

    while (g_gray_baseline_ready == 0U) {
        gray_scan_all_channels();
        gray_update_baseline_and_changes();
        gray_update_led();
        delay_ms_rough(GRAY_MAIN_LOOP_MS);
    }

    if (g_gray_baseline_uniform == 0U) {
        g_phase = PHASE_STOP;
        g_stop_reason = STOP_REASON_BASELINE_INVALID;
        motors_stop();
        while (1) {
            led_set(true);
            delay_ms_rough(100U);
            led_set(false);
            delay_ms_rough(100U);
        }
    }

    led_set(true);
    delay_ms_rough(START_READY_HOLD_MS);
    led_set(false);

    g_phase = PHASE_START_BRIDGE;
    g_stop_reason = STOP_REASON_NONE;
    g_start_boost_active = 1U;
    g_start_bridge_heading_active = g_mpu_link_ok;
    g_start_bridge_heading_target_mdeg = g_yaw_mdeg;
    g_start_bridge_heading_error_mdeg = 0;
    g_start_bridge_filtered_gyro_z_mdps = 0;
    g_start_bridge_gyro_rate_correction = 0;
    g_start_bridge_heading_correction = 0;
    g_start_bridge_desired_correction = 0;
    g_start_bridge_mpu_correction = 0;
    g_start_bridge_right_confirm_count = 0U;
    g_start_bridge_correction_mode = 0U;
    g_start_bridge_left_cmd_ticks = START_BOOST_LEFT_TICKS;
    g_start_bridge_right_cmd_ticks = START_BOOST_RIGHT_TICKS;
    set_pwm(
        g_start_bridge_left_cmd_ticks,
        g_start_bridge_right_cmd_ticks);
    motors_forward();

    for (g_run_frame = 0U; g_run_frame < RUN_MAX_FRAMES; g_run_frame++) {
        mpu_update_sample();
        course_handle_mpu_runtime_loss();
        if ((g_gap_heading_active != 0U) &&
            (g_gap_frame_count < GAP_REENTRY_MAX_FRAMES)) {
            g_gap_frame_count++;
        }
        curve_update_measurement();
        gray_scan_all_channels();
        gray_update_baseline_and_changes();
        curve_update_reentry();

        if (g_phase == PHASE_START_BRIDGE) {
            int32_t bridge_correction;
            int32_t bridge_left_base;
            int32_t bridge_right_base;

            g_start_bridge_frame_count++;
            if (g_start_bridge_frame_count <= START_BOOST_FRAMES) {
                g_start_boost_active = 1U;
                bridge_left_base = START_BOOST_LEFT_TICKS;
                bridge_right_base = START_BOOST_RIGHT_TICKS;
            } else {
                g_start_boost_active = 0U;
                bridge_left_base = START_BRIDGE_LEFT_TICKS;
                bridge_right_base = START_BRIDGE_RIGHT_TICKS;
            }
            bridge_correction = start_bridge_compute_mpu_correction();
            g_start_bridge_left_cmd_ticks = clamp_pwm(
                bridge_left_base + bridge_correction);
            g_start_bridge_right_cmd_ticks = clamp_pwm(
                bridge_right_base - bridge_correction);
            set_pwm(
                g_start_bridge_left_cmd_ticks,
                g_start_bridge_right_cmd_ticks);

            if (g_gray_line_valid != 0U) {
                if (g_line_entry_confirm_count < LINE_ENTRY_CONFIRM_FRAMES) {
                    g_line_entry_confirm_count++;
                }
            } else {
                g_line_entry_confirm_count = 0U;
            }

            if (g_line_entry_confirm_count >= LINE_ENTRY_CONFIRM_FRAMES) {
                g_start_boost_active = 0U;
                g_start_bridge_heading_active = 0U;
                g_start_bridge_mpu_correction = 0;
                g_phase = PHASE_LINE_RUN;
                g_line_lost_count = 0U;
                curve_update_entry_without_mpu();
                line_follow_update();
            } else if (g_start_bridge_frame_count >= START_BRIDGE_MAX_FRAMES) {
                g_start_bridge_heading_active = 0U;
                g_start_bridge_mpu_correction = 0;
                g_phase = PHASE_STOP;
                g_stop_reason = STOP_REASON_BRIDGE_TIMEOUT;
                break;
            }
        } else if (g_phase == PHASE_LINE_RUN) {
            if (g_gray_line_valid != 0U) {
                g_line_lost_count = 0U;
            } else {
                g_line_lost_count++;
            }

            if ((g_gap_heading_active != 0U) &&
                (g_gap_frame_count >= GAP_REENTRY_MAX_FRAMES)) {
                g_phase = PHASE_STOP;
                g_stop_reason = STOP_REASON_GAP_TIMEOUT;
                break;
            }
            if ((g_curve_active != 0U) &&
                (g_line_lost_count >= CURVE_NO_LINE_MAX_FRAMES)) {
                g_phase = PHASE_STOP;
                g_stop_reason = STOP_REASON_CURVE_TIMEOUT;
                break;
            }
            if ((g_curve_active == 0U) &&
                (g_gap_heading_active == 0U) &&
                (g_line_lost_count >= LINE_LOST_MAX_FRAMES)) {
                g_phase = PHASE_STOP;
                g_stop_reason = STOP_REASON_LINE_LOST_TIMEOUT;
                break;
            }
            curve_update_entry_without_mpu();
            line_follow_update();
        }

        gray_update_led();
        delay_ms_rough(GRAY_MAIN_LOOP_MS);
    }

    if (g_stop_reason == STOP_REASON_NONE) {
        g_phase = PHASE_COMPLETE;
        g_stop_reason = STOP_REASON_TEST_COMPLETE;
    }
    g_start_boost_active = 0U;
    motors_stop();

    while (1) {
        led_set(true);
        delay_ms_rough(80U);
        led_set(false);
        delay_ms_rough(320U);
    }
}
