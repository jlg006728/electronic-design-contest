/*
 * 2021 F题第一阶段：新到陀螺仪独立验收。
 *
 * 只打开 I2C 和板载 LED；电机、PWM、舵机、灰度板、OpenMV UART 全部关闭。
 * 默认 AD0 接 GND，因此 I2C 地址为 0x68。程序会实读 WHO_AM_I：
 *   0x68 -> MPU6050
 *   0x70 -> MPU6500 兼容模块
 * 其他值直接进入故障状态。
 *
 * 接线：MPU VCC->MSPM0 3.3V，GND->GND，SDA->PA0，SCL->PA1，AD0->GND。
 * 禁止把 VCC 接 5V，禁止带电插拔。LaunchPad 跳帽沿用 V61：J4 OFF，
 * J19 1:2，J20 1:2；如实物板不同，以当前板卡丝印和 V61 工程配置为准。
 */

#include <stdbool.h>
#include <stdint.h>
#include "ti_msp_dl_config.h"

#define NOINLINE __attribute__((noinline))

#define FIRMWARE_VERSION 20260722U
#define CPU_CLOCK_HZ 32000000U

#define MPU_I2C I2C0
#define MPU_I2C_ADDRESS 0x68U
#define MPU_I2C_TIMEOUT 200000U
#define MPU_I2C_TIMER_PERIOD_100KHZ 31U

#define MPU_REG_SMPLRT_DIV 0x19U
#define MPU_REG_CONFIG 0x1AU
#define MPU_REG_GYRO_CONFIG 0x1BU
#define MPU_REG_GYRO_XOUT_H 0x43U
#define MPU_REG_PWR_MGMT_1 0x6BU
#define MPU_REG_WHO_AM_I 0x75U

#define MPU6050_WHO_AM_I 0x68U
#define MPU6500_WHO_AM_I 0x70U
#define MPU_GYRO_FS_500DPS 0x08U
#define MPU_CALIBRATION_SAMPLES 500U
#define MPU_SAMPLE_PERIOD_MS 20U
#define MPU_GYRO_STATIONARY_LIMIT_MDPS 3000

#define MOTOR_PORT GPIOB
#define AIN1_PIN DL_GPIO_PIN_0
#define AIN2_PIN DL_GPIO_PIN_1
#define BIN1_PIN DL_GPIO_PIN_2
#define BIN2_PIN DL_GPIO_PIN_3
#define STBY_PIN DL_GPIO_PIN_4

#define LED_PORT GPIOA
#define LED_PIN DL_GPIO_PIN_15

#define BUZZER_PORT GPIOB
#define BUZZER_PIN DL_GPIO_PIN_11

volatile uint32_t g_firmware_version = FIRMWARE_VERSION;
volatile uint8_t g_motor_disabled = 1U;
volatile uint8_t g_mpu_state = 0U;
volatile uint8_t g_mpu_error = 0U;
volatile uint8_t g_mpu_link_ok = 0U;
volatile uint8_t g_mpu_who_am_i = 0U;
volatile uint8_t g_mpu_device_type = 0U;
volatile uint32_t g_i2c_error_count = 0U;
volatile uint32_t g_mpu_sample_count = 0U;
volatile uint32_t g_mpu_read_fail_count = 0U;
volatile uint32_t g_mpu_read_fail_total = 0U;

volatile int16_t g_gyro_x_raw = 0;
volatile int16_t g_gyro_y_raw = 0;
volatile int16_t g_gyro_z_raw = 0;
volatile int32_t g_gyro_z_bias_raw = 0;
volatile int32_t g_gyro_z_corrected_raw = 0;
volatile int32_t g_gyro_z_mdps = 0;
volatile int32_t g_yaw_mdeg = 0;
volatile uint32_t g_stationary_sample_count = 0U;
volatile uint32_t g_motion_sample_count = 0U;
volatile uint32_t g_calibration_sample_count = 0U;
volatile uint8_t g_led_state = 0U;

static NOINLINE void delay_ms(uint32_t ms)
{
    for (uint32_t i = 0U; i < ms; i++) {
        delay_cycles(CPU_CLOCK_HZ / 1000U);
    }
}

static NOINLINE void init_output(uint32_t pincm, GPIO_Regs *port, uint32_t pin, bool high)
{
    DL_GPIO_initDigitalOutput(pincm);
    if (high) {
        DL_GPIO_setPins(port, pin);
    } else {
        DL_GPIO_clearPins(port, pin);
    }
    DL_GPIO_enableOutput(port, pin);
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
    init_output(IOMUX_PINCM34, GPIOA, DL_GPIO_PIN_12, false);
    init_output(IOMUX_PINCM35, GPIOA, DL_GPIO_PIN_13, false);
    init_output(IOMUX_PINCM36, GPIOA, DL_GPIO_PIN_14, false);
    init_output(IOMUX_PINCM39, GPIOA, DL_GPIO_PIN_17, false);
    init_output(IOMUX_PINCM28, BUZZER_PORT, BUZZER_PIN, true);
    init_output(IOMUX_PINCM37, LED_PORT, LED_PIN, false);
}

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
        IOMUX_PINCM1, IOMUX_PINCM1_PF_I2C0_SDA,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_NONE,
        DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initPeripheralInputFunctionFeatures(
        IOMUX_PINCM2, IOMUX_PINCM2_PF_I2C0_SCL,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_NONE,
        DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_enableHiZ(IOMUX_PINCM1);
    DL_GPIO_enableHiZ(IOMUX_PINCM2);
    DL_I2C_setClockConfig(MPU_I2C, &clock_config);
    DL_I2C_disableAnalogGlitchFilter(MPU_I2C);
    DL_I2C_resetControllerTransfer(MPU_I2C);
    DL_I2C_setTimerPeriod(MPU_I2C, MPU_I2C_TIMER_PERIOD_100KHZ);
    DL_I2C_setControllerTXFIFOThreshold(MPU_I2C, DL_I2C_TX_FIFO_LEVEL_BYTES_1);
    DL_I2C_setControllerRXFIFOThreshold(MPU_I2C, DL_I2C_RX_FIFO_LEVEL_BYTES_1);
    DL_I2C_enableControllerClockStretching(MPU_I2C);
    DL_I2C_enableController(MPU_I2C);
}

static NOINLINE bool i2c_wait_idle(void)
{
    uint32_t timeout = MPU_I2C_TIMEOUT;
    while (timeout > 0U) {
        if ((DL_I2C_getControllerStatus(MPU_I2C) & DL_I2C_CONTROLLER_STATUS_IDLE) != 0U) {
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
        if ((DL_I2C_getControllerStatus(MPU_I2C) & DL_I2C_CONTROLLER_STATUS_BUSY) == 0U) {
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

static NOINLINE bool i2c_write_bytes(uint8_t address, const uint8_t *data, uint8_t length)
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
    DL_I2C_startControllerTransfer(MPU_I2C, address, DL_I2C_CONTROLLER_DIRECTION_TX, length);
    delay_cycles(3U);
    if (!i2c_wait_not_busy() || ((DL_I2C_getControllerStatus(MPU_I2C) & DL_I2C_CONTROLLER_STATUS_ERROR) != 0U)) {
        i2c_recover_after_error();
        return false;
    }
    return i2c_wait_idle();
}

static NOINLINE bool i2c_read_bytes(uint8_t address, uint8_t *data, uint8_t length)
{
    if ((length == 0U) || (length > 8U) || !i2c_wait_idle()) {
        i2c_recover_after_error();
        return false;
    }
    DL_I2C_flushControllerRXFIFO(MPU_I2C);
    DL_I2C_startControllerTransfer(MPU_I2C, address, DL_I2C_CONTROLLER_DIRECTION_RX, length);
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
    if (!i2c_wait_not_busy() || ((DL_I2C_getControllerStatus(MPU_I2C) & DL_I2C_CONTROLLER_STATUS_ERROR) != 0U)) {
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

static NOINLINE bool mpu_read_registers(uint8_t first_reg, uint8_t *data, uint8_t length)
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
    g_gyro_x_raw = (int16_t)(((uint16_t)data[0] << 8) | data[1]);
    g_gyro_y_raw = (int16_t)(((uint16_t)data[2] << 8) | data[3]);
    g_gyro_z_raw = (int16_t)(((uint16_t)data[4] << 8) | data[5]);
    return true;
}

static NOINLINE bool mpu_init(void)
{
    if (!mpu_read_registers(MPU_REG_WHO_AM_I, (uint8_t *)&g_mpu_who_am_i, 1U)) {
        g_mpu_error = 1U;
        return false;
    }
    if (g_mpu_who_am_i == MPU6050_WHO_AM_I) {
        g_mpu_device_type = 1U;
    } else if (g_mpu_who_am_i == MPU6500_WHO_AM_I) {
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
    delay_ms(100U);
    if (!mpu_write_register(MPU_REG_PWR_MGMT_1, 0x01U) ||
        !mpu_write_register(MPU_REG_SMPLRT_DIV, 0x04U) ||
        !mpu_write_register(MPU_REG_CONFIG, 0x03U) ||
        !mpu_write_register(MPU_REG_GYRO_CONFIG, MPU_GYRO_FS_500DPS)) {
        g_mpu_error = 3U;
        return false;
    }
    delay_ms(50U);
    g_mpu_link_ok = 1U;
    g_mpu_error = 0U;
    return true;
}

static NOINLINE bool mpu_calibrate_gyro_z(void)
{
    int64_t sum = 0;
    g_mpu_state = 2U;
    g_gyro_z_bias_raw = 0;
    g_calibration_sample_count = 0U;
    for (uint32_t i = 0U; i < MPU_CALIBRATION_SAMPLES; i++) {
        if (!mpu_read_gyro()) {
            g_mpu_error = 4U;
            return false;
        }
        sum += g_gyro_z_raw;
        g_calibration_sample_count = i + 1U;
        led_set(((i / 50U) & 0x01U) != 0U);
        delay_ms(4U);
    }
    g_gyro_z_bias_raw = (int32_t)(sum / MPU_CALIBRATION_SAMPLES);
    g_yaw_mdeg = 0;
    g_mpu_state = 3U;
    g_mpu_error = 0U;
    return true;
}

int main(void)
{
    SYSCFG_DL_init();
    safe_outputs_init();
    i2c0_init_100khz();
    led_set(true);
    delay_ms(100U);
    led_set(false);
    delay_ms(200U);

    if (!mpu_init() || !mpu_calibrate_gyro_z()) {
        g_mpu_state = 4U;
        g_mpu_link_ok = 0U;
        while (1) {
            led_set(true);
            delay_ms(100U);
            led_set(false);
            delay_ms(100U);
        }
    }

    while (1) {
        if (mpu_read_gyro()) {
            g_mpu_read_fail_count = 0U;
            g_mpu_sample_count++;
            g_gyro_z_corrected_raw = (int32_t)g_gyro_z_raw - g_gyro_z_bias_raw;
            /* +/-500 dps: 65.5 LSB/(degree/s), result in mdps. */
            g_gyro_z_mdps = (g_gyro_z_corrected_raw * 2000) / 131;
            g_yaw_mdeg += (int32_t)(((int64_t)g_gyro_z_mdps * MPU_SAMPLE_PERIOD_MS) / 1000);
            if ((g_gyro_z_mdps >= -MPU_GYRO_STATIONARY_LIMIT_MDPS) &&
                (g_gyro_z_mdps <= MPU_GYRO_STATIONARY_LIMIT_MDPS)) {
                g_stationary_sample_count++;
            } else {
                g_stationary_sample_count = 0U;
                g_motion_sample_count++;
            }
            led_set((g_mpu_sample_count % 50U) < 5U);
        } else {
            g_mpu_read_fail_count++;
            g_mpu_read_fail_total++;
            g_mpu_error = 4U;
            led_set(((g_mpu_read_fail_count / 2U) & 0x01U) != 0U);
        }
        delay_ms(MPU_SAMPLE_PERIOD_MS);
    }
}
