/*
 * MSPM0G3507 <-> OpenMV UART test.
 *
 * Wiring:
 *   MSPM0 PA10 / UART0_TX -> OpenMV P5 / UART3_RX
 *   MSPM0 PA11 / UART0_RX <- OpenMV P4 / UART3_TX
 *   MSPM0 GND             -> OpenMV GND
 *
 * Protocol:
 *   MSPM0 sends one request byte 'X' every 100 ms.
 *   OpenMV replies:
 *     [0x2C, detected, cx_l, cx_h, cy_l, cy_h, xor, 0x5B]
 *
 * Watch variables:
 *   g_openmv_tx_requests
 *   g_openmv_rx_bytes
 *   g_openmv_frame_count
 *   g_openmv_detected
 *   g_openmv_cx
 *   g_openmv_cy
 *   g_openmv_bad_checksum
 *   g_openmv_bad_footer
 *   g_openmv_link_ok
 */

#include <stdbool.h>
#include <stdint.h>
#include "ti_msp_dl_config.h"

#define SYSCLK_HZ               32000000U
#define UART_OPENMV             UART0
#define UART_BAUDRATE           115200U

#define OPENMV_REQUEST_BYTE     ((uint8_t)'X')
#define OPENMV_FRAME_HEADER     0x2CU
#define OPENMV_FRAME_FOOTER     0x5BU
#define OPENMV_FRAME_LENGTH     8U

#define LED_PORT                GPIOA
#define LED_PIN                 DL_GPIO_PIN_15

#define MOTOR_PORT              GPIOB
#define AIN1_PIN                DL_GPIO_PIN_0
#define AIN2_PIN                DL_GPIO_PIN_1
#define BIN1_PIN                DL_GPIO_PIN_2
#define BIN2_PIN                DL_GPIO_PIN_3
#define STBY_PIN                DL_GPIO_PIN_4

volatile uint32_t g_openmv_tx_requests = 0;
volatile uint32_t g_openmv_rx_bytes = 0;
volatile uint32_t g_openmv_frame_count = 0;
volatile uint32_t g_openmv_bad_footer = 0;
volatile uint32_t g_openmv_bad_checksum = 0;
volatile uint32_t g_openmv_sync_drop = 0;
volatile uint32_t g_openmv_miss_count = 0;
volatile uint8_t g_openmv_link_ok = 0;
volatile uint8_t g_openmv_detected = 0;
volatile int16_t g_openmv_cx = -1;
volatile int16_t g_openmv_cy = -1;
volatile int16_t g_openmv_dx = 0;
volatile int16_t g_openmv_dy = 0;
volatile uint8_t g_openmv_last_frame[OPENMV_FRAME_LENGTH] = {0};
volatile uint8_t g_led_state = 0;

static uint8_t s_rx_buf[OPENMV_FRAME_LENGTH];
static uint8_t s_rx_idx = 0;

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

static uint8_t frame_checksum(const uint8_t *buf)
{
    uint8_t x = 0U;
    for (uint8_t i = 1U; i <= 5U; i++) {
        x ^= buf[i];
    }
    return x;
}

static void led_set(bool on)
{
    if (on) {
        DL_GPIO_setPins(LED_PORT, LED_PIN);
        g_led_state = 1U;
    } else {
        DL_GPIO_clearPins(LED_PORT, LED_PIN);
        g_led_state = 0U;
    }
}

static void led_toggle(void)
{
    DL_GPIO_togglePins(LED_PORT, LED_PIN);
    g_led_state = (g_led_state == 0U) ? 1U : 0U;
}

static void safe_motor_outputs_init(void)
{
    init_output(IOMUX_PINCM12, MOTOR_PORT, AIN1_PIN, false);
    init_output(IOMUX_PINCM13, MOTOR_PORT, AIN2_PIN, false);
    init_output(IOMUX_PINCM15, MOTOR_PORT, BIN1_PIN, false);
    init_output(IOMUX_PINCM16, MOTOR_PORT, BIN2_PIN, false);
    init_output(IOMUX_PINCM17, MOTOR_PORT, STBY_PIN, false);
}

static void uart0_openmv_init(void)
{
    DL_UART_Main_reset(UART_OPENMV);
    DL_UART_Main_enablePower(UART_OPENMV);
    delay_cycles(16);

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
    DL_UART_Main_enable(UART_OPENMV);
}

static void openmv_parse_byte(uint8_t byte)
{
    g_openmv_rx_bytes++;

    if (s_rx_idx == 0U) {
        if (byte != OPENMV_FRAME_HEADER) {
            g_openmv_sync_drop++;
            return;
        }
    }

    s_rx_buf[s_rx_idx++] = byte;

    if (s_rx_idx < OPENMV_FRAME_LENGTH) {
        return;
    }

    s_rx_idx = 0U;

    for (uint8_t i = 0U; i < OPENMV_FRAME_LENGTH; i++) {
        g_openmv_last_frame[i] = s_rx_buf[i];
    }

    if (s_rx_buf[7] != OPENMV_FRAME_FOOTER) {
        g_openmv_bad_footer++;
        return;
    }

    if (s_rx_buf[6] != frame_checksum(s_rx_buf)) {
        g_openmv_bad_checksum++;
        return;
    }

    g_openmv_detected = s_rx_buf[1];
    g_openmv_cx = (int16_t)((uint16_t)s_rx_buf[2] | ((uint16_t)s_rx_buf[3] << 8));
    g_openmv_cy = (int16_t)((uint16_t)s_rx_buf[4] | ((uint16_t)s_rx_buf[5] << 8));

    if (g_openmv_detected != 0U) {
        g_openmv_dx = (int16_t)(g_openmv_cx - 160);
        g_openmv_dy = (int16_t)(g_openmv_cy - 120);
    } else {
        g_openmv_dx = 0;
        g_openmv_dy = 0;
    }

    g_openmv_frame_count++;
    g_openmv_link_ok = 1U;
    g_openmv_miss_count = 0U;
}

static void openmv_poll_rx(void)
{
    while (!DL_UART_Main_isRXFIFOEmpty(UART_OPENMV)) {
        openmv_parse_byte(DL_UART_Main_receiveDataBlocking(UART_OPENMV));
    }
}

static void openmv_send_request(void)
{
    DL_UART_Main_transmitDataBlocking(UART_OPENMV, OPENMV_REQUEST_BYTE);
    g_openmv_tx_requests++;
}

int main(void)
{
    SYSCFG_DL_init();

    safe_motor_outputs_init();
    init_output(IOMUX_PINCM37, LED_PORT, LED_PIN, false);
    uart0_openmv_init();

    led_set(false);
    delay_ms(500);

    while (1) {
        uint32_t frames_before = g_openmv_frame_count;

        openmv_send_request();

        for (uint32_t i = 0U; i < 100U; i++) {
            openmv_poll_rx();
            delay_ms(1);
        }

        if (g_openmv_frame_count != frames_before) {
            led_toggle();
        } else {
            g_openmv_miss_count++;
            if (g_openmv_miss_count >= 10U) {
                g_openmv_link_ok = 0U;
                led_set(false);
            }
        }
    }
}
