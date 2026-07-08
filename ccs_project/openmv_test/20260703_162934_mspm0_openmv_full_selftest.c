/*
 * MSPM0G3507 <-> OpenMV full self-test.
 *
 * This test isolates OpenMV. Motors are disabled and PA14/PA17 servo signals
 * are driven low. The program only tests UART link + camera status frames.
 *
 * Wiring:
 *   MSPM0 PA10 / UART0_TX -> OpenMV P5 / UART3_RX
 *   MSPM0 PA11 / UART0_RX <- OpenMV P4 / UART3_TX
 *   MSPM0 GND             -> OpenMV GND
 *
 * Important:
 *   Remove LaunchPad J21/J22 backchannel UART jumpers before connecting
 *   OpenMV to PA10/PA11, otherwise XDS110 can interfere with UART0.
 *
 * Watch variables:
 *   g_omv_link_ok
 *   g_omv_frame_count
 *   g_omv_last_cmd
 *   g_omv_frame_type
 *   g_omv_camera_ok
 *   g_omv_target_detected
 *   g_omv_cx
 *   g_omv_cy
 *   g_omv_pixels
 *   g_omv_fps_x10
 *   g_omv_l_mean
 *   g_omv_bad_checksum
 *   g_omv_bad_footer
 *   g_omv_timeout_count
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

#define OMV_CMD_PING                ((uint8_t) 'P')
#define OMV_CMD_STATS               ((uint8_t) 'S')
#define OMV_CMD_COLOR               ((uint8_t) 'C')

#define OMV_FLAG_CAMERA_OK          0x01U
#define OMV_FLAG_TARGET_DETECTED    0x02U
#define OMV_FLAG_UART_RX_SEEN       0x04U

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
volatile uint32_t g_omv_bad_checksum = 0;
volatile uint32_t g_omv_bad_footer = 0;
volatile uint32_t g_omv_sync_drop = 0;
volatile uint32_t g_omv_timeout_count = 0;

volatile uint8_t g_omv_link_ok = 0;
volatile uint8_t g_omv_last_cmd = 0;
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
volatile uint8_t g_led_state = 0;

static uint8_t s_rx_buf[OMV_FRAME_LENGTH];
static uint8_t s_rx_idx = 0;
static uint8_t s_cmd_index = 0;

static const uint8_t s_cmd_cycle[3] = {
    OMV_CMD_PING,
    OMV_CMD_STATS,
    OMV_CMD_COLOR,
};

static NOINLINE void delay_ms(uint32_t ms)
{
    for (uint32_t i = 0; i < ms; i++) {
        delay_cycles(32000U);
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

static NOINLINE void safe_servo_outputs_init(void)
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
    for (uint8_t i = 0; i < 3U; i++) {
        led_set(true);
        delay_ms(80U);
        led_set(false);
        delay_ms(80U);
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
    DL_UART_Main_enable(UART_OPENMV);
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
    g_omv_timeout_count = 0U;
    led_toggle();
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

static NOINLINE void omv_poll_rx(void)
{
    while (!DL_UART_Main_isRXFIFOEmpty(UART_OPENMV)) {
        omv_parse_byte(DL_UART_Main_receiveDataBlocking(UART_OPENMV));
    }
}

static NOINLINE void omv_send_command(uint8_t cmd)
{
    g_omv_last_cmd = cmd;
    DL_UART_Main_transmitDataBlocking(UART_OPENMV, cmd);
    g_omv_tx_requests++;
}

int main(void)
{
    SYSCFG_DL_init();

    safe_motor_outputs_init();
    safe_servo_outputs_init();
    init_output(IOMUX_PINCM37, LED_PORT, LED_PIN, false);
    uart0_openmv_init();

    startup_blink();

    while (1) {
        uint32_t frames_before = g_omv_frame_count;
        uint8_t cmd = s_cmd_cycle[s_cmd_index];

        omv_send_command(cmd);
        s_cmd_index++;
        if (s_cmd_index >= 3U) {
            s_cmd_index = 0U;
        }

        for (uint16_t i = 0U; i < 250U; i++) {
            omv_poll_rx();
            delay_ms(1U);
        }

        if (g_omv_frame_count == frames_before) {
            g_omv_timeout_count++;
            if (g_omv_timeout_count >= 8U) {
                g_omv_link_ok = 0U;
                led_set(false);
            }
        }
    }
}
