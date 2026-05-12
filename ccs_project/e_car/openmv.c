#include "openmv.h"
#include "ti_msp_dl_config.h"

static uint8_t g_buf[OPENMV_FRAME_LENGTH];
static uint8_t g_idx;
static openmv_data_t g_data;
static volatile bool g_ready;

void openmv_init(void)
{
    g_idx = 0;
    g_ready = false;
    g_data.cx = 0;
    g_data.cy = 0;
    g_data.detected = 0;
    g_data.frame_id = 0;
}

void openmv_uart_isr(uint8_t byte)
{
    if (g_idx == 0) {
        if (byte == OPENMV_FRAME_HEADER) {
            g_buf[g_idx++] = byte;
        }
        return;
    }

    g_buf[g_idx++] = byte;

    if (g_idx >= OPENMV_FRAME_LENGTH) {
        g_idx = 0;
        if (g_buf[OPENMV_FRAME_LENGTH - 1] == OPENMV_FRAME_FOOTER) {
            g_data.cx       = (int16_t)((g_buf[1] << 8) | g_buf[2]);
            g_data.cy       = (int16_t)((g_buf[3] << 8) | g_buf[4]);
            g_data.detected = g_buf[5];
            g_data.frame_id++;
            g_ready = true;
        }
    }
}

bool openmv_data_ready(void)  { return g_ready; }

openmv_data_t openmv_get_data(void)
{
    g_ready = false;
    return g_data;
}

void openmv_request(void)
{
    DL_UART_transmitDataBlocking(UART_OPENMV_INST, 0x55);
}
