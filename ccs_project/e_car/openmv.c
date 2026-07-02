#include "openmv.h"
#include "board.h"

static uint8_t g_buf[OPENMV_FRAME_LENGTH];
static uint8_t g_idx;
static openmv_data_t g_data;
static volatile bool g_ready;

static uint8_t frame_checksum(const uint8_t *buf)
{
    uint8_t x = 0U;
    for (uint8_t i = 1U; i < (OPENMV_FRAME_LENGTH - 2U); i++) {
        x ^= buf[i];
    }
    return x;
}

void openmv_init(void)
{
    g_idx = 0U;
    g_ready = false;
    g_data.cx = -1;
    g_data.cy = -1;
    g_data.detected = 0U;
    g_data.frame_id = 0U;
}

void openmv_uart_isr(uint8_t byte)
{
    if (g_idx == 0U) {
        if (byte == OPENMV_FRAME_HEADER) {
            g_buf[g_idx++] = byte;
        }
        return;
    }

    g_buf[g_idx++] = byte;

    if (g_idx >= OPENMV_FRAME_LENGTH) {
        g_idx = 0U;
        if (g_buf[OPENMV_FRAME_LENGTH - 1U] != OPENMV_FRAME_FOOTER) {
            return;
        }
        if (g_buf[OPENMV_FRAME_LENGTH - 2U] != frame_checksum(g_buf)) {
            return;
        }

        g_data.detected = g_buf[1];
        g_data.cx = (int16_t)((uint16_t)g_buf[2] | ((uint16_t)g_buf[3] << 8));
        g_data.cy = (int16_t)((uint16_t)g_buf[4] | ((uint16_t)g_buf[5] << 8));
        g_data.frame_id++;
        g_ready = true;
    }
}

bool openmv_data_ready(void)
{
    return g_ready;
}

openmv_data_t openmv_get_data(void)
{
    openmv_data_t data = g_data;
    g_ready = false;
    return data;
}

openmv_data_t openmv_peek_data(void)
{
    return g_data;
}

void openmv_request(void)
{
    board_uart_send_byte((uint8_t)OPENMV_REQUEST_BYTE);
}
