#ifndef OPENMV_H
#define OPENMV_H

#include <stdint.h>
#include <stdbool.h>
#include "config.h"

typedef struct {
    int16_t cx;
    int16_t cy;
    uint8_t detected;
    uint8_t frame_id;
} openmv_data_t;

void openmv_init(void);
bool openmv_data_ready(void);
openmv_data_t openmv_get_data(void);
void openmv_uart_isr(uint8_t byte);
void openmv_request(void);

#endif
