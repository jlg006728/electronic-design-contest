#ifndef STATE_MACHINE_H
#define STATE_MACHINE_H

#include <stdint.h>
#include <stdbool.h>
#include "config.h"

typedef enum {
    STATE_IDLE,
    STATE_TRACK_ONLY,
    STATE_AIM_STATIC,
    STATE_TRACK_AIM,
    STATE_TRACK_DRAW,
    STATE_STOP
} system_state_t;

void sm_init(void);
system_state_t sm_get_state(void);
system_mode_t sm_get_mode(void);
uint8_t sm_get_laps(void);
void sm_set_laps(uint8_t n);
void sm_button_mode(void);
void sm_button_start(void);
void sm_update(uint32_t sys_time_ms);

#endif
