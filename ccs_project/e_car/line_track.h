#ifndef LINE_TRACK_H
#define LINE_TRACK_H

#include <stdbool.h>
#include <stdint.h>
#include "config.h"

void line_track_init(void);
void line_track_update(void);
uint8_t line_track_get_mask(void);
float line_track_get_position(void);
bool line_track_is_lost(void);
float line_track_update_position_pid(float position_error);

#endif
