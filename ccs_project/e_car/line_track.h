#ifndef LINE_TRACK_H
#define LINE_TRACK_H

#include <stdint.h>
#include "config.h"

void line_track_init(void);
void line_track_read(uint16_t sensor_values[LINE_SENSOR_COUNT]);
float line_track_get_position(void);
float line_track_update_position_pid(float position_error);

#endif
