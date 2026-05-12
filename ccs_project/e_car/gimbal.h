#ifndef GIMBAL_H
#define GIMBAL_H

#include <stdint.h>
#include <stdbool.h>
#include "config.h"
#include "openmv.h"

void gimbal_init(void);
void gimbal_set_angle_x(float angle_deg);
void gimbal_set_angle_y(float angle_deg);
void gimbal_aiming_update(const openmv_data_t *target);
void gimbal_circle_update(float car_progress_rad);
void gimbal_enable_laser(bool enable);
void gimbal_reset(void);

#endif
