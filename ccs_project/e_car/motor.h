#ifndef MOTOR_H
#define MOTOR_H

#include <stdint.h>
#include <stdbool.h>
#include "config.h"

void motor_init(void);
void motor_drive(float left_duty, float right_duty);
void motor_stop(void);
void motor_brake(void);
void motor_set_target(float left_mps, float right_mps);
void motor_update_speed_pid(uint32_t sys_time_ms);

#endif
