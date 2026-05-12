#ifndef ENCODER_H
#define ENCODER_H

#include <stdint.h>
#include "config.h"

typedef struct {
    volatile int32_t pulse_count;
    float speed_rpm;
    float speed_m_per_s;
    float distance_m;
    uint32_t last_time_ms;
    int32_t last_pulse_count;
} encoder_t;

void encoder_left_init(void);
void encoder_right_init(void);
void encoder_left_isr(void);
void encoder_right_isr(void);
float encoder_left_get_speed(void);
float encoder_right_get_speed(void);
float encoder_left_get_distance(void);
float encoder_right_get_distance(void);
void encoder_update(uint32_t sys_time_ms);
void encoder_reset_distance(void);

extern encoder_t g_encoder_left;
extern encoder_t g_encoder_right;

#endif
