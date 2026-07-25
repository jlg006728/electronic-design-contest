#ifndef SIMPLE_LINE_PROFILE_H
#define SIMPLE_LINE_PROFILE_H

#include "simple_line_control.h"

#define SIMPLE_LINE_PROFILE_LEFT_BASE             380
#define SIMPLE_LINE_PROFILE_RIGHT_BASE            383
#define SIMPLE_LINE_PROFILE_PWM_MIN               250
#define SIMPLE_LINE_PROFILE_PWM_MAX               657
#define SIMPLE_LINE_PROFILE_KP_DIVISOR            26
#define SIMPLE_LINE_PROFILE_CORRECTION_LIMIT      110
#define SIMPLE_LINE_PROFILE_LOST_SEARCH_CORRECTION 70
#define SIMPLE_LINE_PROFILE_LOST_TIMEOUT_FRAMES   200U

#define SIMPLE_LINE_PROFILE_CONFIG_INITIALIZER                  \
    {                                                           \
        .left_base = SIMPLE_LINE_PROFILE_LEFT_BASE,             \
        .right_base = SIMPLE_LINE_PROFILE_RIGHT_BASE,           \
        .pwm_min = SIMPLE_LINE_PROFILE_PWM_MIN,                 \
        .pwm_max = SIMPLE_LINE_PROFILE_PWM_MAX,                 \
        .kp_divisor = SIMPLE_LINE_PROFILE_KP_DIVISOR,           \
        .correction_limit = SIMPLE_LINE_PROFILE_CORRECTION_LIMIT, \
        .lost_search_correction =                               \
            SIMPLE_LINE_PROFILE_LOST_SEARCH_CORRECTION,         \
        .lost_timeout_frames =                                  \
            SIMPLE_LINE_PROFILE_LOST_TIMEOUT_FRAMES,            \
    }

#endif
