#include "pid.h"

void pid_init(pid_controller_t *pc, float kp, float ki, float kd,
              float i_limit, float o_limit, float deadband)
{
    pc->kp = kp;
    pc->ki = ki;
    pc->kd = kd;
    pc->integral = 0.0f;
    pc->prev_error = 0.0f;
    pc->prev_measurement = 0.0f;
    pc->integral_limit = i_limit;
    pc->output_limit = o_limit;
    pc->deadband = deadband;
    pc->setpoint = 0.0f;
}

void pid_set_setpoint(pid_controller_t *pc, float setpoint)
{
    pc->setpoint = setpoint;
}

void pid_reset(pid_controller_t *pc)
{
    pc->integral = 0.0f;
    pc->prev_error = 0.0f;
    pc->prev_measurement = 0.0f;
}

float pid_update_positional(pid_controller_t *pc, float measurement)
{
    float error = pc->setpoint - measurement;

    if (error < pc->deadband && error > -pc->deadband) {
        error = 0.0f;
    }

    float ki_effective = pc->ki;
    if (error > pc->integral_limit * 0.5f || error < -pc->integral_limit * 0.5f) {
        ki_effective = 0.0f;
    }

    pc->integral += error;
    if (pc->integral > pc->integral_limit) {
        pc->integral = pc->integral_limit;
    } else if (pc->integral < -pc->integral_limit) {
        pc->integral = -pc->integral_limit;
    }

    float derivative = -(measurement - pc->prev_measurement);

    float output = pc->kp * error + ki_effective * pc->integral + pc->kd * derivative;

    if (output > pc->output_limit) {
        output = pc->output_limit;
    } else if (output < -pc->output_limit) {
        output = -pc->output_limit;
    }

    pc->prev_error = error;
    pc->prev_measurement = measurement;

    return output;
}

void pid_update_incremental(pid_controller_t *pc, float measurement,
                             float *out)
{
    float error = pc->setpoint - measurement;

    if (error < pc->deadband && error > -pc->deadband) {
        error = 0.0f;
    }

    float p_term = pc->kp * (error - pc->prev_error);
    float i_term = pc->ki * error;

    pc->integral += i_term;
    if (pc->integral > pc->integral_limit) {
        pc->integral = pc->integral_limit;
    } else if (pc->integral < -pc->integral_limit) {
        pc->integral = -pc->integral_limit;
    }

    float d_term = pc->kd * (error - 2.0f * pc->prev_error + pc->prev_measurement);

    float delta = p_term + i_term + d_term;

    *out += delta;
    if (*out > pc->output_limit) {
        *out = pc->output_limit;
    } else if (*out < -pc->output_limit) {
        *out = -pc->output_limit;
    }

    pc->prev_measurement = pc->prev_error;
    pc->prev_error = error;
}
