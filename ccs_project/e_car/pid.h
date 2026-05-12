#ifndef PID_H
#define PID_H

typedef struct {
    float kp, ki, kd;
    float integral;
    float prev_error;
    float prev_measurement;
    float integral_limit;
    float output_limit;
    float deadband;
    float setpoint;
} pid_controller_t;

void pid_init(pid_controller_t *pc, float kp, float ki, float kd,
              float i_limit, float o_limit, float deadband);

float pid_update_positional(pid_controller_t *pc, float measurement);

void pid_update_incremental(pid_controller_t *pc, float measurement,
                             float *out);

void pid_reset(pid_controller_t *pc);

void pid_set_setpoint(pid_controller_t *pc, float setpoint);

#endif
