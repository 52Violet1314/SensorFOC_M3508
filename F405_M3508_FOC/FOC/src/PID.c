#include "PID.h"
#include <stdint.h>

static float PID_Clamp(float value, float min_value, float max_value)
{
    if (value > max_value) return max_value;
    if (value < min_value) return min_value;
    return value;
}

void PID_Init(PID_Controller_t *pid,
              float kp, float ki, float dt,
              float out_min, float out_max,
              float int_min, float int_max)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->dt = dt;
    pid->out_min = out_min;
    pid->out_max = out_max;
    pid->int_min = int_min;
    pid->int_max = int_max;
    pid->integrator = 0.0f;
}

void PID_Reset(PID_Controller_t *pid)
{
    pid->integrator = 0.0f;
}

float PID_Step(PID_Controller_t *pid, float error)
{
    float unsaturated;
    float output;
    uint8_t integrate;

    pid->integrator = PID_Clamp(pid->integrator, pid->int_min, pid->int_max);
    unsaturated = pid->kp * error + pid->integrator;
    output = PID_Clamp(unsaturated, pid->out_min, pid->out_max);

    integrate = (output > pid->out_min && output < pid->out_max) ||
                (output >= pid->out_max && error < 0.0f) ||
                (output <= pid->out_min && error > 0.0f);
    if (integrate)
    {
        pid->integrator += pid->ki * error * pid->dt;
        pid->integrator = PID_Clamp(pid->integrator, pid->int_min, pid->int_max);
    }

    output = pid->kp * error + pid->integrator;
    return PID_Clamp(output, pid->out_min, pid->out_max);
}
