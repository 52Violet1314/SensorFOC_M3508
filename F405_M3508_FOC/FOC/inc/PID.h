#ifndef __PID_H__
#define __PID_H__

typedef struct
{
    float kp;
    float ki;
    float dt;
    float out_min;
    float out_max;
    float int_min;
    float int_max;
    float integrator;
} PID_Controller_t;

void PID_Init(PID_Controller_t *pid,
              float kp, float ki, float dt,
              float out_min, float out_max,
              float int_min, float int_max);
void PID_Reset(PID_Controller_t *pid);
float PID_Step(PID_Controller_t *pid, float error);

#endif
