#ifndef __CONTROL_H__
#define __CONTROL_H__

#include <stdint.h>
#include "PID.h"

typedef enum
{
    CONTROL_MODE_SPEED = 0u,
    CONTROL_MODE_CURRENT = 1u,
    CONTROL_MODE_POSITION = 2u
} Control_Mode_t;

typedef struct
{
    float kp;
    float ki;
    float dt;
    float output_limit;
    float integral_limit;
} Control_PID_Config_t;

void Control_Init(void);
void Control_SetCurrentPID(float kp, float ki, float dt,
                           float output_limit, float integral_limit);
void Control_SetSpeedPID(float kp, float ki, float dt,
                         float iq_limit, float integral_limit);
void Control_SetPositionPID(float kp, float ki, float dt,
                            float speed_limit, float integral_limit);
void Control_SetSpeedLoop(uint16_t loop_ticks);
uint16_t Control_GetSpeedLoop(void);
float Control_GetCurrentDt(void);
const Control_PID_Config_t *Control_GetCurrentPID(void);
const Control_PID_Config_t *Control_GetSpeedPID(void);
const Control_PID_Config_t *Control_GetPositionPID(void);
void Control_ResetCurrentPID(void);
void Control_ResetSpeedPID(void);
void Control_ResetPositionPID(void);
float Control_CurrentDStep(float error);
float Control_CurrentQStep(float error);
float Control_SpeedStep(float error);
float Control_PositionStep(float actual_position_rad);
void Control_SetSpeedTarget(float speed_ref_rpm);
void Control_SetCurrentTarget(float id_ref_a, float iq_ref_a);
void Control_SetPositionTarget(float position_rad);
Control_Mode_t Control_GetMode(void);
float Control_GetSpeedTarget(void);
float Control_GetPositionTarget(void);
float Control_GetPositionError(float actual_position_rad);
void Control_GetCurrentTarget(float *id_ref_a, float *iq_ref_a);

#endif
