#include "Control.h"

static PID_Controller_t pid_id;
static PID_Controller_t pid_iq;
static PID_Controller_t pid_spd;
static PID_Controller_t pid_pos;
static Control_PID_Config_t current_pid_config;
static Control_PID_Config_t speed_pid_config;
static Control_PID_Config_t position_pid_config;
static uint16_t speed_loop_ticks = 1u;
/* CAN current command range is -30000..30000 -> -30..30 A. */
static const float control_current_limit_a = 30.0f;
/* CAN speed command range is -20000..20000 rpm. */
static const float control_speed_limit_rpm = 20000.0f;
static volatile float control_speed_ref_rpm = 0.0f;
static volatile float control_position_ref_rad = 0.0f;
static volatile float control_id_ref_a = 0.0f;
static volatile float control_iq_ref_a = 0.0f;
static volatile Control_Mode_t control_mode = CONTROL_MODE_SPEED;
static volatile uint32_t current_target_seq = 0u;

static float Control_Clamp(float value, float min_value, float max_value)
{
    if (value > max_value) return max_value;
    if (value < min_value) return min_value;
    return value;
}

void Control_Init(void)
{
    control_speed_ref_rpm = 0.0f;
    control_position_ref_rad = 0.0f;
    control_id_ref_a = 0.0f;
    control_iq_ref_a = 0.0f;
    control_mode = CONTROL_MODE_SPEED;
    current_target_seq = 0u;
}

void Control_SetCurrentPID(float kp, float ki, float dt,
                           float output_limit, float integral_limit)
{
    current_pid_config.kp = kp;
    current_pid_config.ki = ki;
    current_pid_config.dt = dt;
    current_pid_config.output_limit = Control_Clamp(output_limit, 0.0f, 1000.0f);
    current_pid_config.integral_limit = Control_Clamp(integral_limit, 0.0f,
                                                      current_pid_config.output_limit);
    PID_Init(&pid_id, kp, ki, dt,
             -current_pid_config.output_limit, current_pid_config.output_limit,
             -current_pid_config.integral_limit, current_pid_config.integral_limit);
    PID_Init(&pid_iq, kp, ki, dt,
             -current_pid_config.output_limit, current_pid_config.output_limit,
             -current_pid_config.integral_limit, current_pid_config.integral_limit);
}

void Control_SetSpeedPID(float kp, float ki, float dt,
                         float iq_limit, float integral_limit)
{
    speed_pid_config.kp = kp;
    speed_pid_config.ki = ki;
    speed_pid_config.dt = dt;
    speed_pid_config.output_limit = Control_Clamp(iq_limit, 0.0f,
                                                  control_current_limit_a);
    speed_pid_config.integral_limit = Control_Clamp(integral_limit, 0.0f,
                                                    speed_pid_config.output_limit);
    PID_Init(&pid_spd, kp, ki, dt,
             -speed_pid_config.output_limit, speed_pid_config.output_limit,
             -speed_pid_config.integral_limit, speed_pid_config.integral_limit);
}

void Control_SetPositionPID(float kp, float ki, float dt,
                            float speed_limit, float integral_limit)
{
    position_pid_config.kp = kp;
    position_pid_config.ki = ki;
    position_pid_config.dt = dt;
    position_pid_config.output_limit = Control_Clamp(speed_limit, 0.0f,
                                                     control_speed_limit_rpm);
    position_pid_config.integral_limit = Control_Clamp(integral_limit, 0.0f,
                                                       position_pid_config.output_limit);
    PID_Init(&pid_pos, kp, ki, dt,
             -position_pid_config.output_limit, position_pid_config.output_limit,
             -position_pid_config.integral_limit, position_pid_config.integral_limit);
}

void Control_SetSpeedLoop(uint16_t loop_ticks)
{
    speed_loop_ticks = (loop_ticks == 0u) ? 1u : loop_ticks;
}

uint16_t Control_GetSpeedLoop(void)
{
    return speed_loop_ticks;
}

float Control_GetCurrentDt(void)
{
    return current_pid_config.dt;
}

const Control_PID_Config_t *Control_GetCurrentPID(void)
{
    return &current_pid_config;
}

const Control_PID_Config_t *Control_GetSpeedPID(void)
{
    return &speed_pid_config;
}

const Control_PID_Config_t *Control_GetPositionPID(void)
{
    return &position_pid_config;
}

void Control_ResetCurrentPID(void)
{
    PID_Reset(&pid_id);
    PID_Reset(&pid_iq);
}

void Control_ResetSpeedPID(void)
{
    PID_Reset(&pid_spd);
}

void Control_ResetPositionPID(void)
{
    PID_Reset(&pid_pos);
}

float Control_CurrentDStep(float error)
{
    return PID_Step(&pid_id, error);
}

float Control_CurrentQStep(float error)
{
    return PID_Step(&pid_iq, error);
}

float Control_SpeedStep(float error)
{
    return PID_Step(&pid_spd, error);
}

float Control_GetPositionError(float actual_position_rad)
{
    float error = control_position_ref_rad - actual_position_rad;
    const float pi = 3.14159265359f;
    const float two_pi = 6.28318530718f;

    /* Position commands are single-turn angles. Always choose the shortest
       path when the target/actual angle crosses the +/-pi boundary. */
    while (error > pi) error -= two_pi;
    while (error < -pi) error += two_pi;
    return error;
}

float Control_PositionStep(float actual_position_rad)
{
    return PID_Step(&pid_pos, Control_GetPositionError(actual_position_rad));
}

void Control_SetSpeedTarget(float speed_ref_rpm)
{
    control_speed_ref_rpm = Control_Clamp(speed_ref_rpm,
                                           -control_speed_limit_rpm,
                                           control_speed_limit_rpm);
    control_mode = CONTROL_MODE_SPEED;
}

void Control_SetCurrentTarget(float id_ref_a, float iq_ref_a)
{
    current_target_seq++;
    control_id_ref_a = Control_Clamp(id_ref_a,
                                     -control_current_limit_a,
                                     control_current_limit_a);
    control_iq_ref_a = Control_Clamp(iq_ref_a,
                                     -control_current_limit_a,
                                     control_current_limit_a);
    current_target_seq++;
    control_mode = CONTROL_MODE_CURRENT;
}

void Control_SetPositionTarget(float position_rad)
{
    control_position_ref_rad = position_rad;
    control_mode = CONTROL_MODE_POSITION;
}

Control_Mode_t Control_GetMode(void)
{
    return control_mode;
}

float Control_GetSpeedTarget(void)
{
    return control_speed_ref_rpm;
}

float Control_GetPositionTarget(void)
{
    return control_position_ref_rad;
}

void Control_GetCurrentTarget(float *id_ref_a, float *iq_ref_a)
{
    uint32_t seq_start;
    uint32_t seq_end;

    do
    {
        seq_start = current_target_seq;
        *id_ref_a = control_id_ref_a;
        *iq_ref_a = control_iq_ref_a;
        seq_end = current_target_seq;
    } while ((seq_start != seq_end) || ((seq_start & 1u) != 0u));
}
