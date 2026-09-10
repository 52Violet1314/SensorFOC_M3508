#include "VF.h"

#include "SVPWM.h"
#include "main.h"
#include "tim.h"

#include <math.h>

#define VF_PI       3.14159265359f
#define VF_TWO_PI   6.28318530718f

static float vf_speed_rpm;
static float vf_target_rpm;
static float vf_dt = VF_CONTROL_DT;
static uint8_t vf_calib;
static uint32_t vf_calib_t0;
static uint32_t vf_calib_n;
static float vf_theta_prev;
static uint8_t vf_theta_valid;
static float vf_spd_accum;
static uint16_t vf_spd_cnt;
static uint32_t vf_cmd_active_t0;
static volatile uint8_t vf_cal_active;
static volatile uint8_t vf_cal_done;
static uint32_t vf_cal_t0;
static float vf_cal_sin_sum;
static float vf_cal_cos_sum;
static uint32_t vf_cal_count;
volatile float vf_encoder_offset;

volatile float vf_speed_meas_rpm;
volatile uint8_t vf_running;
volatile uint8_t vf_stall;
volatile float vf_sim_elec_angle;

static float wrap_angle(float angle)
{
    angle = fmodf(angle, VF_TWO_PI);
    if (angle < 0.0f) angle += VF_TWO_PI;
    return angle;
}

void VF_SetTargetRPM(float rpm)
{
    if (rpm > VF_MAX_RPM) rpm = VF_MAX_RPM;
    if (rpm < -VF_MAX_RPM) rpm = -VF_MAX_RPM;
    vf_target_rpm = rpm;
}

float VF_GetTargetRPM(void)
{
    return vf_target_rpm;
}

void VF_Init(void)
{
    vf_speed_rpm = 0.0f;
    vf_target_rpm = 0.0f;
    vf_dt = VF_CONTROL_DT;
    vf_calib = 1u;
    vf_calib_t0 = 0u;
    vf_calib_n = 0u;
    vf_theta_prev = 0.0f;
    vf_theta_valid = 0u;
    vf_spd_accum = 0.0f;
    vf_spd_cnt = 0u;
    vf_cmd_active_t0 = 0u;
    vf_speed_meas_rpm = 0.0f;
    vf_running = 0u;
    vf_stall = 0u;
    vf_sim_elec_angle = 0.0f;
    vf_cal_active = 0u;
    vf_cal_done = (VF_ENABLE_STARTUP_CALIBRATION == 0u) ? 1u : 0u;
    vf_cal_t0 = 0u;
    vf_cal_sin_sum = 0.0f;
    vf_cal_cos_sum = 0.0f;
    vf_cal_count = 0u;
    vf_encoder_offset = VF_ENCODER_OFFSET_RAD;
}

void VF_CalibrationStart(void)
{
    vf_cal_active = 1u;
    vf_cal_done = 0u;
    vf_cal_t0 = HAL_GetTick();
    vf_cal_sin_sum = 0.0f;
    vf_cal_cos_sum = 0.0f;
    vf_cal_count = 0u;
    vf_target_rpm = 0.0f;
    vf_speed_rpm = 0.0f;
}

void VF_CalibrationCancel(void)
{
    vf_cal_active = 0u;
    vf_cal_done = 1u;
    vf_speed_rpm = 0.0f;
    vf_sim_elec_angle = 0.0f;
}

uint8_t VF_CalibrationIsDone(void) { return vf_cal_done; }
uint8_t VF_CalibrationIsActive(void) { return vf_cal_active; }
float VF_GetEncoderOffset(void) { return vf_encoder_offset; }

void VF_CalibrationSample(float theta_elec)
{
    uint32_t elapsed;
    if (vf_cal_active == 0u) return;
    elapsed = HAL_GetTick() - vf_cal_t0;
    if (elapsed >= VF_CALIBRATION_SETTLE_MS)
    {
        vf_cal_sin_sum += sinf(theta_elec);
        vf_cal_cos_sum += cosf(theta_elec);
        vf_cal_count++;
    }
    if (elapsed >= VF_CALIBRATION_HOLD_MS)
    {
        if (vf_cal_count >= 10u)
        {
            vf_encoder_offset = -atan2f(vf_cal_sin_sum,
                                         vf_cal_cos_sum);
            vf_cal_done = 1u;
        }
        vf_cal_active = 0u;
    }
}

void VF_Tick(float vdc, float theta_elec)
{
    float diff;
    float step;
    float fe;
    float voltage;
    float theta;
    uint16_t ccr1;
    uint16_t ccr2;
    uint16_t ccr3;

    if (vf_cal_active != 0u)
    {
        /* Hold a stationary alpha-axis vector while the control interrupt
         * samples the absolute encoder. No speed command is accepted here. */
        vf_sim_elec_angle = 0.0f;
        SVPWM(VF_CALIBRATION_V, 0.0f, vdc, &ccr1, &ccr2, &ccr3);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, ccr1);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, ccr2);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, ccr3);
        return;
    }

    /* Measure the actual callback period during startup. */
    if (vf_calib != 0u)
    {
        if (vf_calib_n == 0u) vf_calib_t0 = HAL_GetTick();
        vf_calib_n++;
        if ((HAL_GetTick() - vf_calib_t0) >= 200u)
        {
            vf_dt = 0.001f * 200.0f / (float)vf_calib_n;
            if (vf_dt < 0.000001f || vf_dt > 0.001f) vf_dt = VF_CONTROL_DT;
            vf_calib = 0u;
        }
    }

    diff = vf_target_rpm - vf_speed_rpm;
    step = VF_ACC_RPM_PER_S * vf_dt;
    if (diff > step) vf_speed_rpm += step;
    else if (diff < -step) vf_speed_rpm -= step;
    else vf_speed_rpm = vf_target_rpm;

    if (vf_theta_valid != 0u)
    {
        float delta = theta_elec - vf_theta_prev;
        if (delta > VF_PI) delta -= VF_TWO_PI;
        if (delta < -VF_PI) delta += VF_TWO_PI;
        vf_spd_accum += delta;
        if (++vf_spd_cnt >= VF_SPD_WIN_TICKS)
        {
            float rpm = vf_spd_accum * 60.0f * 0.15915494309f /
                        (vf_dt * (float)VF_SPD_WIN_TICKS *
                         (float)VF_POLE_PAIRS);
            vf_speed_meas_rpm += VF_SPD_LPF_ALPHA *
                                 (rpm - vf_speed_meas_rpm);
            vf_spd_accum = 0.0f;
            vf_spd_cnt = 0u;
        }
    }
    else
    {
        vf_theta_valid = 1u;
    }
    vf_theta_prev = wrap_angle(theta_elec);

    if (fabsf(vf_speed_rpm) < VF_STOP_RPM)
    {
        vf_running = 0u;
        vf_stall = 0u;
        vf_cmd_active_t0 = 0u;
        vf_speed_meas_rpm *= 0.95f;
        SVPWM(0.0f, 0.0f, vdc, &ccr1, &ccr2, &ccr3);
    }
    else
    {
        if (fabsf(vf_speed_meas_rpm) > VF_START_DETECT_RPM)
        {
            vf_running = 1u;
            vf_stall = 0u;
            vf_cmd_active_t0 = 0u;
        }
        else if (vf_stall == 0u)
        {
            if (vf_cmd_active_t0 == 0u) vf_cmd_active_t0 = HAL_GetTick();
            if ((HAL_GetTick() - vf_cmd_active_t0) >= VF_STALL_MS)
                vf_stall = 1u;
        }

        fe = vf_speed_rpm * (float)VF_POLE_PAIRS / 60.0f;
        /* Open-loop field angle. The encoder angle is used for monitoring and
         * speed estimation only; it must not drive the rotating voltage vector. */
        vf_sim_elec_angle = wrap_angle(vf_sim_elec_angle +
                                       VF_TWO_PI * fe * vf_dt);
        voltage = VF_V_BOOST + VF_V_SLOPE * fabsf(fe);
        if (vf_running == 0u) voltage += VF_START_EXTRA_V;
        if (voltage > VF_V_MAX) voltage = VF_V_MAX;
        if (vf_stall != 0u) voltage = VF_V_BOOST;
        theta = vf_sim_elec_angle + VF_TORQUE_ANGLE_RAD;
        SVPWM(voltage * cosf(theta), voltage * sinf(theta), vdc,
              &ccr1, &ccr2, &ccr3);
    }

    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, ccr1);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, ccr2);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, ccr3);
}
