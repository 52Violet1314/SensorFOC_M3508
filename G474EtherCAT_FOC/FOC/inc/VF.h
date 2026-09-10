#ifndef FOC_VF_H
#define FOC_VF_H

#include <stdint.h>
#include "FOC_Config.h"

/* Motor and V/F parameters, matching the M3508 reference setup. */
#define VF_POLE_PAIRS          FOC_MOTOR_POLE_PAIRS
#define VF_CONTROL_DT          0.000025f /* 40.5 kHz TIM1 center-aligned PWM */
#define VF_DT                  VF_CONTROL_DT
#define VF_V_BOOST             1.5f
#define VF_START_EXTRA_V       2.0f
#define VF_V_SLOPE             0.12f
#define VF_V_MAX               6.0f
#define VF_ACC_RPM_PER_S       30.0f
#define VF_MAX_RPM             100.0f
#define VF_TORQUE_ANGLE_RAD    1.57079632679f
#define VF_SPD_WIN_TICKS       50u
#define VF_SPD_LPF_ALPHA       0.2f
#define VF_START_DETECT_RPM    10.0f
#define VF_STALL_MS            1500u
#define VF_STOP_RPM            0.05f

/* MT6701 is a mechanical-angle sensor; this offset is the electrical
 * alignment correction and can be tuned without changing the control code. */
#define VF_ENCODER_OFFSET_RAD  FOC_ENCODER_OFFSET_RAD

/* Set to 1 to run rotor alignment once after power-up. The rotor is held at
 * electrical angle 0 with VF_CALIBRATION_V for VF_CALIBRATION_HOLD_MS. */
#define VF_ENABLE_STARTUP_CALIBRATION 0u
#define VF_CALIBRATION_V              2.0f
#define VF_CALIBRATION_HOLD_MS        1000u
#define VF_CALIBRATION_SETTLE_MS      300u

extern volatile float vf_speed_meas_rpm;
extern volatile uint8_t vf_running;
extern volatile uint8_t vf_stall;
extern volatile float vf_encoder_offset;
extern volatile float vf_sim_elec_angle;

void VF_Init(void);
void VF_CalibrationStart(void);
void VF_CalibrationCancel(void);
uint8_t VF_CalibrationIsDone(void);
uint8_t VF_CalibrationIsActive(void);
void VF_CalibrationSample(float theta_elec);
float VF_GetEncoderOffset(void);
void VF_SetTargetRPM(float rpm);
float VF_GetTargetRPM(void);
void VF_Tick(float vdc, float theta_elec);

#endif
