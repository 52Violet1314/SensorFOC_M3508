/*
 * File: FOC_Controller.h
 *
 * Code generated for Simulink model 'FOC_Controller'.
 *
 * Model version                  : 6.18
 * Simulink Coder version         : 9.5 (R2021a) 14-Nov-2020
 * C/C++ source code generated on : Thu Mar 26 20:18:50 2026
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: ARM Compatible->ARM Cortex
 * Code generation objectives:
 *    1. Execution efficiency
 *    2. RAM efficiency
 *    3. ROM efficiency
 * Validation result: Not run
 */

#ifndef RTW_HEADER_FOC_Controller_h_
#define RTW_HEADER_FOC_Controller_h_
#include <string.h>
#ifndef FOC_Controller_COMMON_INCLUDES_
#define FOC_Controller_COMMON_INCLUDES_
#include "rtwtypes.h"
#endif                                 /* FOC_Controller_COMMON_INCLUDES_ */

#include "FOC_Controller_types.h"

/* Child system includes */
#include "CurrentControl.h"
#include "ModeRegul.h"
#include "PositionControl.h"
#include "SpeedControl.h"

/* Macros for accessing real-time model data structure */
#ifndef rtmGetErrorStatus
#define rtmGetErrorStatus(rtm)         ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
#define rtmSetErrorStatus(rtm, val)    ((rtm)->errorStatus = (val))
#endif

/* Block signals (default storage) */
typedef struct {
  real32_T Switch_Inf;                 /* '<S34>/Switch_Inf' */
  uint16_T calibration_angle;          /* '<S2>/Chart' */
  boolean_T bControlEnable;            /* '<S2>/Chart' */
  B_lib_filter_T filter;               /* '<S4>/filter' */
} B_FOC_Controller_T;

/* Block states (default storage) for system '<Root>' */
typedef struct {
  real32_T Id_Integ_Buff_DSTATE;       /* '<S39>/Id_Integ_Buff' */
  real32_T Delay1_DSTATE;              /* '<S39>/Delay1' */
  real32_T Id_Integ_Buff_DSTATE_m;     /* '<S30>/Id_Integ_Buff' */
  real32_T Delay1_DSTATE_g;            /* '<S30>/Delay1' */
  real32_T Iq_Integ_Buff_DSTATE;       /* '<S22>/Iq_Integ_Buff' */
  real32_T Delay1_DSTATE_c;            /* '<S22>/Delay1' */
  real32_T Id_Integ_Buff_DSTATE_j;     /* '<S16>/Id_Integ_Buff' */
  real32_T Delay1_DSTATE_j;            /* '<S16>/Delay1' */
  uint16_T holdtime;                   /* '<S2>/Chart' */
  uint8_T Delay_DSTATE;                /* '<S2>/Delay' */
  boolean_T Delay6_DSTATE;             /* '<S38>/Delay6' */
  boolean_T Delay6_DSTATE_c;           /* '<S29>/Delay6' */
  boolean_T Delay6_DSTATE_i;           /* '<S21>/Delay6' */
  boolean_T Delay6_DSTATE_g;           /* '<S15>/Delay6' */
  uint8_T is_active_c5_FOC_Controller; /* '<S2>/Chart' */
  uint8_T is_c5_FOC_Controller;        /* '<S2>/Chart' */
  boolean_T SpeedControl_MODE;         /* '<Root>/SpeedControl' */
} DW_FOC_Controller_T;

/* Real-time Model Data Structure */
struct tag_RTM_FOC_Controller_T {
  const char_T *errorStatus;

  /*
   * Timing:
   * The following substructure contains information regarding
   * the timing information for the model.
   */
  struct {
    struct {
      uint16_T TID[3];
    } TaskCounters;
  } Timing;
};

/* Block signals (default storage) */
extern B_FOC_Controller_T FOC_Controller_B;

/* Block states (default storage) */
extern DW_FOC_Controller_T FOC_Controller_DW;

/*
 * Exported Global Signals
 *
 * Note: Exported global signals are block signals with an exported global
 * storage class designation.  Code generation will declare the memory for
 * these signals and export their symbols.
 *
 */
extern real32_T FOC_iPhaseAMeas_S;     /* '<Root>/FOC_iPhaseAMeas_S' */
extern real32_T FOC_iPhaseBMeas_S;     /* '<Root>/FOC_iPhaseBMeas_S' */
extern real32_T FOC_nSpeedMeasRaw_S;   /* '<Root>/FOC_nSpeedMeasRaw_S' */
extern uint16_T FOC_cntThetaElec_S;    /* '<Root>/FOC_cntThetaElec_S' */
extern real32_T FOC_iPhaseCMeas_S;     /* '<Root>/FOC_iPhaseCMeas_S' */
extern real32_T Position_degMeasured_S;/* '<Root>/Position_degMeasured_S' */
extern real32_T Motor_iTorque_S;       /* '<S35>/Switch_Inf' */
extern real32_T Speed_nSetPoint_S;     /* '<S4>/Switch2' */
extern real32_T Speed_nMeas_S;         /* '<S4>/filter' */
extern real32_T FOC_iIqSP_S;           /* '<S4>/Switch1' */
extern real32_T FOC_iIdSP_S;           /* '<S4>/Constant7' */
extern real32_T FOC_thetaElecMeas_S;   /* '<S1>/Switch3' */
extern real32_T FOC_sinCoefficient_S;  /* '<S1>/Sine_Cosine' */
extern real32_T FOC_cosCoefficient_S;  /* '<S1>/Sine_Cosine' */
extern real32_T FOC_iIdMeas_S;         /* '<S9>/Add1' */
extern real32_T FOC_iIqMeas_S;         /* '<S9>/Add2' */
extern real32_T FOC_uVqSP_S;           /* '<S13>/Switch2' */
extern real32_T FOC_uVdSP_S;           /* '<S14>/Switch2' */
extern real32_T FOC_uPhaseUVolt_S;     /* '<S8>/Add1' */
extern real32_T FOC_uPhaseVVolt_S;     /* '<S7>/Add' */
extern real32_T FOC_uPhaseWVolt_S;     /* '<S7>/Add1' */
extern real32_T SVM_uPhaseU_S;         /* '<S1>/Signal Conversion' */
extern real32_T SVM_uPhaseV_S;         /* '<S1>/Signal Conversion1' */
extern real32_T SVM_uPhaseW_S;         /* '<S1>/Signal Conversion2' */
extern boolean_T FOC_bInitPID_S;       /* '<S2>/AND' */
extern boolean_T Motor_flagFindPos_S;  /* '<S2>/Chart' */

/*
 * Exported States
 *
 * Note: Exported states are block states with an exported global
 * storage class designation.  Code generation will declare the memory for these
 * states and exports their symbols.
 *
 */
extern uint16_T FOC_cntThetaElecOffset_S;/* '<Root>/Data Store Memory' */

/* Model entry point functions */
extern void FOC_Controller_initialize(void);
extern void FOC_Controller_step(void);
extern void FOC_Controller_terminate(void);

/* Exported data declaration */

/* Const memory section */
/* Declaration for custom storage class: Const */
extern const boolean_T FOC_bManualVdVq_C;/* Referenced by: '<S6>/Constant2' */
extern const real32_T FOC_iIdSP_C;     /* Referenced by: '<S4>/Constant7' */
extern const real32_T FOC_kiPID_C;     /* Referenced by: '<S6>/Constant1' */
extern const real32_T FOC_kpPID_C;     /* Referenced by: '<S6>/Constant5' */
extern const real32_T FOC_uBusVoltage_C;
                       /* Referenced by: '<S6>/paramCurrentControlSatVoltage' */
extern const real32_T FOC_uFindPosVd_C;/* Referenced by: '<S6>/Constant8' */
extern const real32_T FOC_uManualVd_C; /* Referenced by: '<S6>/Constant3' */
extern const real32_T FOC_uManualVq_C; /* Referenced by: '<S6>/Constant4' */
extern const boolean_T Motor_bFindPosEnable_C;/* Referenced by: '<S2>/Constant1' */
extern const real32_T Motor_facTrq2Current_C;/* Referenced by: '<S4>/Constant9' */
extern const uint8_T Motor_swtMode_C;  /* Referenced by:
                                        * '<S2>/Constant17'
                                        * '<S4>/Constant1'
                                        * '<S4>/Constant17'
                                        * '<S4>/Constant5'
                                        * '<S6>/Constant17'
                                        */
extern const uint16_T Motor_tiFindPos_C;/* Referenced by: '<S2>/Constant2' */
extern const real32_T Motor_trqTorqueSetPoint_C;/* Referenced by: '<S4>/Constant10' */
extern const real32_T Position_degSetPoint_C;/* Referenced by: '<S3>/Constant8' */
extern const real32_T Position_kiPID_C;/* Referenced by: '<S3>/ki_0.12' */
extern const real32_T Position_kpPID_C;/* Referenced by: '<S3>/kp_0.0015' */
extern const real32_T Position_nPIDMax_C;
                    /* Referenced by: '<S3>/paramPositionControlSatCurrent_4' */
extern const real32_T Speed_iPIDMax_C;
                    /* Referenced by: '<S4>/paramVelocityControlSatCurrent_4' */
extern const real32_T Speed_kiPID_C;   /* Referenced by: '<S4>/ki_0.12' */
extern const real32_T Speed_kpPID_C;   /* Referenced by: '<S4>/kp_0.0015' */
extern const real32_T Speed_nSetPoint_C;/* Referenced by: '<S4>/Constant8' */
extern const real32_T Speed_tiFilter_C;/* Referenced by: '<S4>/Constant12' */

/* Real-time Model object */
extern RT_MODEL_FOC_Controller_T *const FOC_Controller_M;

/*-
 * The generated code includes comments that allow you to trace directly
 * back to the appropriate location in the model.  The basic format
 * is <system>/block_name, where system is the system number (uniquely
 * assigned by Simulink) and block_name is the name of the block.
 *
 * Use the MATLAB hilite_system command to trace the generated code back
 * to the model.  For example,
 *
 * hilite_system('<S3>')    - opens system 3
 * hilite_system('<S3>/Kp') - opens and selects block Kp which resides in S3
 *
 * Here is the system hierarchy for this model
 *
 * '<Root>' : 'FOC_Controller'
 * '<S1>'   : 'FOC_Controller/CurrentControl'
 * '<S2>'   : 'FOC_Controller/ModeRegul'
 * '<S3>'   : 'FOC_Controller/PositionControl'
 * '<S4>'   : 'FOC_Controller/SpeedControl'
 * '<S5>'   : 'FOC_Controller/CurrentControl/Clarke Transform'
 * '<S6>'   : 'FOC_Controller/CurrentControl/DQ_Current_Control'
 * '<S7>'   : 'FOC_Controller/CurrentControl/Inverse_Clarke_Transform'
 * '<S8>'   : 'FOC_Controller/CurrentControl/Inverse_Park_Transform'
 * '<S9>'   : 'FOC_Controller/CurrentControl/Park_Transform'
 * '<S10>'  : 'FOC_Controller/CurrentControl/Space_Vector_Modulation'
 * '<S11>'  : 'FOC_Controller/CurrentControl/DQ_Current_Control/D_Current_Control'
 * '<S12>'  : 'FOC_Controller/CurrentControl/DQ_Current_Control/Q_Current_Control'
 * '<S13>'  : 'FOC_Controller/CurrentControl/DQ_Current_Control/Saturation Dynamic1'
 * '<S14>'  : 'FOC_Controller/CurrentControl/DQ_Current_Control/Saturation Dynamic2'
 * '<S15>'  : 'FOC_Controller/CurrentControl/DQ_Current_Control/D_Current_Control/PID'
 * '<S16>'  : 'FOC_Controller/CurrentControl/DQ_Current_Control/D_Current_Control/PID/Regul'
 * '<S17>'  : 'FOC_Controller/CurrentControl/DQ_Current_Control/D_Current_Control/PID/Saturation'
 * '<S18>'  : 'FOC_Controller/CurrentControl/DQ_Current_Control/D_Current_Control/PID/Saturation/CompZero1'
 * '<S19>'  : 'FOC_Controller/CurrentControl/DQ_Current_Control/D_Current_Control/PID/Saturation/CompZero2'
 * '<S20>'  : 'FOC_Controller/CurrentControl/DQ_Current_Control/D_Current_Control/PID/Saturation/Sat'
 * '<S21>'  : 'FOC_Controller/CurrentControl/DQ_Current_Control/Q_Current_Control/PID'
 * '<S22>'  : 'FOC_Controller/CurrentControl/DQ_Current_Control/Q_Current_Control/PID/Regul'
 * '<S23>'  : 'FOC_Controller/CurrentControl/DQ_Current_Control/Q_Current_Control/PID/Saturation'
 * '<S24>'  : 'FOC_Controller/CurrentControl/DQ_Current_Control/Q_Current_Control/PID/Saturation/CompZero1'
 * '<S25>'  : 'FOC_Controller/CurrentControl/DQ_Current_Control/Q_Current_Control/PID/Saturation/CompZero2'
 * '<S26>'  : 'FOC_Controller/CurrentControl/DQ_Current_Control/Q_Current_Control/PID/Saturation/Sat'
 * '<S27>'  : 'FOC_Controller/ModeRegul/Chart'
 * '<S28>'  : 'FOC_Controller/PositionControl/Position_Control'
 * '<S29>'  : 'FOC_Controller/PositionControl/Position_Control/PID'
 * '<S30>'  : 'FOC_Controller/PositionControl/Position_Control/PID/Regul'
 * '<S31>'  : 'FOC_Controller/PositionControl/Position_Control/PID/Saturation'
 * '<S32>'  : 'FOC_Controller/PositionControl/Position_Control/PID/Saturation/CompZero1'
 * '<S33>'  : 'FOC_Controller/PositionControl/Position_Control/PID/Saturation/CompZero2'
 * '<S34>'  : 'FOC_Controller/PositionControl/Position_Control/PID/Saturation/Sat'
 * '<S35>'  : 'FOC_Controller/SpeedControl/Sat'
 * '<S36>'  : 'FOC_Controller/SpeedControl/Speed_Control'
 * '<S37>'  : 'FOC_Controller/SpeedControl/filter'
 * '<S38>'  : 'FOC_Controller/SpeedControl/Speed_Control/PID'
 * '<S39>'  : 'FOC_Controller/SpeedControl/Speed_Control/PID/Regul'
 * '<S40>'  : 'FOC_Controller/SpeedControl/Speed_Control/PID/Saturation'
 * '<S41>'  : 'FOC_Controller/SpeedControl/Speed_Control/PID/Saturation/CompZero1'
 * '<S42>'  : 'FOC_Controller/SpeedControl/Speed_Control/PID/Saturation/CompZero2'
 * '<S43>'  : 'FOC_Controller/SpeedControl/Speed_Control/PID/Saturation/Sat'
 */
#endif                                 /* RTW_HEADER_FOC_Controller_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
