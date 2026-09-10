/*
 * File: FOC_Controller.c
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

#include "FOC_Controller.h"
#include "FOC_Controller_private.h"

/* Exported block signals */
real32_T FOC_iPhaseAMeas_S;            /* '<Root>/FOC_iPhaseAMeas_S' */
real32_T FOC_iPhaseBMeas_S;            /* '<Root>/FOC_iPhaseBMeas_S' */
real32_T FOC_nSpeedMeasRaw_S;          /* '<Root>/FOC_nSpeedMeasRaw_S' */
uint16_T FOC_cntThetaElec_S;           /* '<Root>/FOC_cntThetaElec_S' */
real32_T FOC_iPhaseCMeas_S;            /* '<Root>/FOC_iPhaseCMeas_S' */
real32_T Position_degMeasured_S;       /* '<Root>/Position_degMeasured_S' */
real32_T Motor_iTorque_S;              /* '<S35>/Switch_Inf' */
real32_T Speed_nSetPoint_S;            /* '<S4>/Switch2' */
real32_T Speed_nMeas_S;                /* '<S4>/filter' */
real32_T FOC_iIqSP_S;                  /* '<S4>/Switch1' */
real32_T FOC_iIdSP_S;                  /* '<S4>/Constant7' */
real32_T FOC_thetaElecMeas_S;          /* '<S1>/Switch3' */
real32_T FOC_sinCoefficient_S;         /* '<S1>/Sine_Cosine' */
real32_T FOC_cosCoefficient_S;         /* '<S1>/Sine_Cosine' */
real32_T FOC_iIdMeas_S;                /* '<S9>/Add1' */
real32_T FOC_iIqMeas_S;                /* '<S9>/Add2' */
real32_T FOC_uVqSP_S;                  /* '<S13>/Switch2' */
real32_T FOC_uVdSP_S;                  /* '<S14>/Switch2' */
real32_T FOC_uPhaseUVolt_S;            /* '<S8>/Add1' */
real32_T FOC_uPhaseVVolt_S;            /* '<S7>/Add' */
real32_T FOC_uPhaseWVolt_S;            /* '<S7>/Add1' */
real32_T SVM_uPhaseU_S;                /* '<S1>/Signal Conversion' */
real32_T SVM_uPhaseV_S;                /* '<S1>/Signal Conversion1' */
real32_T SVM_uPhaseW_S;                /* '<S1>/Signal Conversion2' */
boolean_T FOC_bInitPID_S;              /* '<S2>/AND' */
boolean_T Motor_flagFindPos_S;         /* '<S2>/Chart' */

/* Exported block states */
uint16_T FOC_cntThetaElecOffset_S;     /* '<Root>/Data Store Memory' */

/* Exported data definition */

/* Const memory section */
/* Definition for custom storage class: Const */
const boolean_T FOC_bManualVdVq_C = 0; /* Referenced by: '<S6>/Constant2' */
const real32_T FOC_iIdSP_C = 0.0F;     /* Referenced by: '<S4>/Constant7' */
const real32_T FOC_kiPID_C = 0.001F;   /* Referenced by: '<S6>/Constant1' */
const real32_T FOC_kpPID_C = 0.15F;    /* Referenced by: '<S6>/Constant5' */
const real32_T FOC_uBusVoltage_C = 12.0F;
                       /* Referenced by: '<S6>/paramCurrentControlSatVoltage' */
const real32_T FOC_uFindPosVd_C = 2.0F;/* Referenced by: '<S6>/Constant8' */
const real32_T FOC_uManualVd_C = 0.0F; /* Referenced by: '<S6>/Constant3' */
const real32_T FOC_uManualVq_C = 0.0F; /* Referenced by: '<S6>/Constant4' */
const boolean_T Motor_bFindPosEnable_C = 1;/* Referenced by: '<S2>/Constant1' */
const real32_T Motor_facTrq2Current_C = 0.01F;/* Referenced by: '<S4>/Constant9' */
const uint8_T Motor_swtMode_C = 0U;    /* Referenced by:
                                        * '<S2>/Constant17'
                                        * '<S4>/Constant1'
                                        * '<S4>/Constant17'
                                        * '<S4>/Constant5'
                                        * '<S6>/Constant17'
                                        */
const uint16_T Motor_tiFindPos_C = 2000U;/* Referenced by: '<S2>/Constant2' */
const real32_T Motor_trqTorqueSetPoint_C = 0.0F;/* Referenced by: '<S4>/Constant10' */
const real32_T Position_degSetPoint_C = 0.0F;/* Referenced by: '<S3>/Constant8' */
const real32_T Position_kiPID_C = 0.0015F;/* Referenced by: '<S3>/ki_0.12' */
const real32_T Position_kpPID_C = 0.85F;/* Referenced by: '<S3>/kp_0.0015' */
const real32_T Position_nPIDMax_C = 200.0F;
                    /* Referenced by: '<S3>/paramPositionControlSatCurrent_4' */
const real32_T Speed_iPIDMax_C = 0.8F;
                    /* Referenced by: '<S4>/paramVelocityControlSatCurrent_4' */
const real32_T Speed_kiPID_C = 0.0015F;/* Referenced by: '<S4>/ki_0.12' */
const real32_T Speed_kpPID_C = 0.12F;  /* Referenced by: '<S4>/kp_0.0015' */
const real32_T Speed_nSetPoint_C = 0.0F;/* Referenced by: '<S4>/Constant8' */
const real32_T Speed_tiFilter_C = 50.0F;/* Referenced by: '<S4>/Constant12' */

/* Block signals (default storage) */
B_FOC_Controller_T FOC_Controller_B;

/* Block states (default storage) */
DW_FOC_Controller_T FOC_Controller_DW;

/* Real-time model */
static RT_MODEL_FOC_Controller_T FOC_Controller_M_;
RT_MODEL_FOC_Controller_T *const FOC_Controller_M = &FOC_Controller_M_;
static void rate_scheduler(void);

/*
 *   This function updates active task flag for each subrate.
 * The function is called at model base rate, hence the
 * generated code self-manages all its subrates.
 */
static void rate_scheduler(void)
{
  /* Compute which subrates run during the next base time step.  Subrates
   * are an integer multiple of the base rate counter.  Therefore, the subtask
   * counter is reset when it reaches its limit (zero means run).
   */
  (FOC_Controller_M->Timing.TaskCounters.TID[1])++;
  if ((FOC_Controller_M->Timing.TaskCounters.TID[1]) > 49) {/* Sample time: [0.0002s, 0.0s] */
    FOC_Controller_M->Timing.TaskCounters.TID[1] = 0;
  }

  (FOC_Controller_M->Timing.TaskCounters.TID[2])++;
  if ((FOC_Controller_M->Timing.TaskCounters.TID[2]) > 12499) {/* Sample time: [0.05s, 0.0s] */
    FOC_Controller_M->Timing.TaskCounters.TID[2] = 0;
  }
}

/* Model step function */
void FOC_Controller_step(void)
{
  if (FOC_Controller_M->Timing.TaskCounters.TID[2] == 0) {
    /* Outputs for Atomic SubSystem: '<Root>/ModeRegul' */
    ModeRegul_Co();

    /* End of Outputs for SubSystem: '<Root>/ModeRegul' */

    /* Outputs for Enabled SubSystem: '<Root>/PositionControl' */
    PositionControl_Co();

    /* End of Outputs for SubSystem: '<Root>/PositionControl' */
  }

  /* Outputs for Enabled SubSystem: '<Root>/SpeedControl' */
  SpeedControl_Co();

  /* End of Outputs for SubSystem: '<Root>/SpeedControl' */
  if (FOC_Controller_M->Timing.TaskCounters.TID[1] == 0) {
    /* Outputs for Atomic SubSystem: '<Root>/CurrentControl' */
    CurrentControl_Co();

    /* End of Outputs for SubSystem: '<Root>/CurrentControl' */
  }

  rate_scheduler();
}

/* Model initialize function */
void FOC_Controller_initialize(void)
{
  /* Registration code */

  /* initialize real-time model */
  (void) memset((void *)FOC_Controller_M, 0,
                sizeof(RT_MODEL_FOC_Controller_T));

  /* block I/O */
  (void) memset(((void *) &FOC_Controller_B), 0,
                sizeof(B_FOC_Controller_T));

  /* exported global signals */
  Motor_iTorque_S = 0.0F;
  Speed_nSetPoint_S = 0.0F;
  Speed_nMeas_S = 0.0F;
  FOC_iIqSP_S = 0.0F;
  FOC_thetaElecMeas_S = 0.0F;
  FOC_sinCoefficient_S = 0.0F;
  FOC_cosCoefficient_S = 0.0F;
  FOC_iIdMeas_S = 0.0F;
  FOC_iIqMeas_S = 0.0F;
  FOC_uVqSP_S = 0.0F;
  FOC_uVdSP_S = 0.0F;
  FOC_uPhaseUVolt_S = 0.0F;
  FOC_uPhaseVVolt_S = 0.0F;
  FOC_uPhaseWVolt_S = 0.0F;
  SVM_uPhaseU_S = 0.0F;
  SVM_uPhaseV_S = 0.0F;
  SVM_uPhaseW_S = 0.0F;
  FOC_bInitPID_S = false;
  Motor_flagFindPos_S = false;

  /* states (dwork) */
  (void) memset((void *)&FOC_Controller_DW, 0,
                sizeof(DW_FOC_Controller_T));

  /* exported global states */
  FOC_cntThetaElecOffset_S = 0U;

  /* external inputs */
  FOC_iPhaseAMeas_S = 0.0F;
  FOC_iPhaseBMeas_S = 0.0F;
  FOC_nSpeedMeasRaw_S = 0.0F;
  FOC_cntThetaElec_S = 0U;
  FOC_iPhaseCMeas_S = 0.0F;
  Position_degMeasured_S = 0.0F;

  /* SystemInitialize for Atomic SubSystem: '<Root>/ModeRegul' */
  ModeRegul_Co_Init();

  /* End of SystemInitialize for SubSystem: '<Root>/ModeRegul' */

  /* SystemInitialize for Enabled SubSystem: '<Root>/SpeedControl' */
  SpeedControl_Co_Init();

  /* End of SystemInitialize for SubSystem: '<Root>/SpeedControl' */
}

/* Model terminate function */
void FOC_Controller_terminate(void)
{
  /* (no terminate code required) */
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
