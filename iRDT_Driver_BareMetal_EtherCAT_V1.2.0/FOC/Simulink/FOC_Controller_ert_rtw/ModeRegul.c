/*
 * File: ModeRegul.c
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

#include "ModeRegul.h"
#include "FOC_Controller_private.h"

/* Includes for objects with custom storage classes. */
#include "FOC_Controller.h"

/* Named constants for Chart: '<S2>/Chart' */
#define FOC_Controller_IN_Alignment    ((uint8_T)1U)
#define FOC_Controller_IN_Initialization ((uint8_T)2U)
#define FOC_Controller_IN_NO_ACTIVE_CHILD ((uint8_T)0U)
#define FOC_Controller_IN_Ready        ((uint8_T)3U)

/* System initialize for atomic system: '<Root>/ModeRegul' */
void ModeRegul_Co_Init(void)
{
  /* SystemInitialize for Chart: '<S2>/Chart' */
  FOC_Controller_DW.is_active_c5_FOC_Controller = 0U;
  FOC_Controller_DW.is_c5_FOC_Controller = FOC_Controller_IN_NO_ACTIVE_CHILD;
  FOC_Controller_DW.holdtime = 0U;
  Motor_flagFindPos_S = false;
  FOC_Controller_B.bControlEnable = false;
  FOC_Controller_B.calibration_angle = 0U;
}

/* Output and update for atomic system: '<Root>/ModeRegul' */
void ModeRegul_Co(void)
{
  real32_T tmp;

  /* Chart: '<S2>/Chart' incorporates:
   *  Constant: '<S2>/Constant1'
   *  Constant: '<S2>/Constant2'
   *  Constant: '<S2>/Constant3'
   *  Inport: '<Root>/FOC_cntThetaElec_S'
   */
  /* Gateway: ModeRegul/Chart */
  /* During: ModeRegul/Chart */
  if (FOC_Controller_DW.is_active_c5_FOC_Controller == 0U) {
    /* Entry: ModeRegul/Chart */
    FOC_Controller_DW.is_active_c5_FOC_Controller = 1U;

    /* Entry Internal: ModeRegul/Chart */
    /* Transition: '<S27>:4' */
    FOC_Controller_DW.is_c5_FOC_Controller = FOC_Controller_IN_Initialization;

    /* Entry 'Initialization': '<S27>:1' */
    FOC_Controller_B.bControlEnable = false;
    Motor_flagFindPos_S = false;
  } else {
    switch (FOC_Controller_DW.is_c5_FOC_Controller) {
     case FOC_Controller_IN_Alignment:
      FOC_Controller_B.bControlEnable = false;

      /* During 'Alignment': '<S27>:9' */
      if (FOC_Controller_DW.holdtime > Motor_tiFindPos_C) {
        /* Transition: '<S27>:16' */
        Motor_flagFindPos_S = false;
        FOC_Controller_B.calibration_angle = FOC_cntThetaElec_S;
        FOC_Controller_DW.is_c5_FOC_Controller = FOC_Controller_IN_Ready;

        /* Entry 'Ready': '<S27>:15' */
        FOC_Controller_B.bControlEnable = true;
      } else {
        tmp = (real32_T)FOC_Controller_DW.holdtime + 50.0F;
        if (tmp < 65536.0F) {
          if (tmp >= 0.0F) {
            FOC_Controller_DW.holdtime = (uint16_T)tmp;
          } else {
            FOC_Controller_DW.holdtime = 0U;
          }
        } else {
          FOC_Controller_DW.holdtime = MAX_uint16_T;
        }
      }
      break;

     case FOC_Controller_IN_Initialization:
      FOC_Controller_B.bControlEnable = false;

      /* During 'Initialization': '<S27>:1' */
      if (Motor_bFindPosEnable_C) {
        /* Transition: '<S27>:11' */
        FOC_Controller_DW.holdtime = 0U;
        FOC_Controller_DW.is_c5_FOC_Controller = FOC_Controller_IN_Alignment;

        /* Entry 'Alignment': '<S27>:9' */
        FOC_Controller_B.bControlEnable = false;
        Motor_flagFindPos_S = true;
      } else {
        /* Transition: '<S27>:99' */
        FOC_Controller_DW.is_c5_FOC_Controller = FOC_Controller_IN_Ready;

        /* Entry 'Ready': '<S27>:15' */
        FOC_Controller_B.bControlEnable = true;
      }
      break;

     default:
      FOC_Controller_B.bControlEnable = true;

      /* During 'Ready': '<S27>:15' */
      break;
    }
  }

  /* End of Chart: '<S2>/Chart' */

  /* DataStoreWrite: '<S2>/Data Store Write' */
  FOC_cntThetaElecOffset_S = FOC_Controller_B.calibration_angle;

  /* Logic: '<S2>/AND' incorporates:
   *  Constant: '<S2>/Constant17'
   *  Constant: '<S2>/Constant24'
   *  Delay: '<S2>/Delay'
   *  RelationalOperator: '<S2>/Relational Operator'
   *  RelationalOperator: '<S2>/Relational Operator2'
   */
  FOC_bInitPID_S = ((FOC_Controller_B.bControlEnable == false) ||
                    (Motor_swtMode_C != FOC_Controller_DW.Delay_DSTATE));

  /* Update for Delay: '<S2>/Delay' incorporates:
   *  Constant: '<S2>/Constant17'
   */
  FOC_Controller_DW.Delay_DSTATE = Motor_swtMode_C;
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
