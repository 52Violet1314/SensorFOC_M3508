/*
 * File: SpeedControl.c
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

#include "SpeedControl.h"
#include "FOC_Controller_private.h"

/* Includes for objects with custom storage classes. */
#include "FOC_Controller.h"

/* System initialize for enable system: '<Root>/SpeedControl' */
void SpeedControl_Co_Init(void)
{
  /* SystemInitialize for Outport: '<S4>/current_Id' incorporates:
   *  Constant: '<S4>/Constant7'
   */
  FOC_iIdSP_S = 0.0F;
}

/* Disable for enable system: '<Root>/SpeedControl' */
void SpeedControl_Co_Disable(void)
{
  FOC_Controller_DW.SpeedControl_MODE = false;
}

/* Output and update for enable system: '<Root>/SpeedControl' */
void SpeedControl_Co(void)
{
  real32_T rtb_Switch_Init1_j;
  real32_T rtb_Switch_Sup_o;
  real32_T rtb_UnaryMinus2_d;
  boolean_T rtb_LogicalOperator1_a;
  boolean_T rtb_RelationalOperator_d;

  /* Outputs for Enabled SubSystem: '<Root>/SpeedControl' incorporates:
   *  EnablePort: '<S4>/Enable'
   */
  if (FOC_Controller_M->Timing.TaskCounters.TID[2] == 0) {
    if (FOC_Controller_B.bControlEnable) {
      FOC_Controller_DW.SpeedControl_MODE = true;
    } else if (FOC_Controller_DW.SpeedControl_MODE) {
      SpeedControl_Co_Disable();
    }
  }

  if (FOC_Controller_DW.SpeedControl_MODE) {
    if (FOC_Controller_M->Timing.TaskCounters.TID[2] == 0) {
      /* Product: '<S4>/Divide' incorporates:
       *  Constant: '<S4>/Constant10'
       *  Constant: '<S4>/Constant9'
       */
      rtb_UnaryMinus2_d = Motor_trqTorqueSetPoint_C / Motor_facTrq2Current_C;

      /* Switch: '<S35>/Switch_Sup' incorporates:
       *  Constant: '<S4>/paramVelocityControlSatCurrent_4'
       *  RelationalOperator: '<S35>/Relational Operator1'
       */
      if (Speed_iPIDMax_C <= rtb_UnaryMinus2_d) {
        rtb_UnaryMinus2_d = Speed_iPIDMax_C;
      }

      /* End of Switch: '<S35>/Switch_Sup' */

      /* Switch: '<S35>/Switch_Inf' incorporates:
       *  Constant: '<S4>/paramVelocityControlSatCurrent_4'
       *  RelationalOperator: '<S35>/Relational Operator'
       *  UnaryMinus: '<S4>/Unary Minus3'
       */
      if (rtb_UnaryMinus2_d > -Speed_iPIDMax_C) {
        /* Switch: '<S35>/Switch_Inf' */
        Motor_iTorque_S = rtb_UnaryMinus2_d;
      } else {
        /* Switch: '<S35>/Switch_Inf' */
        Motor_iTorque_S = -Speed_iPIDMax_C;
      }

      /* End of Switch: '<S35>/Switch_Inf' */

      /* Switch: '<S4>/Switch2' incorporates:
       *  Constant: '<S4>/Constant5'
       *  Constant: '<S4>/Constant6'
       *  RelationalOperator: '<S4>/Relational Operator1'
       */
      if (Motor_swtMode_C == ((uint8_T)3U)) {
        /* Switch: '<S4>/Switch2' */
        Speed_nSetPoint_S = FOC_Controller_B.Switch_Inf;
      } else {
        /* Switch: '<S4>/Switch2' incorporates:
         *  Constant: '<S4>/Constant8'
         */
        Speed_nSetPoint_S = Speed_nSetPoint_C;
      }

      /* End of Switch: '<S4>/Switch2' */

      /* Outputs for Atomic SubSystem: '<S4>/filter' */
      /* Inport: '<Root>/FOC_nSpeedMeasRaw_S' incorporates:
       *  Constant: '<S4>/Constant12'
       *  Constant: '<S4>/Constant4'
       */
      lib_filter(FOC_nSpeedMeasRaw_S, Speed_tiFilter_C, 10.0F,
                 &FOC_Controller_B.filter);

      /* End of Outputs for SubSystem: '<S4>/filter' */

      /* SignalConversion generated from: '<S4>/filter' */
      Speed_nMeas_S = FOC_Controller_B.filter.Yn;

      /* Sum: '<S36>/Error' */
      rtb_UnaryMinus2_d = Speed_nSetPoint_S - Speed_nMeas_S;

      /* Outputs for Atomic SubSystem: '<S36>/PID' */
      /* Switch: '<S39>/Switch_Init1' incorporates:
       *  Constant: '<S39>/Constant'
       *  Constant: '<S4>/ki_0.12'
       *  Delay: '<S38>/Delay6'
       *  Delay: '<S39>/Id_Integ_Buff'
       *  Product: '<S39>/Gain_I'
       *  Sum: '<S39>/Add1'
       *  Switch: '<S39>/Switch_Init'
       */
      if (FOC_bInitPID_S) {
        rtb_Switch_Init1_j = 0.0F;
      } else {
        if (FOC_Controller_DW.Delay6_DSTATE) {
          /* Switch: '<S39>/Switch_Init' incorporates:
           *  Constant: '<S39>/Constant1'
           */
          rtb_Switch_Init1_j = 0.0F;
        } else {
          /* Switch: '<S39>/Switch_Init' */
          rtb_Switch_Init1_j = rtb_UnaryMinus2_d;
        }

        rtb_Switch_Init1_j = rtb_Switch_Init1_j * Speed_kiPID_C +
          FOC_Controller_DW.Id_Integ_Buff_DSTATE;
      }

      /* End of Switch: '<S39>/Switch_Init1' */

      /* Sum: '<S39>/Add2' incorporates:
       *  Constant: '<S36>/Constant5'
       *  Constant: '<S4>/kp_0.0015'
       *  Delay: '<S39>/Delay1'
       *  Product: '<S39>/Gain_P'
       *  Product: '<S39>/Gain_d'
       *  Sum: '<S39>/Add3'
       */
      rtb_Switch_Sup_o = (rtb_UnaryMinus2_d * Speed_kpPID_C + rtb_Switch_Init1_j)
        + (rtb_UnaryMinus2_d - FOC_Controller_DW.Delay1_DSTATE) * 0.0F;

      /* RelationalOperator: '<S43>/Relational Operator1' incorporates:
       *  Constant: '<S4>/paramVelocityControlSatCurrent_4'
       */
      rtb_RelationalOperator_d = (Speed_iPIDMax_C <= rtb_Switch_Sup_o);

      /* Logic: '<S40>/LogicalOperator1' incorporates:
       *  Constant: '<S42>/Constant'
       *  RelationalOperator: '<S42>/Compare'
       */
      rtb_LogicalOperator1_a = (rtb_RelationalOperator_d && (rtb_UnaryMinus2_d >
        0.0F));

      /* Switch: '<S43>/Switch_Sup' incorporates:
       *  Constant: '<S4>/paramVelocityControlSatCurrent_4'
       */
      if (rtb_RelationalOperator_d) {
        rtb_Switch_Sup_o = Speed_iPIDMax_C;
      }

      /* End of Switch: '<S43>/Switch_Sup' */

      /* RelationalOperator: '<S43>/Relational Operator' incorporates:
       *  Constant: '<S4>/paramVelocityControlSatCurrent_4'
       *  UnaryMinus: '<S38>/Unary Minus2'
       */
      rtb_RelationalOperator_d = (rtb_Switch_Sup_o >= -Speed_iPIDMax_C);

      /* Update for Delay: '<S38>/Delay6' incorporates:
       *  Constant: '<S41>/Constant'
       *  Logic: '<S40>/LogicalOperator2'
       *  Logic: '<S40>/LogicalOperator3'
       *  Logic: '<S43>/LogicalOperator2'
       *  RelationalOperator: '<S41>/Compare'
       */
      FOC_Controller_DW.Delay6_DSTATE = (rtb_LogicalOperator1_a ||
        ((!rtb_RelationalOperator_d) && (rtb_UnaryMinus2_d < 0.0F)));

      /* Update for Delay: '<S39>/Id_Integ_Buff' */
      FOC_Controller_DW.Id_Integ_Buff_DSTATE = rtb_Switch_Init1_j;

      /* Update for Delay: '<S39>/Delay1' */
      FOC_Controller_DW.Delay1_DSTATE = rtb_UnaryMinus2_d;

      /* End of Outputs for SubSystem: '<S36>/PID' */

      /* Switch: '<S4>/Switch1' incorporates:
       *  Constant: '<S4>/Constant1'
       *  Constant: '<S4>/Constant17'
       *  Constant: '<S4>/Constant2'
       *  Constant: '<S4>/Constant24'
       *  Constant: '<S4>/Constant3'
       *  RelationalOperator: '<S4>/Relational Operator'
       *  RelationalOperator: '<S4>/Relational Operator2'
       *  Switch: '<S4>/Switch3'
       */
      if (Motor_swtMode_C == ((uint8_T)1U)) {
        FOC_iIqSP_S = Motor_iTorque_S;
      } else if (Motor_swtMode_C >= ((uint8_T)2U)) {
        /* Outputs for Atomic SubSystem: '<S36>/PID' */
        /* Switch: '<S43>/Switch_Inf' */
        if (rtb_RelationalOperator_d) {
          /* Switch: '<S4>/Switch3' */
          FOC_iIqSP_S = rtb_Switch_Sup_o;
        } else {
          /* Switch: '<S4>/Switch3' incorporates:
           *  Constant: '<S4>/paramVelocityControlSatCurrent_4'
           *  UnaryMinus: '<S38>/Unary Minus2'
           */
          FOC_iIqSP_S = -Speed_iPIDMax_C;
        }

        /* End of Switch: '<S43>/Switch_Inf' */
        /* End of Outputs for SubSystem: '<S36>/PID' */
      } else {
        FOC_iIqSP_S = 0.0F;
      }

      /* End of Switch: '<S4>/Switch1' */
    }

    if (FOC_Controller_M->Timing.TaskCounters.TID[1] == 0) {
      /* Constant: '<S4>/Constant7' */
      FOC_iIdSP_S = FOC_iIdSP_C;
    }
  }

  /* End of Outputs for SubSystem: '<Root>/SpeedControl' */
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
