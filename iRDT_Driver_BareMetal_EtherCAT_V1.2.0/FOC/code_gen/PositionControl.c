/*
 * File: PositionControl.c
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

#include "PositionControl.h"
#include "FOC_Controller_private.h"

/* Includes for objects with custom storage classes. */
#include "FOC_Controller.h"

/* Output and update for enable system: '<Root>/PositionControl' */
void PositionControl_Co(void)
{
  real32_T rtb_Switch_Init1_g;
  real32_T rtb_Switch_Sup_i;
  real32_T rtb_currentControlError_e;
  boolean_T rtb_LogicalOperator1_a;
  boolean_T rtb_RelationalOperator_k;

  /* Outputs for Enabled SubSystem: '<Root>/PositionControl' incorporates:
   *  EnablePort: '<S3>/Enable'
   */
  if (FOC_Controller_B.bControlEnable) {
    /* Sum: '<S28>/Error' incorporates:
     *  Constant: '<S3>/Constant8'
     *  Inport: '<Root>/Position_degMeasured_S'
     */
    rtb_currentControlError_e = Position_degSetPoint_C - Position_degMeasured_S;

    /* Outputs for Atomic SubSystem: '<S28>/PID' */
    /* Switch: '<S30>/Switch_Init1' incorporates:
     *  Constant: '<S30>/Constant'
     *  Constant: '<S3>/ki_0.12'
     *  Delay: '<S29>/Delay6'
     *  Delay: '<S30>/Id_Integ_Buff'
     *  Product: '<S30>/Gain_I'
     *  Sum: '<S30>/Add1'
     *  Switch: '<S30>/Switch_Init'
     */
    if (FOC_bInitPID_S) {
      rtb_Switch_Init1_g = 0.0F;
    } else {
      if (FOC_Controller_DW.Delay6_DSTATE_c) {
        /* Switch: '<S30>/Switch_Init' incorporates:
         *  Constant: '<S30>/Constant1'
         */
        rtb_Switch_Init1_g = 0.0F;
      } else {
        /* Switch: '<S30>/Switch_Init' */
        rtb_Switch_Init1_g = rtb_currentControlError_e;
      }

      rtb_Switch_Init1_g = rtb_Switch_Init1_g * Position_kiPID_C +
        FOC_Controller_DW.Id_Integ_Buff_DSTATE_m;
    }

    /* End of Switch: '<S30>/Switch_Init1' */

    /* Sum: '<S30>/Add2' incorporates:
     *  Constant: '<S28>/Constant5'
     *  Constant: '<S3>/kp_0.0015'
     *  Delay: '<S30>/Delay1'
     *  Product: '<S30>/Gain_P'
     *  Product: '<S30>/Gain_d'
     *  Sum: '<S30>/Add3'
     */
    rtb_Switch_Sup_i = (rtb_currentControlError_e * Position_kpPID_C +
                        rtb_Switch_Init1_g) + (rtb_currentControlError_e -
      FOC_Controller_DW.Delay1_DSTATE_g) * 0.0F;

    /* RelationalOperator: '<S34>/Relational Operator1' incorporates:
     *  Constant: '<S3>/paramPositionControlSatCurrent_4'
     */
    rtb_RelationalOperator_k = (Position_nPIDMax_C <= rtb_Switch_Sup_i);

    /* Logic: '<S31>/LogicalOperator1' incorporates:
     *  Constant: '<S33>/Constant'
     *  RelationalOperator: '<S33>/Compare'
     */
    rtb_LogicalOperator1_a = (rtb_RelationalOperator_k &&
      (rtb_currentControlError_e > 0.0F));

    /* Switch: '<S34>/Switch_Sup' incorporates:
     *  Constant: '<S3>/paramPositionControlSatCurrent_4'
     */
    if (rtb_RelationalOperator_k) {
      rtb_Switch_Sup_i = Position_nPIDMax_C;
    }

    /* End of Switch: '<S34>/Switch_Sup' */

    /* RelationalOperator: '<S34>/Relational Operator' incorporates:
     *  Constant: '<S3>/paramPositionControlSatCurrent_4'
     *  UnaryMinus: '<S29>/Unary Minus2'
     */
    rtb_RelationalOperator_k = (rtb_Switch_Sup_i >= -Position_nPIDMax_C);

    /* Switch: '<S34>/Switch_Inf' */
    if (rtb_RelationalOperator_k) {
      /* Switch: '<S34>/Switch_Inf' */
      FOC_Controller_B.Switch_Inf = rtb_Switch_Sup_i;
    } else {
      /* Switch: '<S34>/Switch_Inf' incorporates:
       *  Constant: '<S3>/paramPositionControlSatCurrent_4'
       *  UnaryMinus: '<S29>/Unary Minus2'
       */
      FOC_Controller_B.Switch_Inf = -Position_nPIDMax_C;
    }

    /* End of Switch: '<S34>/Switch_Inf' */

    /* Update for Delay: '<S29>/Delay6' incorporates:
     *  Constant: '<S32>/Constant'
     *  Logic: '<S31>/LogicalOperator2'
     *  Logic: '<S31>/LogicalOperator3'
     *  Logic: '<S34>/LogicalOperator2'
     *  RelationalOperator: '<S32>/Compare'
     */
    FOC_Controller_DW.Delay6_DSTATE_c = (rtb_LogicalOperator1_a ||
      ((!rtb_RelationalOperator_k) && (rtb_currentControlError_e < 0.0F)));

    /* Update for Delay: '<S30>/Id_Integ_Buff' */
    FOC_Controller_DW.Id_Integ_Buff_DSTATE_m = rtb_Switch_Init1_g;

    /* Update for Delay: '<S30>/Delay1' */
    FOC_Controller_DW.Delay1_DSTATE_g = rtb_currentControlError_e;

    /* End of Outputs for SubSystem: '<S28>/PID' */
  }

  /* End of Outputs for SubSystem: '<Root>/PositionControl' */
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
