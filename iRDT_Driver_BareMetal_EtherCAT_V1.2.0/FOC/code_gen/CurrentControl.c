/*
 * File: CurrentControl.c
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

#include "CurrentControl.h"
#include "FOC_Controller_private.h"

/* Includes for objects with custom storage classes. */
#include "FOC_Controller.h"

/* Output and update for atomic system: '<Root>/CurrentControl' */
void CurrentControl_Co(void)
{
  real32_T rtb_Angle2Rad;
  real32_T rtb_Gain;
  real32_T rtb_Min;
  real32_T rtb_Switch_Init1_i;
  real32_T rtb_Switch_Sup_pp;
  real32_T rtb_usqrt3;
  boolean_T rtb_LogicalOperator1_c;
  boolean_T rtb_RelationalOperator1;
  boolean_T rtb_RelationalOperator_g;

  /* Switch: '<S1>/Switch3' */
  if (Motor_flagFindPos_S) {
    /* Switch: '<S1>/Switch3' incorporates:
     *  Constant: '<S1>/Constant3'
     */
    FOC_thetaElecMeas_S = 0.0F;
  } else {
    /* Switch: '<S1>/Switch3' incorporates:
     *  Constant: '<S1>/Constant'
     *  DataStoreRead: '<S1>/Data Store Read'
     *  Gain: '<S1>/Hex2Angle'
     *  Inport: '<Root>/FOC_cntThetaElec_S'
     *  S-Function (sfix_bitop): '<S1>/BitwiseOperator'
     *  Sum: '<S1>/Add'
     *  Sum: '<S1>/Add1'
     */
    FOC_thetaElecMeas_S = (real32_T)(((uint32_T)((uint16_T)(((uint16_T)0U) -
      (uint16_T)((uint32_T)FOC_cntThetaElec_S - FOC_cntThetaElecOffset_S)) &
      ((uint16_T)65535U)) * ((uint16_T)46081U)) >> 23);
  }

  /* End of Switch: '<S1>/Switch3' */

  /* Gain: '<S1>/Angle2Rad' */
  rtb_Angle2Rad = 0.0174532924F * FOC_thetaElecMeas_S;

  /* Sum: '<S5>/Add1' incorporates:
   *  Gain: '<S5>/A_Gain'
   *  Gain: '<S5>/B_Gain'
   *  Gain: '<S5>/C_Gain'
   *  Inport: '<Root>/FOC_iPhaseAMeas_S'
   *  Inport: '<Root>/FOC_iPhaseBMeas_S'
   *  Inport: '<Root>/FOC_iPhaseCMeas_S'
   */
  rtb_Min = (0.666666687F * FOC_iPhaseAMeas_S - 0.333333343F * FOC_iPhaseBMeas_S)
    - 0.333333343F * FOC_iPhaseCMeas_S;

  /* Gain: '<S5>/Alpha_Gain4' incorporates:
   *  Inport: '<Root>/FOC_iPhaseBMeas_S'
   *  Inport: '<Root>/FOC_iPhaseCMeas_S'
   *  Sum: '<S5>/Add'
   */
  rtb_Gain = (FOC_iPhaseBMeas_S - FOC_iPhaseCMeas_S) * 0.577350259F;

  /* Gain: '<S6>/1//sqrt(3)' incorporates:
   *  Constant: '<S6>/paramCurrentControlSatVoltage'
   */
  rtb_usqrt3 = 0.577350259F * FOC_uBusVoltage_C;

  /* Trigonometry: '<S1>/Sine_Cosine' */
  FOC_sinCoefficient_S = sinf(rtb_Angle2Rad);

  /* Trigonometry: '<S1>/Sine_Cosine' */
  FOC_cosCoefficient_S = cosf(rtb_Angle2Rad);

  /* Sum: '<S9>/Add1' incorporates:
   *  Product: '<S9>/Product1'
   *  Product: '<S9>/Product2'
   */
  FOC_iIdMeas_S = rtb_Min * FOC_cosCoefficient_S + rtb_Gain *
    FOC_sinCoefficient_S;

  /* Sum: '<S11>/Error' */
  rtb_Angle2Rad = FOC_iIdSP_S - FOC_iIdMeas_S;

  /* Outputs for Atomic SubSystem: '<S11>/PID' */
  /* Switch: '<S16>/Switch_Init1' incorporates:
   *  Constant: '<S16>/Constant'
   *  Constant: '<S6>/Constant1'
   *  Delay: '<S15>/Delay6'
   *  Delay: '<S16>/Id_Integ_Buff'
   *  Product: '<S16>/Gain_I'
   *  Sum: '<S16>/Add1'
   *  Switch: '<S16>/Switch_Init'
   */
  if (FOC_bInitPID_S) {
    rtb_Switch_Init1_i = 0.0F;
  } else {
    if (FOC_Controller_DW.Delay6_DSTATE_g) {
      /* Switch: '<S16>/Switch_Init' incorporates:
       *  Constant: '<S16>/Constant1'
       */
      rtb_Switch_Init1_i = 0.0F;
    } else {
      /* Switch: '<S16>/Switch_Init' */
      rtb_Switch_Init1_i = rtb_Angle2Rad;
    }

    rtb_Switch_Init1_i = rtb_Switch_Init1_i * FOC_kiPID_C +
      FOC_Controller_DW.Id_Integ_Buff_DSTATE_j;
  }

  /* End of Switch: '<S16>/Switch_Init1' */

  /* Sum: '<S16>/Add2' incorporates:
   *  Constant: '<S11>/Constant5'
   *  Constant: '<S6>/Constant5'
   *  Delay: '<S16>/Delay1'
   *  Product: '<S16>/Gain_P'
   *  Product: '<S16>/Gain_d'
   *  Sum: '<S16>/Add3'
   */
  rtb_Switch_Sup_pp = (rtb_Angle2Rad * FOC_kpPID_C + rtb_Switch_Init1_i) +
    (rtb_Angle2Rad - FOC_Controller_DW.Delay1_DSTATE_j) * 0.0F;

  /* RelationalOperator: '<S20>/Relational Operator1' */
  rtb_RelationalOperator1 = (rtb_usqrt3 <= rtb_Switch_Sup_pp);

  /* Logic: '<S17>/LogicalOperator1' incorporates:
   *  Constant: '<S19>/Constant'
   *  RelationalOperator: '<S19>/Compare'
   */
  rtb_LogicalOperator1_c = (rtb_RelationalOperator1 && (rtb_Angle2Rad > 0.0F));

  /* Switch: '<S20>/Switch_Sup' */
  if (rtb_RelationalOperator1) {
    rtb_Switch_Sup_pp = rtb_usqrt3;
  }

  /* End of Switch: '<S20>/Switch_Sup' */

  /* RelationalOperator: '<S20>/Relational Operator' incorporates:
   *  UnaryMinus: '<S15>/Unary Minus2'
   */
  rtb_RelationalOperator1 = (rtb_Switch_Sup_pp >= -rtb_usqrt3);

  /* Switch: '<S20>/Switch_Inf' incorporates:
   *  UnaryMinus: '<S15>/Unary Minus2'
   */
  if (!rtb_RelationalOperator1) {
    rtb_Switch_Sup_pp = -rtb_usqrt3;
  }

  /* End of Switch: '<S20>/Switch_Inf' */

  /* Update for Delay: '<S15>/Delay6' incorporates:
   *  Constant: '<S18>/Constant'
   *  Logic: '<S17>/LogicalOperator2'
   *  Logic: '<S17>/LogicalOperator3'
   *  Logic: '<S20>/LogicalOperator2'
   *  RelationalOperator: '<S18>/Compare'
   */
  FOC_Controller_DW.Delay6_DSTATE_g = (rtb_LogicalOperator1_c ||
    ((!rtb_RelationalOperator1) && (rtb_Angle2Rad < 0.0F)));

  /* Update for Delay: '<S16>/Id_Integ_Buff' */
  FOC_Controller_DW.Id_Integ_Buff_DSTATE_j = rtb_Switch_Init1_i;

  /* Update for Delay: '<S16>/Delay1' */
  FOC_Controller_DW.Delay1_DSTATE_j = rtb_Angle2Rad;

  /* End of Outputs for SubSystem: '<S11>/PID' */

  /* Logic: '<S6>/Logical Operator' incorporates:
   *  Constant: '<S6>/Constant17'
   *  Constant: '<S6>/Constant2'
   *  Constant: '<S6>/Constant24'
   *  RelationalOperator: '<S6>/Relational Operator'
   */
  rtb_LogicalOperator1_c = ((Motor_swtMode_C == ((uint8_T)0U)) ||
    FOC_bManualVdVq_C);

  /* Sum: '<S9>/Add2' incorporates:
   *  Product: '<S9>/Product3'
   *  Product: '<S9>/Product4'
   */
  FOC_iIqMeas_S = rtb_Gain * FOC_cosCoefficient_S - rtb_Min *
    FOC_sinCoefficient_S;

  /* Sum: '<S12>/Error' */
  rtb_Angle2Rad = FOC_iIqSP_S - FOC_iIqMeas_S;

  /* Switch: '<S22>/Switch_Init1' incorporates:
   *  Constant: '<S22>/Constant'
   *  Constant: '<S6>/Constant1'
   *  Delay: '<S21>/Delay6'
   *  Delay: '<S22>/Iq_Integ_Buff'
   *  Product: '<S22>/Gain_I'
   *  Sum: '<S22>/Add1'
   *  Switch: '<S22>/Switch_Init'
   */
  if (FOC_bInitPID_S) {
    rtb_Switch_Init1_i = 0.0F;
  } else {
    if (FOC_Controller_DW.Delay6_DSTATE_i) {
      /* Switch: '<S22>/Switch_Init' incorporates:
       *  Constant: '<S22>/Constant1'
       */
      rtb_Switch_Init1_i = 0.0F;
    } else {
      /* Switch: '<S22>/Switch_Init' */
      rtb_Switch_Init1_i = rtb_Angle2Rad;
    }

    rtb_Switch_Init1_i = rtb_Switch_Init1_i * FOC_kiPID_C +
      FOC_Controller_DW.Iq_Integ_Buff_DSTATE;
  }

  /* End of Switch: '<S22>/Switch_Init1' */

  /* Sum: '<S22>/Add2' incorporates:
   *  Constant: '<S12>/Constant5'
   *  Constant: '<S6>/Constant5'
   *  Delay: '<S22>/Delay1'
   *  Product: '<S22>/Gain_P'
   *  Product: '<S22>/Gain_d'
   *  Sum: '<S22>/Add3'
   */
  rtb_Gain = (rtb_Angle2Rad * FOC_kpPID_C + rtb_Switch_Init1_i) + (rtb_Angle2Rad
    - FOC_Controller_DW.Delay1_DSTATE_c) * 0.0F;

  /* RelationalOperator: '<S26>/Relational Operator1' */
  rtb_RelationalOperator1 = (rtb_usqrt3 <= rtb_Gain);

  /* Switch: '<S26>/Switch_Sup' */
  if (rtb_RelationalOperator1) {
    rtb_Min = rtb_usqrt3;
  } else {
    rtb_Min = rtb_Gain;
  }

  /* End of Switch: '<S26>/Switch_Sup' */

  /* RelationalOperator: '<S26>/Relational Operator' incorporates:
   *  UnaryMinus: '<S21>/Unary Minus2'
   */
  rtb_RelationalOperator_g = (rtb_Min >= -rtb_usqrt3);

  /* Switch: '<S6>/Switch2' incorporates:
   *  Constant: '<S6>/Constant7'
   *  Switch: '<S26>/Switch_Inf'
   *  Switch: '<S6>/Switch_Init2'
   */
  if (Motor_flagFindPos_S) {
    rtb_Min = 0.0F;
  } else if (rtb_LogicalOperator1_c) {
    /* Switch: '<S6>/Switch_Init2' incorporates:
     *  Constant: '<S6>/Constant4'
     */
    rtb_Min = FOC_uManualVq_C;
  } else if (!rtb_RelationalOperator_g) {
    /* Switch: '<S6>/Switch_Init2' incorporates:
     *  UnaryMinus: '<S21>/Unary Minus2'
     */
    rtb_Min = -rtb_usqrt3;
  }

  /* End of Switch: '<S6>/Switch2' */

  /* Switch: '<S13>/Switch2' incorporates:
   *  RelationalOperator: '<S13>/LowerRelop1'
   *  RelationalOperator: '<S13>/UpperRelop'
   *  Switch: '<S13>/Switch'
   *  UnaryMinus: '<S6>/Unary Minus'
   */
  if (rtb_Min > rtb_usqrt3) {
    /* Switch: '<S13>/Switch2' */
    FOC_uVqSP_S = rtb_usqrt3;
  } else if (rtb_Min < -rtb_usqrt3) {
    /* Switch: '<S13>/Switch' incorporates:
     *  Switch: '<S13>/Switch2'
     *  UnaryMinus: '<S6>/Unary Minus'
     */
    FOC_uVqSP_S = -rtb_usqrt3;
  } else {
    /* Switch: '<S13>/Switch2' incorporates:
     *  Switch: '<S13>/Switch'
     */
    FOC_uVqSP_S = rtb_Min;
  }

  /* End of Switch: '<S13>/Switch2' */

  /* Switch: '<S6>/Switch1' incorporates:
   *  Constant: '<S6>/Constant8'
   *  Switch: '<S6>/Switch_Init1'
   */
  if (Motor_flagFindPos_S) {
    rtb_Min = FOC_uFindPosVd_C;
  } else if (rtb_LogicalOperator1_c) {
    /* Switch: '<S6>/Switch_Init1' incorporates:
     *  Constant: '<S6>/Constant3'
     */
    rtb_Min = FOC_uManualVd_C;
  } else {
    rtb_Min = rtb_Switch_Sup_pp;
  }

  /* End of Switch: '<S6>/Switch1' */

  /* Switch: '<S14>/Switch2' incorporates:
   *  RelationalOperator: '<S14>/LowerRelop1'
   *  RelationalOperator: '<S14>/UpperRelop'
   *  Switch: '<S14>/Switch'
   *  UnaryMinus: '<S6>/Unary Minus1'
   */
  if (rtb_Min > rtb_usqrt3) {
    /* Switch: '<S14>/Switch2' */
    FOC_uVdSP_S = rtb_usqrt3;
  } else if (rtb_Min < -rtb_usqrt3) {
    /* Switch: '<S14>/Switch' incorporates:
     *  Switch: '<S14>/Switch2'
     *  UnaryMinus: '<S6>/Unary Minus1'
     */
    FOC_uVdSP_S = -rtb_usqrt3;
  } else {
    /* Switch: '<S14>/Switch2' incorporates:
     *  Switch: '<S14>/Switch'
     */
    FOC_uVdSP_S = rtb_Min;
  }

  /* End of Switch: '<S14>/Switch2' */

  /* Sum: '<S8>/Add1' incorporates:
   *  Product: '<S8>/Product1'
   *  Product: '<S8>/Product2'
   */
  FOC_uPhaseUVolt_S = FOC_uVdSP_S * FOC_cosCoefficient_S - FOC_uVqSP_S *
    FOC_sinCoefficient_S;

  /* Gain: '<S7>/Gain' */
  rtb_Gain = 0.5F * FOC_uPhaseUVolt_S;

  /* Gain: '<S7>/Gain1' incorporates:
   *  Product: '<S8>/Product3'
   *  Product: '<S8>/Product4'
   *  Sum: '<S8>/Add2'
   */
  rtb_Min = (FOC_uVdSP_S * FOC_sinCoefficient_S + FOC_uVqSP_S *
             FOC_cosCoefficient_S) * 0.866025388F;

  /* Sum: '<S7>/Add' */
  FOC_uPhaseVVolt_S = rtb_Min - rtb_Gain;

  /* Sum: '<S7>/Add1' */
  FOC_uPhaseWVolt_S = (0.0F - rtb_Gain) - rtb_Min;

  /* Gain: '<S10>/Gain' incorporates:
   *  MinMax: '<S10>/Max'
   *  MinMax: '<S10>/Min'
   *  Sum: '<S10>/Add'
   */
  rtb_Gain = (fmaxf(fmaxf(FOC_uPhaseUVolt_S, FOC_uPhaseVVolt_S),
                    FOC_uPhaseWVolt_S) + fminf(fminf(FOC_uPhaseUVolt_S,
    FOC_uPhaseVVolt_S), FOC_uPhaseWVolt_S)) * 0.5F;

  /* SignalConversion: '<S1>/Signal Conversion' incorporates:
   *  Sum: '<S10>/Add1'
   */
  SVM_uPhaseU_S = FOC_uPhaseUVolt_S - rtb_Gain;

  /* SignalConversion: '<S1>/Signal Conversion1' incorporates:
   *  Sum: '<S10>/Add1'
   */
  SVM_uPhaseV_S = FOC_uPhaseVVolt_S - rtb_Gain;

  /* SignalConversion: '<S1>/Signal Conversion2' incorporates:
   *  Sum: '<S10>/Add1'
   */
  SVM_uPhaseW_S = FOC_uPhaseWVolt_S - rtb_Gain;

  /* Update for Delay: '<S21>/Delay6' incorporates:
   *  Constant: '<S24>/Constant'
   *  Constant: '<S25>/Constant'
   *  Logic: '<S23>/LogicalOperator1'
   *  Logic: '<S23>/LogicalOperator2'
   *  Logic: '<S23>/LogicalOperator3'
   *  Logic: '<S26>/LogicalOperator2'
   *  RelationalOperator: '<S24>/Compare'
   *  RelationalOperator: '<S25>/Compare'
   */
  FOC_Controller_DW.Delay6_DSTATE_i = ((rtb_RelationalOperator1 &&
    (rtb_Angle2Rad > 0.0F)) || ((!rtb_RelationalOperator_g) && (rtb_Angle2Rad <
    0.0F)));

  /* Update for Delay: '<S22>/Iq_Integ_Buff' */
  FOC_Controller_DW.Iq_Integ_Buff_DSTATE = rtb_Switch_Init1_i;

  /* Update for Delay: '<S22>/Delay1' */
  FOC_Controller_DW.Delay1_DSTATE_c = rtb_Angle2Rad;
}

/* Output and update for atomic system: '<S4>/filter' */
void lib_filter(real32_T rtu_Filter_In, real32_T rtu_t_ms, real32_T rtu_dt_ms,
                B_lib_filter_T *localB)
{
  real32_T rtb_a;

  /* Product: '<S37>/Product1' incorporates:
   *  Sum: '<S37>/Add2'
   */
  rtb_a = rtu_dt_ms / (rtu_t_ms + rtu_dt_ms);

  /* Sum: '<S37>/Add' incorporates:
   *  Constant: '<S37>/Constant'
   *  Delay: '<S37>/Delay'
   *  Product: '<S37>/Product'
   *  Product: '<S37>/Product2'
   *  Sum: '<S37>/Add1'
   */
  localB->Yn = (1.0F - rtb_a) * localB->Yn + rtb_a * rtu_Filter_In;
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
