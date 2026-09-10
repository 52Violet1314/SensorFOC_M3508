/*
 * File: CurrentControl.h
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

#ifndef RTW_HEADER_CurrentControl_h_
#define RTW_HEADER_CurrentControl_h_
#include <math.h>
#ifndef FOC_Controller_COMMON_INCLUDES_
#define FOC_Controller_COMMON_INCLUDES_
#include "rtwtypes.h"
#endif                                 /* FOC_Controller_COMMON_INCLUDES_ */

#include "FOC_Controller_types.h"

/* Block signals for system '<S4>/filter' */
typedef struct {
  real32_T Yn;                         /* '<S37>/Add' */
} B_lib_filter_T;

extern void lib_filter(real32_T rtu_Filter_In, real32_T rtu_t_ms, real32_T
  rtu_dt_ms, B_lib_filter_T *localB);
extern void CurrentControl_Co(void);

#endif                                 /* RTW_HEADER_CurrentControl_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
