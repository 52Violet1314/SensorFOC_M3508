/*
 * File: SpeedControl.h
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

#ifndef RTW_HEADER_SpeedControl_h_
#define RTW_HEADER_SpeedControl_h_
#ifndef FOC_Controller_COMMON_INCLUDES_
#define FOC_Controller_COMMON_INCLUDES_
#include "rtwtypes.h"
#endif                                 /* FOC_Controller_COMMON_INCLUDES_ */

#include "FOC_Controller_types.h"

/* Child system includes */
#include "CurrentControl.h"

extern void SpeedControl_Co_Init(void);
extern void SpeedControl_Co_Disable(void);
extern void SpeedControl_Co(void);

#endif                                 /* RTW_HEADER_SpeedControl_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
