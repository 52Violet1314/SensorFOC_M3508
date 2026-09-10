/*************************************************************************************************
*   File Name            : bsp_pwm.h
*   File Descriptions    : low-level driver for pwm output and current sample
*   Kernel Architecture  : Cortex-M4
*   MCU Series           : STM32G474
*
*   (c) Copyright 2026-2036 iRobotTribe Technology Co.,Ltd.
*   All Rights Reserved.
*
*   iRobotTribe Confidential. This software is owned or controlled by iRobotTribe and may only be
*   used strictly in accordance with the applicable license terms. By expressly
*   accepting such terms or by downloading, installing, activating and/or otherwise
*   using the software, you are agreeing that you have read, and that you agree to
*   comply with and are bound by, such license terms. If you do not agree to be
*   bound by the applicable license terms, then you may not retain, install,
*   activate or otherwise use the software.
*
*   Revision History:
*
*   Version     Date          Author           Descriptions
*   ---------   ----------    ------------     ---------------
*   0.0.1       2026-3-17     iRobotTribe       First version;

*************************************************************************************************/

#ifndef _BSP_PWM_CURR_H_
#define _BSP_PWM_CURR_H_

#include "tim.h"


/* 母线电压 */
#define BUS_VOLTAGE            (12.0f)
/* 电压到PWM占空比转换系数，计算公式：(HALF_PWM_PERIOD_CYCLES / 母线电压) */
#define VOLTAGE_FACTOR_TO_PWM_DUTY  ((float) (HALF_PWM_PERIOD_CYCLES/BUS_VOLTAGE)) //4250/12=354.166666



static inline void set_a_voltage(float voltage)
{
  htim1.Instance->CCR1 = (voltage * VOLTAGE_FACTOR_TO_PWM_DUTY) + (HALF_PWM_PERIOD_CYCLES/2);//+2125
}

static inline void set_b_voltage(float voltage)
{
  htim1.Instance->CCR2 = (voltage * VOLTAGE_FACTOR_TO_PWM_DUTY) + (HALF_PWM_PERIOD_CYCLES/2);
}

static inline void set_c_voltage(float voltage)
{
  htim1.Instance->CCR3 = (voltage * VOLTAGE_FACTOR_TO_PWM_DUTY) + (HALF_PWM_PERIOD_CYCLES/2);
}



/* 提供给其他C文件调用的函数 */
void bsp_pwm_init(void);
void bsp_pwm_switch_on(void);
void bsp_pwm_switch_off(void);
uint8_t bsp_pwm_is_enable(void);
#endif

/************************************ iRobotTribe (END OF FILE) ************************************/
