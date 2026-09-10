/*************************************************************************************************
*   File Name            : bsp_adc.h
*   File Descriptions    : board support package for led light
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

#ifndef _BSP_ADC_H_
#define _BSP_ADC_H_

#include "adc.h"


/* 采样电阻 */
#if (USER_HW_VERSION < 0x01040000)
#define SHUNT_RESISTENCE       (0.020f)
#elif (USER_HW_VERSION >= 0x01040000)
#define SHUNT_RESISTENCE       (0.005f)
#endif

#define AMPLIFIER_GAIN         (50.0f)
/* 电流采样到实际电流的转换系数，计算公式：(3.3V / 4095 / 采样电阻 / 运放增益) */
#define I_SCALE                ((float) ((3.3f / 4095.0f) / SHUNT_RESISTENCE / AMPLIFIER_GAIN))
#define U_SCALE                ((float) ((3.3f / 4095.0f) * 32.0f / 2.0f))    

typedef enum
{
	VOLT_PHASE_A = 0,
	VOLT_PHASE_B,
	VOLT_PHASE_C,
	NTC_MOTOR,
	VOLT_BUS,
	NTC_DRIVER,
}ANOLOG_INDEX_E;

/* 三相电流采样的偏移量 */
extern volatile int16_t  phase_a_adc_offset;
extern volatile int16_t  phase_b_adc_offset;
extern volatile int16_t  phase_c_adc_offset;

static inline float read_iphase_a(void)
{
  return (float)((uint16_t)hadc1.Instance->JDR1 - phase_a_adc_offset) * I_SCALE;
}

static inline float read_iphase_b(void)
{
  return (float)((uint16_t)hadc1.Instance->JDR2 - phase_b_adc_offset) * I_SCALE;
}

static inline float read_iphase_c(void)
{
  return (float)((uint16_t)hadc1.Instance->JDR3 - phase_c_adc_offset) * I_SCALE;
}

/* 供外部调用的函数声明 */
float bsp_adc_bus_voltage(void);
float bsp_adc_drv_temperature(void);
float bsp_adc_motor_temperature(void);
#endif

/************************************ iRobotTribe (END OF FILE) ************************************/
