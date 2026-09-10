/*************************************************************************************************
*   File Name            : bsp_pwm.c
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
#include "bsp_pwm.h"

extern ADC_HandleTypeDef hadc1;
uint8_t motor_enable_flag = 0;
/*
*********************************************************************************************************
*	函 数 名: PWM_Cur_Init
*	功能说明: PWM和电流采样初始化
*	形    参：无
*	返 回 值: 无
*********************************************************************************************************
*/
void bsp_pwm_init(void)
{
	/* ADC1 calibration */
	HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
	HAL_ADCEx_InjectedStart_IT(&hadc1);

	/* Set all duty to 50% */
	htim1.Instance->CCR1 = ((uint32_t) HALF_PWM_PERIOD_CYCLES / (uint32_t) 2);
	htim1.Instance->CCR2 = ((uint32_t) HALF_PWM_PERIOD_CYCLES / (uint32_t) 2);
	htim1.Instance->CCR3 = ((uint32_t) HALF_PWM_PERIOD_CYCLES / (uint32_t) 2);
	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
	HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_1);
	HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_2);
	HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_3);
	
    /* 定时器通道4触发ADC采样 */
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
    motor_enable_flag = 0;
}

/*
*********************************************************************************************************
*	函 数 名: bsp_pwm_switch_on
*	功能说明: 打开PWM输出
*	形    参：无
*	返 回 值: 无
*********************************************************************************************************
*/
void bsp_pwm_switch_on(void)
{
	/* Set all duty to 50% */
	htim1.Instance->CCR1 = ((uint32_t) HALF_PWM_PERIOD_CYCLES / (uint32_t) 2);
	htim1.Instance->CCR2 = ((uint32_t) HALF_PWM_PERIOD_CYCLES / (uint32_t) 2);
	htim1.Instance->CCR3 = ((uint32_t) HALF_PWM_PERIOD_CYCLES / (uint32_t) 2);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
	HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
	HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
    motor_enable_flag = 1;
}

/*
*********************************************************************************************************
*	函 数 名: bsp_pwm_switch_off
*	功能说明: 关闭PWM输出
*	形    参：无
*	返 回 值: 无
*********************************************************************************************************
*/
void bsp_pwm_switch_off(void)
{
	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
	HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_1);
	HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_2);
	HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_3);
    motor_enable_flag = 0;
}

/*
*********************************************************************************************************
*	函 数 名: bsp_pwm_is_enable
*	功能说明: 查询pwm使能状态
*	形    参：无
*	返 回 值: 1=使能，0=关闭
*********************************************************************************************************
*/
uint8_t bsp_pwm_is_enable(void)
{
    return motor_enable_flag;
}

/************************************ iRobotTribe (END OF FILE) ************************************/
