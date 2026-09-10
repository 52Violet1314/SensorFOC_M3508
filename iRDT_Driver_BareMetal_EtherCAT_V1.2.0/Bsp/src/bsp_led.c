/*************************************************************************************************
*   File Name            : bsp_led.c
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

#include "bsp_led.h"

/*
*********************************************************************************************************
*	函 数 名: bsp_LedOn
*	功能说明: 点亮指定的LED指示灯。
*	形    参:  _no : 指示灯序号，范围 1 - 3
*	返 回 值: 无
*********************************************************************************************************
*/
void bsp_LedOn(uint8_t _no)
{
	if (_no == 1)
	{
		HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
	}
	else if (_no == 2)
	{
		HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);
	}
	else if (_no == 3)
	{
		HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_SET);
	}
}

/*
*********************************************************************************************************
*	函 数 名: bsp_LedOff
*	功能说明: 熄灭指定的LED指示灯。
*	形    参:  _no : 指示灯序号，范围 1 - 3
*	返 回 值: 无
*********************************************************************************************************
*/
void bsp_LedOff(uint8_t _no)
{
	if (_no == 1)
	{
		HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
	}
	else if (_no == 2)
	{
		HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);
	}
	else if (_no == 3)
	{
		HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_RESET);
	}
}

/*
*********************************************************************************************************
*	函 数 名: bsp_LedToggle
*	功能说明: 翻转指定的LED指示灯。
*	形    参:  _no : 指示灯序号，范围 1 - 3
*	返 回 值: 按键代码
*********************************************************************************************************
*/
void bsp_LedToggle(uint8_t _no)
{
	if (_no == 1)
	{
		HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
	}
	else if (_no == 2)
	{
		HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);
	}
	else if (_no == 3)
	{
		HAL_GPIO_TogglePin(LED3_GPIO_Port, LED3_Pin);
	}
}

/*
*********************************************************************************************************
*	函 数 名: bsp_IsLedOn
*	功能说明: 判断LED指示灯是否已经点亮。
*	形    参:  _no : 指示灯序号，范围 1 - 3
*	返 回 值: 1表示已经点亮，0表示未点亮
*********************************************************************************************************
*/
uint8_t bsp_IsLedOn(uint8_t _no)
{
	if (_no == 1)
	{
		if (HAL_GPIO_ReadPin(LED1_GPIO_Port, LED1_Pin))
		{
			return 1;
		}
		else
		{
			return 0;
		}
	}
	else if (_no == 2)
	{
		if (HAL_GPIO_ReadPin(LED2_GPIO_Port, LED2_Pin))
		{
			return 1;
		}
		else
		{
			return 0;
		}
	}
	else if (_no == 3)
	{
		if (HAL_GPIO_ReadPin(LED3_GPIO_Port, LED3_Pin))
		{
			return 1;
		}
		else
		{
			return 0;
		}
	}
	else
		return 0;
}

/************************************ iRobotTribe (END OF FILE) ************************************/
