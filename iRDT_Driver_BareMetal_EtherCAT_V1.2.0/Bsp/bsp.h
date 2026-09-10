/*************************************************************************************************
*   File Name            : bsp.h
*   File Descriptions    : board support package for all bsp module
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
#ifndef _BSP_H_
#define _BSP_H_


#define USE_FreeRTOS 0	/* 使能FreeRTOS */

/* 包含FreeRTOS头文件 */
#if USE_FreeRTOS == 1
	#include "FreeRTOS.h"
	#include "task.h"
	#include "event_groups.h"
	#include "queue.h"
	#include "semphr.h"
	#define DISABLE_INT()	taskENTER_CRITICAL() /* 禁止全局中断 */
	#define ENABLE_INT()	taskEXIT_CRITICAL()  /* 使能全局中断 */
#else
	#define DISABLE_INT()	__set_PRIMASK(1)	/* 禁止全局中断 */
	#define ENABLE_INT()	__set_PRIMASK(0)	/* 使能全局中断 */
#endif


#include "main.h"

/* 包含标准库头文件 */
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

#ifndef TRUE
	#define TRUE  1
#endif

#ifndef FALSE
	#define FALSE 0
#endif

/* 通过取消注释或者添加注释的方式控制是否包含底层驱动模块 */
#include "bsp_led.h"
#include "bsp_usart.h"
#include "bsp_spi_mt6701.h"
#include "bsp_key.h"
#include "bsp_adc.h"
#include "bsp_pwm.h"
#include "bsp_encoder.h"
#include "bsp_spi_lan9252.h"

/* 通过取消注释或者添加注释的方式控制是否包含第三方软件包 */
#include "cia402appl.h"
#include "FOC_Controller.h"

/* 提供给其他C文件调用的函数 */
void PrintfLogo(void);
#endif

/************************************ iRobotTribe (END OF FILE) ************************************/
