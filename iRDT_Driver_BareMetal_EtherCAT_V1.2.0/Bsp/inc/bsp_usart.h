/*************************************************************************************************
*   File Name            : bsp_usart.h
*   File Descriptions    : board support package for usart
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

#ifndef _BSP_USART_H_
#define _BSP_USART_H_

#define CH_COUNT 6
struct Frame
{
	float fdata[CH_COUNT];
	unsigned char tail[4];
};
extern volatile struct Frame vofaFrame;


/* 供外部调用的函数声明 */
void debug_print(const char *const fmt, ...);
void vofa_print(void);

#endif

/************************************ iRobotTribe (END OF FILE) ************************************/
