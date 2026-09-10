/*************************************************************************************************
*   File Name            : bsp_usart.c
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
#include "bsp.h"
#include "usart.h"


volatile struct Frame vofaFrame = 
{
	.tail = {0x00, 0x00, 0x80, 0x7f}
};

/*
*********************************************************************************************************
*	函 数 名: debug_print
*	功能说明: 格式化并通过UART发送调试信息。该函数接受一个格式化字符串和可变数量的参数，
*            将它们格式化为一个字符串，然后通过UART发送出去。
*	形    参：
*           fmt: 格式化字符串，类似于printf的格式。
*           ...: 可变数量的参数，与fmt中的格式说明符对应。
*	返 回 值: 无
* 注意事项：传入参数不要超过255个字节，否则会导致数据丢失。
*						本函数进行了线程安全保护。(不要在中断调用本函数)
*********************************************************************************************************
*/
void debug_print(const char *const fmt, ...)
{
	char str[256];
	uint16_t len;
	va_list args;
	
	memset((char *)str, 0, sizeof(char)*256); 
	va_start(args, fmt);
	vsnprintf((char *)str, 255, (char const *)fmt, args);
	va_end(args);
	
	len = strlen((char *)str);  
    HAL_UART_Transmit_DMA(&huart3, (uint8_t*)str, len);
    while (huart3.gState != HAL_UART_STATE_READY) {}// 等待UART发送完成，状态变成空闲

}

/*
*********************************************************************************************************
*	函 数 名: vofa_print
*	功能说明: 发送vfoa数据帧
*	形    参：无
*	返 回 值: 无
*********************************************************************************************************
*/
void vofa_print(void)
{
    vofaFrame.fdata[0] = FOC_iPhaseAMeas_S;
    vofaFrame.fdata[1] = FOC_iPhaseBMeas_S;
    vofaFrame.fdata[2] = FOC_iPhaseCMeas_S;
    vofaFrame.fdata[3] = SVM_uPhaseU_S;
    vofaFrame.fdata[4] = SVM_uPhaseV_S;
    vofaFrame.fdata[5] = SVM_uPhaseW_S;
	HAL_UART_Transmit_DMA(&huart3, (uint8_t *) (&vofaFrame), sizeof(vofaFrame));
}


/************************************ iRobotTribe (END OF FILE) ************************************/
