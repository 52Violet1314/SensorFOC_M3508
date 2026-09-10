/*************************************************************************************************
*   File Name            : bsp_adc.c
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

#include "bsp_adc.h"
#include <math.h>

volatile int16_t phase_a_adc_offset = 0;
volatile int16_t phase_b_adc_offset = 0;
volatile int16_t phase_c_adc_offset = 0;


/**
***********************************************************************
* @brief:      bsp_avg_filter(ANOLOG_INDEX_E channel)
* @param[in]:  channel adc通道
* @retval:     value with filted
* @details:    均值滤波
***********************************************************************
**/
uint16_t bsp_avg_filter(ANOLOG_INDEX_E channel)
{
    uint16_t i, temp = 0U;
    for (i=0; i<ANOLOG_SAMPLES; i++)
    {
        temp += adc2_dma_value[i][channel];
    }
    return (temp/ANOLOG_SAMPLES);
}
/*
*********************************************************************************************************
*	函 数 名: bsp_adc_bus_voltage
*	功能说明: 获取母线电压值。
*	形    参:  none
*	返 回 值: 单位V
*********************************************************************************************************
*/
float bsp_adc_bus_voltage(void)
{
    uint16_t adcValue;
    adcValue = bsp_avg_filter(VOLT_BUS);
    return adcValue * U_SCALE;//((volt*3.3/4095)32/2)
}

/*
*********************************************************************************************************
*	函 数 名: bsp_adc_drv_temperature
*	功能说明: 获取驱动板温度。
*	形    参:  none
*	返 回 值: 
*********************************************************************************************************
*/
float bsp_adc_drv_temperature(void)
{
    uint16_t adcValue;
    float R_ntc;
    adcValue = bsp_avg_filter(NTC_DRIVER);
    if (adcValue > 4095) 
    {
        adcValue = 4095;
    }
    
    // 分压计算: R_ntc = R_pullup * (ADC / (4095 - ADC))
    R_ntc = 10000.0f * ((float)adcValue / (4095 - adcValue));
        // 防止对数计算错误
    if (R_ntc > 0) 
    {
        // B 值方程: T = 1 / (1/T25 + (1/B) * ln(R/R25))
        float steinhart = logf(R_ntc / 10000.0f) / 3380.0f;
        steinhart += 1.0f / 298.15f;
        float temperatureK = 1.0f / steinhart;
        return temperatureK - 273.15f; // 转换为摄氏度
    }
    else
    {
        return -999.9f;//error
    }
    
}

/*
*********************************************************************************************************
*	函 数 名: bsp_adc_motor_temperature
*	功能说明: 获取电机温度。
*	形    参:  none
*	返 回 值: 
*********************************************************************************************************
*/
float bsp_adc_motor_temperature(void)
{
    return bsp_avg_filter(NTC_MOTOR);
}



/************************************ iRobotTribe (END OF FILE) ************************************/
