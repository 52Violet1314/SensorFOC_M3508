/*************************************************************************************************
*   File Name            : bsp_encoder.c
*   File Descriptions    : low-level driver for motor encoder
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
#include "bsp_spi_mt6701.h"
#include "bsp_encoder.h"

//tEncoder   Encoder;

EncoderIF_Typedef Encoder = 
{
    .direction=0,
    .accum_pulse=0.0f,
    .position_raw=0.0f,
    .position_filted=0.0f,
    .position_filter_factor=10.0f,
    .speed_rpm_raw=0.0f,
    .speed_rpm_filted=0.0f,
    .speed_filter_factor=10.0f
};

float bsp_filter_co(float input,float filtertime,float output_last)
{
  static float tempfactor;
	static float output;
	
  tempfactor = 1.0F / (filtertime + 1.0F);
  output = (1.0F - tempfactor) * output_last + tempfactor * input;
	return output;
}
/*
*********************************************************************************************************
*	函 数 名: encoder_angle_normalize
*	功能说明: 获取编码器角度,0-65535
*	形    参: 无
*	返 回 值: 角度值
*********************************************************************************************************
*/

//angle range from 0-65535
uint16_t encoder_normalize_angle(void)
{	
    uint32_t temp_cnt = bsp_spi_mt6701_raw_cnt()*Encoder.direction;
    return (uint16_t)(temp_cnt*M_POLE_PAIRS*65535/MT6701_COUNTER_MAX);
}
/*
*********************************************************************************************************
*	函 数 名: encoder_calc_speed
*	功能说明: 计算编码器转速
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
float encoder_calc_speed(void)
{	
    int32_t temp_pusle,temp_period;
    
    bsp_spi_mt6701_get_sum(&temp_pusle, &temp_period);
    /*(deltaPulse/16383)/(deltaPeriod/10^6)(round/s)-->*60(rpm)*/
    Encoder.accum_pulse += temp_pusle;
	Encoder.speed_rpm_raw = ((temp_pusle*3662.332906F)/temp_period);
    bsp_spi_mt6701_clear_sum();
    
	//Encoder.speed_rpm_filted = bsp_filter_co(Encoder.speed_rpm_raw,Encoder.speed_filter_factor,Encoder.speed_rpm_filted);
    Encoder.speed_rpm_filted = Encoder.speed_rpm_raw;

    return (Encoder.direction>0? -Encoder.speed_rpm_filted : Encoder.speed_rpm_filted);
}
/*
*********************************************************************************************************
*	函 数 名: encoder_get_angle
*	功能说明: 获取编码器多圈角度
*	形    参: 无
*	返 回 值: 返回编码器的多圈角度值，单位为度
*********************************************************************************************************
*/
float encoder_get_angle(void)
{
    Encoder.position_raw = Encoder.accum_pulse*360.0f/16384.0f;
    Encoder.position_filted = bsp_filter_co(Encoder.position_raw,Encoder.position_filter_factor,Encoder.position_filted);
    return (Encoder.direction>0? -Encoder.position_filted : Encoder.position_filted);
}
/*
*********************************************************************************************************
*	函 数 名: encoder_set_angle
*	功能说明: 设置编码器多圈角度
*	形    参: 角度值
*	返 回 值: 无
*********************************************************************************************************
*/
void encoder_set_angle(float angle)
{
    Encoder.accum_pulse = angle/365.0f *16384.0f;
}
/*
*********************************************************************************************************
*	函 数 名: encoder_set_angle
*	功能说明: 设置编码器多圈角度
*	形    参: 无
*	返 回 值: 无，FOC初始化后调用，请勿再中断中调用
*********************************************************************************************************
*/
void encoder_calibration(void)
{
    int32_t first_pusle,second_pusle,temp_period;
    
    bsp_pwm_switch_on();
    
    Motor_swtMode_C=0;
    FOC_uManualVq_C=0.0f;
    for (int i = 0; i <=65535; i++ )//360*2/7 =102degree
    {
        if(FOC_uManualVq_C<FOC_uFindPosVd_C)
        {
            FOC_uManualVq_C+=0.00005f;//防抖动
        }
        FOC_cntThetaElec_S += 2;
        CurrentControl_Co();	
        set_a_voltage(SVM_uPhaseU_S);
        set_b_voltage(SVM_uPhaseV_S);
        set_c_voltage(SVM_uPhaseW_S);  
        bsp_spi_mt6701_update(0);//update mt6701
    }
    bsp_spi_mt6701_get_sum(&first_pusle, &temp_period);
    
    FOC_uManualVq_C=0.0f; 
    bsp_pwm_switch_off();
    HAL_Delay(500);
    bsp_pwm_switch_on();

    for (int i = 65535; i >=0; i-- )
    {
        if(FOC_uManualVq_C<FOC_uFindPosVd_C)
        {
            FOC_uManualVq_C+=0.00005f;
        }
        FOC_cntThetaElec_S -= 2;
        CurrentControl_Co();
        set_a_voltage(SVM_uPhaseU_S);
        set_b_voltage(SVM_uPhaseV_S);
        set_c_voltage(SVM_uPhaseW_S);  
        bsp_spi_mt6701_update(0);//update mt6701
    }
    bsp_spi_mt6701_get_sum(&second_pusle, &temp_period);
    
    //determine the direction the sensor moved,depend on the motor install and UVW wiring
    if (first_pusle == second_pusle)
    {
        debug_print("Failed to move the motor\r\n");
    }
    else if (first_pusle < second_pusle)
    {
        debug_print("Encoder direction==CCW\r\n");
        Encoder.direction = -1; // 逆时针
    }
    else
    {
        debug_print("Encoder direction==CW\r\n");
        Encoder.direction = 1; // 顺时针
    }
    FOC_uManualVq_C=0.0f;
    bsp_pwm_switch_off();
}

/************************************ iRobotTribe (END OF FILE) ************************************/

