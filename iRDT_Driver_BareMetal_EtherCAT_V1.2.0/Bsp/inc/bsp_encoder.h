/*************************************************************************************************
*   File Name            : bsp_encoder.h
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

#ifndef _BSP_ENCODER_H_
#define _BSP_ENCODER_H_

#define M_PI		        (3.14159265358f)
#define M_POLE_PAIRS        (7U)
typedef struct 
{
    int16_t   direction;
    float     accum_pulse;						    // accumulated pulse 
    float     position_raw;						    // accumulated angle in degree
    float     position_filted;						// accumulated angle after filter
    float     position_filter_factor;				// factor by lowpass filter
	float     speed_rpm_raw;						// raw speed in rpm
    float     speed_rpm_filted;						// filted speed in rpm
    float     speed_filter_factor;					// factor by lowpass filter
}EncoderIF_Typedef;

//typedef struct sEncoder
//{
//  uint8_t offline;            // 编码器是否离线
//  float angle;                // 当前的机械角度值(不带圈数)，以弧度为单位
//  int32_t full_rotations; 	  // 累计的完整旋转次数，表示编码器已经完整旋转了多少次
//  int32_t vel_full_rotations; // 速度相关的完整旋转次数，用于计算编码器的旋转速度
//  float angle_prev; 		  // 上一次测量的角度值，用于计算角度变化
//  uint32_t angle_prev_ts;     // 上一次测量角度的时间戳，用于计算采样时间
//  float vel_angle_prev; 	  // 上一次速度测量的角度值，用于计算速度
//  uint32_t vel_angle_prev_ts; // 上一次速度测量的时间戳，用于计算速度采样时间

//} tEncoder;


/* 全局变量 */
extern EncoderIF_Typedef Encoder;

/* 提供给其他C文件调用的函数 */
float bsp_filter_co(float input,float filtertime,float output_last);
uint16_t encoder_normalize_angle(void);
float encoder_calc_speed(void);
float encoder_get_angle(void);
void encoder_set_angle(float angle);
void encoder_calibration(void);

#endif

/************************************ iRobotTribe (END OF FILE) ************************************/
