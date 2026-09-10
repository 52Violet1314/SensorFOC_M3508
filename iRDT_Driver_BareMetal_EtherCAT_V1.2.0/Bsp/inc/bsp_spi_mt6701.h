/*************************************************************************************************
*   File Name            : bsp_spi_mt6701.h
*   File Descriptions    : board support package for mt6701 motor encoder
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
#ifndef __BSP_SPI_MT6701_H
#define __BSP_SPI_MT6701_H

#include "bsp.h"

#define MT6701_RESOLUTION_BITS  (14U)
#define MT6701_COUNTER_MAX      (1U<<MT6701_RESOLUTION_BITS)
#define MT6701_COUNTER_HALF     (MT6701_COUNTER_MAX>>1U)

typedef union
{
	uint32_t value;
	struct
	{
		uint8_t  crc_result:6;      //[0 5]
        uint8_t  magnet_strength:2; //[6 7]
        uint8_t  magnet_press:1;    //[8 8] pressed
        uint8_t  magnet_ovs:1;      //[9 9] over-speed
        uint32_t angle_data:14;     //[10 23]
		uint8_t  reserved:8;        //[24 31]
	}bitfield;
}MT6701_DataType;

typedef struct 
{
    MT6701_DataType  data_raw;  // SPI raw data
    MT6701_DataType  data_last;
    
    uint8_t   error_raw;						// error flag (raw)
    uint8_t   error_valid;						// error flag (filted)
    uint16_t  error_filter_cnt;  				// error filter counter
  
    int32_t   pulse_sum;             //sum of pulse number for speed calculation
    int32_t   period_sum;                //sum of time for speed calculation in us
    
	float     elec_angle;				// electrical angle of motor in degree
	float     mech_angle;				// mechanical angle of motor in degree
}SpiSensor_Typedef;

void bsp_spi_mt6701_update(uint32_t ts_us);
uint32_t bsp_spi_mt6701_raw_cnt(void);
float bsp_spi_mt6701_elec_angle(void);
float bsp_spi_mt6701_mech_angle(void);
void bsp_spi_mt6701_get_sum(int32_t *pulse, int32_t *period);
void bsp_spi_mt6701_clear_sum(void);
#endif
