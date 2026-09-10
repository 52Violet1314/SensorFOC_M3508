/*************************************************************************************************
*   File Name            : bsp_spi_mt6701.c
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
#include "bsp_spi_mt6701.h"
#include "spi.h"

#define MT6701_CS_LOW()					GPIOD->BSRR = ((uint32_t)GPIO_PIN_2 << 16U) 
#define MT6701_CS_HIGH()				GPIOD->BSRR = GPIO_PIN_2
#define MT6701_SPI_TIMEOUT_MS			2U
#define MT6701_CRC_MASK				0x3FU
#define MT6701_ANGLE_MASK				0x3FFFU

volatile SpiSensor_Typedef mt6701_sensor = {0};
static uint8_t mt6701_dma_frame[3];
static volatile uint8_t mt6701_dma_busy;
static volatile uint8_t mt6701_dma_ready;
static volatile uint8_t mt6701_dma_valid;
/*
*********************************************************************************************************
 * 函 数 名: calculate_crc
 * 功能说明: 校验读取的原始数据是否正确。
 * 形    参: data: 原始数据。
 * 返 回 值: crc计算结果。
 * 说    明: 单次输入6bit数据。
*********************************************************************************************************
*/
uint8_t calculate_crc(uint32_t data)
{
  uint8_t crc = 0;
  uint32_t polynomial = 0x43; // (X^6 + X + 1)多项式

  // (D[13:0] 和 Mg[3:0])
  for (int i = 17; i >= 0; i--)
  {
    uint8_t bit = (data >> i) & 1;
    crc <<= 1;
    if ((crc >> 6) ^ bit)
    {
      crc ^= polynomial;
    }
    crc &= 0x3F;//6bit crc
  }
  return crc;
}
/*
*********************************************************************************************************
 * 函 数 名: bsp_spi_mt6701_get
 * 功能说明: 通过SPI从MT6701获取角度数据。
 * 形    参: none。
 * 返 回 值: 0: 成功，1: 失败。
 * 说    明: 读取数据并进行CRC校验，最多尝试3次。校验失败返回错误。
*********************************************************************************************************
*/
static uint8_t bsp_spi_mt6701_get(void)
{
	uint8_t data_r[3] = {0U};
	uint32_t raw;
	HAL_StatusTypeDef status;

	for (uint8_t attempt = 0U; attempt < 3U; attempt++)
	{
		/* CS must be released on every exit path, including SPI errors. */
		MT6701_CS_LOW();
		status = HAL_SPI_Receive(&hspi1, data_r, 3U, MT6701_SPI_TIMEOUT_MS);
		MT6701_CS_HIGH();
		if (status != HAL_OK)
		{
			return 1U;
		}

		raw = ((uint32_t)data_r[0] << 16U) |
		      ((uint32_t)data_r[1] << 8U) | (uint32_t)data_r[2];
		if (raw == 0U)
		{
			return 1U;
		}

		/* MT6701 CRC covers angle/status bits [23:6]. */
		if (calculate_crc(raw >> 6U) == (uint8_t)(raw & MT6701_CRC_MASK))
		{
			mt6701_sensor.data_raw.value = raw;
			return 0U;
		}
	}

	/* Do not report success after three invalid frames. */
	return 1U;
}


void bsp_spi_mt6701_update(uint32_t ts_us)
{
    int32_t  cnt_diff = 0;
    uint16_t cnt_now = 0U;
    uint8_t sync_mode = (ts_us == 0U) ? 1U : 0U;

    /* Calibration passes zero period and requires an immediate sample. */
    if (sync_mode != 0U)
    {
        mt6701_dma_valid = (bsp_spi_mt6701_get() == 0U) ? 1U : 0U;
        mt6701_dma_ready = 1U;
    }
    
    /* Consume the previous DMA frame, then launch the next one. */
    if (mt6701_dma_ready != 0U)
    {
        mt6701_dma_ready = 0U;
    if(mt6701_dma_valid != 0U)
    {
        mt6701_sensor.error_raw = 0;//no error
        /* Decode the same explicit 14-bit field as the G474 driver. */
        cnt_now = (uint16_t)((mt6701_sensor.data_raw.value >> 10U) &
                             MT6701_ANGLE_MASK);
        cnt_diff = cnt_now - (int32_t)((mt6701_sensor.data_last.value >> 10U) &
                                       MT6701_ANGLE_MASK);
        if(cnt_diff > (int32_t)MT6701_COUNTER_HALF)
        {
            cnt_diff -= MT6701_COUNTER_MAX;
        }
        else if(cnt_diff < -(int32_t)MT6701_COUNTER_HALF)
        {
            cnt_diff += MT6701_COUNTER_MAX;
        }
        mt6701_sensor.data_last = mt6701_sensor.data_raw;
        
        mt6701_sensor.pulse_sum += cnt_diff;
        mt6701_sensor.period_sum += ts_us; //function called in 50us
        mt6701_sensor.elec_angle = ((cnt_now * M_POLE_PAIRS) % MT6701_COUNTER_MAX) *
                                   (360.0F / (float)MT6701_COUNTER_MAX);
		mt6701_sensor.mech_angle = cnt_now * (360.0F / (float)MT6701_COUNTER_MAX);	
    }
    else
    {
        mt6701_sensor.error_raw = 1;//error
    };
    }
    if ((sync_mode == 0U) && (mt6701_dma_busy == 0U))
    {
        MT6701_CS_LOW();
        if (HAL_SPI_Receive_DMA(&hspi1, mt6701_dma_frame, 3U) == HAL_OK)
        {
            mt6701_dma_busy = 1U;
        }
        else
        {
            MT6701_CS_HIGH();
        }
    }
    
    //encoder error check
    if(mt6701_sensor.error_raw != mt6701_sensor.error_valid)
    {
        if(mt6701_sensor.error_filter_cnt > 0)
        {
            mt6701_sensor.error_filter_cnt--;
        }
        else
        {
            mt6701_sensor.error_valid = mt6701_sensor.error_raw;
        }
    }
    else
    {
        mt6701_sensor.error_filter_cnt = 5;
    }
}
uint32_t bsp_spi_mt6701_raw_cnt(void)
{
    return (mt6701_sensor.data_raw.value >> 10U) & MT6701_ANGLE_MASK;
}

void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
{
    uint32_t raw;
    if ((hspi != &hspi1) || (mt6701_dma_busy == 0U))
    {
        return;
    }
    MT6701_CS_HIGH();
    mt6701_dma_busy = 0U;
    raw = ((uint32_t)mt6701_dma_frame[0] << 16U) |
          ((uint32_t)mt6701_dma_frame[1] << 8U) | mt6701_dma_frame[2];
    mt6701_dma_valid = (raw != 0U) &&
                       (calculate_crc(raw >> 6U) == (uint8_t)(raw & MT6701_CRC_MASK));
    if (mt6701_dma_valid != 0U)
    {
        mt6701_sensor.data_raw.value = raw;
    }
    mt6701_dma_ready = 1U;
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
    if ((hspi == &hspi1) && (mt6701_dma_busy != 0U))
    {
        MT6701_CS_HIGH();
        mt6701_dma_busy = 0U;
        mt6701_dma_valid = 0U;
        mt6701_dma_ready = 1U;
    }
}

float bsp_spi_mt6701_elec_angle(void)
{
    return mt6701_sensor.elec_angle;
}

float bsp_spi_mt6701_mech_angle(void)
{
    return mt6701_sensor.mech_angle;
}

void bsp_spi_mt6701_get_sum(int32_t *pulse, int32_t *period)
{
    *pulse = mt6701_sensor.pulse_sum;
    *period = mt6701_sensor.period_sum;
}
void bsp_spi_mt6701_clear_sum(void)
{
	mt6701_sensor.pulse_sum = 0U;//set to zero after calc speed
	mt6701_sensor.period_sum = 1U;//set to 1 for divide 0
}

/************************************ iRobotTribe (END OF FILE) ************************************/
