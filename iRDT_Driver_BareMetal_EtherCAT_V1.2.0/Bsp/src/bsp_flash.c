/*************************************************************************************************
*   File Name            : bsp_flash.c
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

#include "bsp_flash.h"

/**
 * @brief 根据地址获取 Bank 号
 */
static uint32_t GetBank(uint32_t Address)
{
    if (Address < (FLASH_BASE + FLASH_BANK_SIZE)) {
        return FLASH_BANK_1;  // Bank 1
    } else {
        return FLASH_BANK_2;  // Bank 2
    }
}

/**
***********************************************************************
* @brief:      get_page(uint32_t addr)
* @param:	   flash address
* @retval:     page number
* @details:    get page number of input address
***********************************************************************
**/
static uint32_t GetPage(uint32_t Address)
{
    if (Address < (FLASH_BASE + FLASH_BANK_SIZE)) {
        // Bank 1: 页号 0-127
        return (Address - FLASH_BASE) / FLASH_PAGE_SIZE;
    } else {
        // Bank 2: 页号 0-127
        return (Address - (FLASH_BASE + FLASH_BANK_SIZE)) / FLASH_PAGE_SIZE;
    }
}

/**
 * @brief 擦除指定地址所在的 Flash 页
 * @param Address Flash 地址（如 0x0801C000）
 * @return 1=成功, 0=失败
 */
uint8_t bsp_flash_erase_page_by_addr(uint32_t Address)
{
    uint32_t page_err = 0;
    FLASH_EraseInitTypeDef EraseInit;
    
    HAL_FLASH_Unlock();
    
    // 清除所有错误标志（G4需要清除更多标志）
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_OPTVERR | FLASH_FLAG_EOP | 
                          FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR);
    
    /* 填充 EraseInit 结构体 - G4 特有字段 */
    EraseInit.TypeErase = FLASH_TYPEERASE_PAGES;
    EraseInit.Banks     = GetBank(Address);      // ⭐ 指定 Bank
    EraseInit.Page      = GetPage(Address);      // ⭐ Bank 内的页号
    EraseInit.NbPages   = 1;
    
    if (HAL_FLASHEx_Erase(&EraseInit, &page_err) != HAL_OK) {
        HAL_FLASH_Lock();
        return 0;
    }
    
    HAL_FLASH_Lock();
    return 1;
}

/**
***********************************************************************
* @brief:      bsp_flash_write_data(uint32_t addr, void *data, uint32_t size)
* @param:	   start address to be writen
* @param:      data buffer
* @param:	   size
* @retval:     void
* @details:    write data to flash
***********************************************************************
**/
void bsp_flash_write_data(uint32_t addr, void *data, uint32_t size)
{
	uint64_t *buffer = (uint64_t *)data;
	uint32_t temp_addr = addr;
	
	HAL_FLASH_Unlock();
	
	for (uint32_t i=0; i<size; i+=8)
	{
		HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, temp_addr + i, *buffer);
		buffer++;
	}
	
	HAL_FLASH_Lock();
}

/**
***********************************************************************
* @brief:      bsp_flash_write_canid(uint8_t can_id)
* @param:	   can id to be saved
* @retval:     void
* @details:    save can id to flash
***********************************************************************
**/
void bsp_flash_write_canid(uint8_t can_id)
{
    __disable_irq();
    bsp_flash_erase_page_by_addr(ADDR_FLASH_PAGE_63);
    bsp_flash_write_data(ADDR_FLASH_PAGE_63, &can_id, 1);//64bit
    __enable_irq();
}

/**
***********************************************************************
* @brief:      bsp_flash_read_canid(void)
* @param:	   can id to be saved
* @retval:     void
* @details:    save can id to flash
***********************************************************************
**/
uint8_t bsp_flash_read_canid(void)
{
    return (*(volatile uint8*)(ADDR_FLASH_PAGE_63));
}
/************************************ iRobotTribe (END OF FILE) ************************************/
