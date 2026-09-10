/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    fdcan.c
  * @brief   This file provides code for the configuration
  *          of the SPI instances.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "fdcan.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */
FDCAN_HandleTypeDef hfdcan2;


static void Config_ClockDivider(FDCAN_HandleTypeDef *hfdcan)
{
    if ((hfdcan->Instance == FDCAN2)||(hfdcan->Instance == FDCAN3))
    {
        __HAL_RCC_FDCAN_CLK_ENABLE();
        /*Note: CKDIV can only be set by FDCAN1 in HAL Library*/
        /* Request initialisation */
        SET_BIT(FDCAN1->CCCR, FDCAN_CCCR_INIT);
        while ((FDCAN1->CCCR & FDCAN_CCCR_INIT) == 0);
        
        SET_BIT(FDCAN1->CCCR, FDCAN_CCCR_CCE);
        /* Configure Clock divider */
        FDCAN_CONFIG->CKDIV = hfdcan->Init.ClockDivider;
        if (FDCAN_CONFIG->CKDIV != hfdcan->Init.ClockDivider)
        {
            //printf("CKDIV write failed! Current: %lu\n", FDCAN_CONFIG->CKDIV);
            Error_Handler();
        }
        CLEAR_BIT(FDCAN1->CCCR, FDCAN_CCCR_INIT);
        while ((FDCAN1->CCCR & FDCAN_CCCR_INIT) != 0);
    }
}

/* CAN2 init function */
void MX_FDCAN2_Init(void)
{
    /* USER CODE BEGIN FDCAN2_Init 0 */

    /* USER CODE END FDCAN2_Init 0 */

    /* USER CODE BEGIN FDCAN2_Init 1 */

    /* USER CODE END FDCAN2_Init 1 */
    hfdcan2.Instance = FDCAN2;
    hfdcan2.Init.ClockDivider = FDCAN_CLOCK_DIV2;// 170/2=85MHz 
    hfdcan2.Init.FrameFormat = FDCAN_FRAME_CLASSIC;						//FDCAN_FRAME_FD_BRS
    hfdcan2.Init.Mode = FDCAN_MODE_NORMAL;
    hfdcan2.Init.AutoRetransmission = DISABLE;							//自动重发功能
    hfdcan2.Init.TransmitPause = DISABLE;
    hfdcan2.Init.ProtocolException = DISABLE;
    
    //Tq=85MHz/NominalPrescaler=85/5=17MHz
    //500kbps=17MHz/34
    //34=1(SYNC)+20(Tseg1)+13(Tseg2)
    hfdcan2.Init.NominalPrescaler = 5;									//1~512正常部分波特率配置
    hfdcan2.Init.NominalSyncJumpWidth = 1;								//1~128
    hfdcan2.Init.NominalTimeSeg1 = 20;										//2~256
    hfdcan2.Init.NominalTimeSeg2 = 13;										//2~128

    hfdcan2.Init.DataPrescaler = 1;										//1~32 数据部分波特率配置
    hfdcan2.Init.DataSyncJumpWidth = 1;									//1~16
    hfdcan2.Init.DataTimeSeg1 = 1;										//1~32
    hfdcan2.Init.DataTimeSeg2 = 1;										//1~16
    
    //message RAM configuration
    hfdcan2.Init.StdFiltersNbr = 28;									//0~28 标准帧过滤器个数
    hfdcan2.Init.ExtFiltersNbr = 8;										//0~8  扩展帧过滤器个数
    hfdcan2.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
    
    Config_ClockDivider(&hfdcan2);
    
    if (HAL_FDCAN_Init(&hfdcan2) != HAL_OK)
    {
        Error_Handler();
    }

    //滤波器配置
    FDCAN_FilterTypeDef  sFilterConfig = {0};
    sFilterConfig.IdType = FDCAN_STANDARD_ID;
    sFilterConfig.FilterIndex = 0; 
    sFilterConfig.FilterType = FDCAN_FILTER_RANGE;//Range filter from FilterID1 to FilterID2 
    sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    sFilterConfig.FilterID1 = 0x00;
    sFilterConfig.FilterID2 = 0x7FF;
    if(HAL_FDCAN_ConfigFilter(&hfdcan2, &sFilterConfig) != HAL_OK)
    {
        Error_Handler();
    }
	//滤波策略
	if(HAL_FDCAN_ConfigGlobalFilter(&hfdcan2, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_REJECT_REMOTE) != HAL_OK)
    {
        Error_Handler();
    }

    //start CAN and interrupt
	HAL_FDCAN_Start(&hfdcan2);
    HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);

    /* USER CODE BEGIN FDCAN2_Init 2 */
    /* USER CODE END FDCAN2_Init 2 */
}



void HAL_FDCAN_MspInit(FDCAN_HandleTypeDef* canHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};
  
  if(canHandle->Instance==FDCAN2)
  {
    /** Initializes the peripherals clocks  */
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_FDCAN;
    PeriphClkInit.FdcanClockSelection = RCC_FDCANCLKSOURCE_PCLK1;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
    {
      Error_Handler();
    }

    /* Peripheral clock enable */
    __HAL_RCC_FDCAN_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    
    /**FDCAN2 GPIO Configuration    
    PB5     ------> CAN_RX
    PB6     <------ CAN_TX 
    */
    GPIO_InitStruct.Pin = GPIO_PIN_6;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Alternate=GPIO_AF9_FDCAN2;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_5;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate=GPIO_AF9_FDCAN2;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    
    /* 初始化FDCAN2中断优先级 */		
	HAL_NVIC_SetPriority(FDCAN2_IT0_IRQn, 6, 0);
    HAL_NVIC_EnableIRQ(FDCAN2_IT0_IRQn);
  }
}

void HAL_FDCAN_MspDeInit(FDCAN_HandleTypeDef* canHandle)
{

  if(canHandle->Instance==FDCAN2)
  {
    /* USER CODE BEGIN FDCAN2 0 */

    /* USER CODE END FDCAN2 0 */
    /* Peripheral clock disable */
    __HAL_RCC_FDCAN_CLK_DISABLE();

    /**FDCAN2 GPIO Configuration    
    PB5     ------> CAN_RX
    PB6     <------ CAN_TX 
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_5|GPIO_PIN_6);

    /* USER CODE BEGIN FDCAN2_MspDeInit 1 */
    /* Peripheral interrupt Deinit*/
    HAL_NVIC_DisableIRQ(FDCAN2_IT0_IRQn);
    /* USER CODE END FDCAN2_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
