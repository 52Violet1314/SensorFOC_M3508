/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
extern float Encoder_Angle;
extern float Encoder_Elec_Angle;   /* 电角度 rad, 编码器直接输出 */
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define IA_Pin GPIO_PIN_0
#define IA_GPIO_Port GPIOC
#define IB_Pin GPIO_PIN_1
#define IB_GPIO_Port GPIOC
#define IC_Pin GPIO_PIN_2
#define IC_GPIO_Port GPIOC
#define Power_DC_Pin GPIO_PIN_3
#define Power_DC_GPIO_Port GPIOC
#define VA_Pin GPIO_PIN_0
#define VA_GPIO_Port GPIOA
#define VB_Pin GPIO_PIN_1
#define VB_GPIO_Port GPIOA
#define VC_Pin GPIO_PIN_2
#define VC_GPIO_Port GPIOA
#define ADC_SIN_Pin GPIO_PIN_5
#define ADC_SIN_GPIO_Port GPIOA
#define ADC_COS_Pin GPIO_PIN_6
#define ADC_COS_GPIO_Port GPIOA
#define Temp_Motor_Pin GPIO_PIN_4
#define Temp_Motor_GPIO_Port GPIOC
#define LED_Green_Pin GPIO_PIN_0
#define LED_Green_GPIO_Port GPIOB
#define LED_Red_Pin GPIO_PIN_1
#define LED_Red_GPIO_Port GPIOB
#define L1_Pin GPIO_PIN_13
#define L1_GPIO_Port GPIOB
#define L2_Pin GPIO_PIN_14
#define L2_GPIO_Port GPIOB
#define L3_Pin GPIO_PIN_15
#define L3_GPIO_Port GPIOB
#define CS_Pin GPIO_PIN_9
#define CS_GPIO_Port GPIOC
#define H1_Pin GPIO_PIN_8
#define H1_GPIO_Port GPIOA
#define H2_Pin GPIO_PIN_9
#define H2_GPIO_Port GPIOA
#define H3_Pin GPIO_PIN_10
#define H3_GPIO_Port GPIOA
#define EN_Gate_Pin GPIO_PIN_5
#define EN_Gate_GPIO_Port GPIOB
#define FAULT_Pin GPIO_PIN_7
#define FAULT_GPIO_Port GPIOB
#define FAULT_EXTI_IRQn EXTI9_5_IRQn

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
