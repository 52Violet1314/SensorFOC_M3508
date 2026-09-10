/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32g4xx_hal.h"

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

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define PWM_FREQUENCY 20000
#define TIMER1_CLK_MHz 170
#define PWM_PERIOD_CYCLES (uint16_t)((TIMER1_CLK_MHz * (uint32_t) 1000000u / ((uint32_t) (PWM_FREQUENCY))) & 0xFFFE) //8500
#define HALF_PWM_PERIOD_CYCLES (uint16_t)(PWM_PERIOD_CYCLES / 2U) //4250

#define LED1_Pin       GPIO_PIN_13
#define LED1_GPIO_Port GPIOC
#define LED2_Pin       GPIO_PIN_14
#define LED2_GPIO_Port GPIOC
#define LED3_Pin       GPIO_PIN_15
#define LED3_GPIO_Port GPIOC

#define IC_ADC1_IN1_Pin       GPIO_PIN_0
#define IC_ADC1_IN1_GPIO_Port GPIOA
#define IB_ADC1_IN2_Pin       GPIO_PIN_1
#define IB_ADC1_IN2_GPIO_Port GPIOA
#define IA_ADC1_IN3_Pin       GPIO_PIN_2
#define IA_ADC1_IN3_GPIO_Port GPIOA


#define ECAT_RST_Pin       GPIO_PIN_4
#define ECAT_RST_GPIO_Port GPIOC
#define ECAT_INT_Pin       GPIO_PIN_4
#define ECAT_INT_GPIO_Port GPIOA
#define ECAT_INT_EXTI_IRQn EXTI4_IRQn
#define SYNC0_Pin          GPIO_PIN_5
#define SYNC0_GPIO_Port    GPIOA
#define SYNC0_EXTI_IRQn    EXTI9_5_IRQn
#define SYNC1_Pin          GPIO_PIN_6
#define SYNC1_GPIO_Port    GPIOA
#define SYNC1_EXTI_IRQn    EXTI9_5_IRQn

#define KEY4_Pin       GPIO_PIN_6
#define KEY4_GPIO_Port GPIOC
#define KEY3_Pin       GPIO_PIN_7
#define KEY3_GPIO_Port GPIOC
#define KEY2_Pin       GPIO_PIN_8
#define KEY2_GPIO_Port GPIOC
#define KEY1_Pin       GPIO_PIN_9
#define KEY1_GPIO_Port GPIOC


/* USER CODE BEGIN Private defines */
/**
  * @brief iRDT Driver Hardware version number V1.4.0
  */
#define USER_HW_VERSION_MAIN   (0x01U) /*!< [31:24] main version */
#define USER_HW_VERSION_SUB1   (0x04U) /*!< [23:16] sub1 version */
#define USER_HW_VERSION_SUB2   (0x00U) /*!< [15:8]  sub2 version */
#define USER_HW_VERSION_RC     (0x00U) /*!< [7:0]  release candidate */
#define USER_HW_VERSION        ((USER_HW_VERSION_MAIN << 24)\
                               |(USER_HW_VERSION_SUB1 << 16)\
                               |(USER_HW_VERSION_SUB2 << 8 )\
                               |(USER_HW_VERSION_RC))
/**
  * @brief iRDT Driver Software version number V1.2.0
  */                               
#define USER_SW_VERSION_MAIN   (0x01U) /*!< [31:24] main version */
#define USER_SW_VERSION_SUB1   (0x02U) /*!< [23:16] sub1 version */
#define USER_SW_VERSION_SUB2   (0x00U) /*!< [15:8]  sub2 version */
#define USER_SW_VERSION_RC     (0x00U) /*!< [7:0]  release candidate */
#define USER_SW_VERSION        ((USER_SW_VERSION_MAIN << 24)\
                               |(USER_SW_VERSION_SUB1 << 16)\
                               |(USER_SW_VERSION_SUB2 << 8 )\
                               |(USER_SW_VERSION_RC))
/**
  * @brief iRDT Driver Matlab version number V1.2.0
  */ 
#define MATLAB_SOFTWARE_VERSION        "9.5 (R2021a) 14-Nov-2020"

#define MOTOR_D2208         (1U)
#define MOTOR_D4006         (2U) 
#define USER_MOTOR          (MOTOR_D2208)

#if (USER_MOTOR == MOTOR_D2208)
#define MOTOR_NAME  "D2208"
#elif (USER_MOTOR == MOTOR_D4006)
#define MOTOR_NAME  "D4006"
#endif
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
