/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "adc.h"
#include "spi.h"
#include "tim.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "BSP_Timer.h"
#include "SVPWM.h"
#include "VF.h"
#include "usbd_cdc_if.h"
#include "math.h"
#include "arm_math.h"
#include "VF.h"
#include <stdint.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define USE_CMSIS_OS2 0
/* ADC 换算参数 */
#define ADC_VREF        3.3f    /* ADC 参考电压 (V) */
#define VDC_DIVIDER     18.727272f    /* Power_DC 分压比, 按硬件修改: Vbus = Vadc * K */
#define CSA_VMID        2047   /* 电流采样中点电压 (V), 上电后实测标定 */
#define CSA_SENS        0.1f    /* 电流传感灵敏度 (V/A), 按 CSA 增益与采样电阻计算 */
#define TWO_PI  6.28318530718f
#define SIN_MAX  238
#define COS_MAX  2306
#define SIN_MIN  320
#define COS_MIN  2241
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
float ADC_Current[3];
float ADC_Voltage[3];
float ADC_Power;
float ADC_Temp;
float Current[3];
float Voltage[3];
float Power;
float Temp;
int Encoder_Sin;
int Encoder_Cos;
uint16_t Encoder_Sin_Min = 2000;
uint16_t Encoder_Sin_Max = 0;
uint16_t Encoder_Cos_Min = 2000;
uint16_t Encoder_Cos_Max = 0;
float Encoder_Angle;
uint32_t Heartbeat_ms = 0;   /* LED 心跳计时, 用于确认控制中断在运行 */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_ADC3_Init();
  MX_SPI3_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  MX_USB_DEVICE_Init();
  BSP_Timer_Init();

  /* 注入转换由 TIM1_CC4 触发, TIM1 计数已由 BSP_Timer_Init 启动(无输出) */
  HAL_ADCEx_InjectedStart_IT(&hadc1);
  HAL_ADCEx_InjectedStart_IT(&hadc2);
  HAL_ADCEx_InjectedStart_IT(&hadc3);
  BSP_Timer_PWM_Start();

  /* 使能门极驱动, 开环 V/F 起转, 目标 100rpm (极对数 7) */
  HAL_GPIO_WritePin(EN_Gate_GPIO_Port, EN_Gate_Pin, GPIO_PIN_SET);
  VF_SetTargetRPM(100.0f);

#if USE_CMSIS_OS2 == 1
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();  /* Call init function for freertos objects (in cmsis_os2.c) */
  MX_FREERTOS_Init();

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
#endif


  while (1)
  {
    Current[0] = (ADC_Current[0] - CSA_VMID) / 4095.0f * 3.3f * 20.0f;
    Current[1] = (ADC_Current[1] - CSA_VMID) / 4095.0f * 3.3f * 20.0f;
    Current[2] = (ADC_Current[2] - CSA_VMID) / 4095.0f * 3.3f * 20.0f;
    Voltage[0] = ADC_Voltage[0] / 4095.0f * 3.3f * VDC_DIVIDER;
    Voltage[1] = ADC_Voltage[1] / 4095.0f * 3.3f * VDC_DIVIDER;
    Voltage[2] = ADC_Voltage[2] / 4095.0f * 3.3f * VDC_DIVIDER;
    Power = ADC_Power / 4095.0f * 3.3f * VDC_DIVIDER;
    if(Encoder_Sin < Encoder_Sin_Min && Encoder_Sin != 0)
    Encoder_Sin_Min = Encoder_Sin;
    if(Encoder_Sin > Encoder_Sin_Max)
    Encoder_Sin_Max = Encoder_Sin;
    if(Encoder_Cos < Encoder_Cos_Min && Encoder_Cos != 0)
    Encoder_Cos_Min = Encoder_Cos;
    if(Encoder_Cos > Encoder_Cos_Max)
    Encoder_Cos_Max = Encoder_Cos;
    if(Encoder_Sin != 0 && Encoder_Cos != 0)
    {
    float sin_norm = 2.0f * (float)(Encoder_Sin - SIN_MIN) / (float)(SIN_MAX - SIN_MIN) - 1.0f;
    float cos_norm = 2.0f * (float)(Encoder_Cos - COS_MIN) / (float)(COS_MAX - COS_MIN) - 1.0f;
    Encoder_Angle = atan2f(sin_norm, cos_norm);   // 四象限，输出 -π ~ π
    }
    Encoder_Angle = Encoder_Angle + 0.55f;
    float err = vf_theta - Encoder_Angle;
    if(err > PI)
    err -= TWO_PI;
    else if(err < -PI)
    err += TWO_PI;
    CDC_Printf("%f\r\n",err);
    HAL_Delay(1);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
/**
  * @brief  ADC 注入转换完成回调 (TIM1_CC4 谷底触发, 10kHz)
  *         ADC1: rank1=VA rank2=SIN rank3=IA rank4=Power_DC
  *         ADC2: rank1=VB rank2=COS rank3=IB rank4=Temp_Motor
  *         ADC3: rank1=VC rank2=IC
  *         三个 ADC 由同一触发源同时启动, 数据同一周期, 无需跨 ADC 对齐
  */
void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc)
{
  if(hadc->Instance == ADC1)
  {
    ADC_Voltage[0] = HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_1); /* VA   */
    Encoder_Sin    = HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_2); /* SIN */
    ADC_Current[0] = HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_3); /* IA   */
    ADC_Power      = HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_4); /* VDC  */

    Voltage[0] = ADC_Voltage[0] * ADC_VREF / 4095.0f;
    Current[0] = (ADC_Current[0] * ADC_VREF / 4095.0f - CSA_VMID) / CSA_SENS;
    Power      = ADC_Power * ADC_VREF / 4095.0f * VDC_DIVIDER;

    VF_Tick(Power);  /* 开环 V/F 控制环, 与 PWM 同步, 用实测母线电压 */

    /* 诊断: 绿 LED 0.5s 翻转一次, 证明此中断在运行 */
    if (HAL_GetTick() - Heartbeat_ms >= 500u)
    {
      HAL_GPIO_TogglePin(LED_Green_GPIO_Port, LED_Green_Pin);
      Heartbeat_ms = HAL_GetTick();
    }
    /* 诊断: 红 LED 亮 = 驱动芯片 FAULT 拉低 */
    if (HAL_GPIO_ReadPin(FAULT_GPIO_Port, FAULT_Pin) == GPIO_PIN_RESET)
    {
      HAL_GPIO_WritePin(LED_Red_GPIO_Port, LED_Red_Pin, GPIO_PIN_SET);
    }
    else
    {
      HAL_GPIO_WritePin(LED_Red_GPIO_Port, LED_Red_Pin, GPIO_PIN_RESET);
    }
  }
  if(hadc->Instance == ADC2)
  {
    ADC_Voltage[1] = HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_1); /* VB   */
    Encoder_Cos    = HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_2); /* COS */
    ADC_Current[1] = HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_3); /* IB   */
    ADC_Temp       = HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_4); /* TEMP */

    Voltage[1] = ADC_Voltage[1] * ADC_VREF / 4095.0f;
    Current[1] = (ADC_Current[1] * ADC_VREF / 4095.0f - CSA_VMID) / CSA_SENS;
    Temp       = ADC_Temp * ADC_VREF / 4095.0f; /* 温度通道电压, 换算公式待硬件确定 */
  }
  if(hadc->Instance == ADC3)
  {
    ADC_Voltage[2] = HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_1); /* VC   */
    ADC_Current[2] = HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_2); /* IC   */

    Voltage[2] = ADC_Voltage[2] * ADC_VREF / 4095.0f;
    Current[2] = (ADC_Current[2] * ADC_VREF / 4095.0f - CSA_VMID) / CSA_SENS;
  }
}
/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM2 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM2)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
