/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "main.h"
//#include "cmsis_os.h"
#include "adc.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "fdcan.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint32_t task_cnt_10ms = 0UL;
uint32_t task_cnt_50ms = 0UL;
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
#include "bsp.h"
extern void ecat_Init(void);
extern void ecat_main(void);
static void keyTask(void);
static void MotorStatus_Indication(void);

int main(void)
{

    /* USER CODE BEGIN 1 */
    /* USER CODE END 1 */

    /* MCU Configuration--------------------------------------------------------*/

    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    HAL_Init();

    /* USER CODE BEGIN Init */
    /* USER CODE END Init */

    /* Configure the TIM6 for system clock */
    SystemClock_Config();

    /* USER CODE BEGIN SysInit */

    MX_GPIO_Init();//key and led
    
    MX_DMA_Init();//debug_printf
    MX_USART3_UART_Init();//debug_printf
    
    MX_TIM1_Init();//motor pwm
    MX_ADC1_Init();//motor current
    MX_SPI1_Init();//mt6701

    PrintfLogo();
    
    bsp_InitKey();

    bsp_pwm_init();						/* PWM enable and trigger adc sample */
    FOC_Controller_initialize();        /* FOC algorithm initial*/
    encoder_calibration();
    bsp_pwm_switch_on();    
    
    MX_SPI3_Init();//ethercat lan9252
    ecat_Init(); /* EtherCAT stack initialization */
    
    debug_print("initialization complete, please enjoy the motor\r\n");
    debug_print("current mode: open loop\r\n");

    task_cnt_10ms = 0;
    task_cnt_50ms = 0;
    while(1)
    {
        //HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);
        ecat_main();
        if(task_cnt_10ms >= 10)
        {
            task_cnt_10ms = 0;
            bsp_KeyScan10ms(); /* ???? */
            if(FOC_bInitPID_S == true)
            {
                encoder_set_angle(0);//clear postion when mode change
            }
            Position_degMeasured_S = encoder_get_angle();
            PositionControl_Co();
            
            FOC_nSpeedMeasRaw_S = encoder_calc_speed();//*0.104719758F;
            SpeedControl_Co();
        }
        if(task_cnt_50ms >= 50)
        {
            task_cnt_50ms =0;
            ModeRegul_Co();
            keyTask();
            MotorStatus_Indication();
        }
        //HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);
    }
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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;//8MHz
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV2;//8/2=4MHz
  RCC_OscInitStruct.PLL.PLLN = 85;//4*85=340MHz
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;//340/2=170MHz
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;//340/2=170MHz
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;//340/2=170MHz
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;//select PLL 170MHz
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;//APB1=170MHz,TIM2-7,USART2-5
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;//APB2=170MHz,TIM1/8/20,TIM15/16/17,USART1,

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */


/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */

  /* USER CODE BEGIN Callback 1 */
  if (htim->Instance == TIM6) //1ms
  {
      HAL_IncTick();
      task_cnt_10ms++;
      task_cnt_50ms++;
  }
  /* USER CODE END Callback 1 */
}
/**
***********************************************************************
* @brief:      bsp_flash_write_canid(uint8_t can_id)
* @param:	   can id to be saved
* @retval:     void
* @details:    save can id to flash
***********************************************************************
**/
static void keyTask(void)
{
	uint8_t ucKeyCode;
	
    ucKeyCode = bsp_GetKey();
    switch (ucKeyCode)
    {
        case KEY_DOWN_K1:		/* K1 Pressed - Motor Enable*/
        {
            debug_print("Motor Enable !\r\n");
            bsp_pwm_switch_on();
            FOC_bInitPID_S = true;
            break;
        }
        case KEY_DOWN_K2:		/* K2 Pressed - Motor Disable */
        {
            debug_print("Motor Disable !\r\n");
            bsp_pwm_switch_off();
            FOC_bInitPID_S = true;
            FOC_uManualVq_C = 0.0f;
            Speed_nSetPoint_C = 0.0f;
            Position_degSetPoint_C = 0.0f;
            break;
        }
        case KEY_DOWN_K3:		/* K3 Pressed - Speed Up*/
        {
            if (bsp_pwm_is_enable())
            {
                if(Motor_swtMode_C == 0)
                {
                    if(FOC_uManualVq_C<=6.0f)
                    {
                        FOC_uManualVq_C += 0.5f;
                    }
                    debug_print("Vq=%f V\r\n",FOC_uManualVq_C);
                }
                else if(Motor_swtMode_C == 2)
                {
                    if(Speed_nSetPoint_C<=1700.0f)
                    {
                        Speed_nSetPoint_C += 100.0f;
                    }
                    debug_print("speed=%f rpm\r\n",Speed_nSetPoint_C);
                }
                else if(Motor_swtMode_C == 3)
                {
                    if(Position_degSetPoint_C<=360.0f)
                    {
                        Position_degSetPoint_C += 10.0f;
                    }
                    debug_print("position=%f deg\r\n",Position_degSetPoint_C);
                }
            }
            else
            {
                debug_print("Please Enable Motor first !\r\n");
            }
            break;
        }
        case KEY_DOWN_K4: 	/* K4 Pressed - Speed Down*/
        {
            if (bsp_pwm_is_enable())
            {
                if(Motor_swtMode_C == 0)
                {
                    if(FOC_uManualVq_C>=-6.0f)
                    {
                        FOC_uManualVq_C -= 0.5f;
                    } 
                    debug_print("Vq=%f V\r\n",FOC_uManualVq_C);
                }
                else if(Motor_swtMode_C == 2)
                {
                    if(Speed_nSetPoint_C>=-1700.0f)
                    {
                        Speed_nSetPoint_C -= 100.0f;
                    }
                    debug_print("speed=%f rpm\r\n",Speed_nSetPoint_C);
                }
                else if(Motor_swtMode_C == 3)
                {
                    if(Position_degSetPoint_C>=-360.0f)
                    {
                        Position_degSetPoint_C -= 10.0f;
                    }
                    debug_print("position=%f deg\r\n",Position_degSetPoint_C);
                }
            }
            else
            {
                debug_print("Please Enable Motor first !\r\n");
            }
            break;
        }
        case SYS_DOWN_K1K2:/* K1 and K2 Pressed toggether - Change Mode*/
        {
            bsp_pwm_switch_off();
            if(++Motor_swtMode_C>3)
            {
                Motor_swtMode_C = 0;
            }
            FOC_uManualVq_C = 0.0f;
            Speed_nSetPoint_C = 0.0f;
            Position_degSetPoint_C = 0.0f;
            break;
        }
        default:
            break;
    }

}
/**
***********************************************************************
* @brief:      bsp_flash_write_canid(uint8_t can_id)
* @param:	   can id to be saved
* @retval:     void
* @details:    save can id to flash
***********************************************************************
**/
static uint8_t Motor_swtModeLast;
static void MotorStatus_Indication(void)
{
    if(Motor_swtModeLast != Motor_swtMode_C)
    {
        switch(Motor_swtMode_C)
        {
            case 0:
                bsp_LedOff(1);
                bsp_LedOff(2);
                bsp_LedOff(3);
                debug_print("current mode: open loop\r\n");
                break;
            case 1:
                bsp_LedOn(1);
                bsp_LedOff(2);
                bsp_LedOff(3);
                debug_print("current mode: torque loop\r\n");
                break;
            case 2:
                bsp_LedOn(1);
                bsp_LedOn(2);
                bsp_LedOff(3);
                debug_print("current mode: speed loop\r\n");
                break;
            case 3:
                bsp_LedOn(1);
                bsp_LedOn(2);
                bsp_LedOn(3);
                debug_print("current mode: position loop\r\n");
                break;
            default:
                break;
        } 
        Motor_swtModeLast = Motor_swtMode_C;
    }
}
/*
*********************************************************************************************************
*	? ? ?: MCT_high_frequency_task
*	????: ????
*	?    ?: ?
*	? ? ?: ?
*********************************************************************************************************
*/
float CurrDev_facCurrentFilter_C = 10.0F;
float CurrDev_uPhaseU_S,CurrDev_uPhaseV_S,CurrDev_uPhaseW_S;
void MCT_high_frequency_task(void)
{
    bsp_spi_mt6701_update(50);//call with 50us

    FOC_cntThetaElec_S = encoder_normalize_angle();

	/* ????, ????????????? */
	CurrDev_uPhaseU_S = -read_iphase_a();
	CurrDev_uPhaseV_S = -read_iphase_b();
	CurrDev_uPhaseW_S = -read_iphase_c();
    
    FOC_iPhaseAMeas_S = bsp_filter_co(CurrDev_uPhaseU_S,CurrDev_facCurrentFilter_C,FOC_iPhaseAMeas_S);
	FOC_iPhaseBMeas_S = bsp_filter_co(CurrDev_uPhaseV_S,CurrDev_facCurrentFilter_C,FOC_iPhaseBMeas_S);
	FOC_iPhaseCMeas_S = bsp_filter_co(CurrDev_uPhaseW_S,CurrDev_facCurrentFilter_C,FOC_iPhaseCMeas_S);


    //FOC??
    CurrentControl_Co();		
    
    /* ???? */
	set_a_voltage(SVM_uPhaseU_S);
	set_b_voltage(SVM_uPhaseV_S);
	set_c_voltage(SVM_uPhaseW_S);  

}
/*
*********************************************************************************************************
*	? ? ?: HAL_ADCEx_InjectedConvCpltCallback
*	????: ADC??????????
*	?    ?:hadc:ADC??
*	? ? ?: ?
*********************************************************************************************************
*/
void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc)
{
	/* Calc ADC offset */
	static int adc_sum_a = 0;
	static int adc_sum_b = 0;
	static int adc_sum_c = 0;
	static uint8_t isCalcAdcOffsetOvered = FALSE;
	const int measCnt = 64;
	static int measCntCopy = measCnt;
	if (hadc->Instance == ADC1 && !isCalcAdcOffsetOvered)
	{
		adc_sum_a += hadc1.Instance->JDR1;
		adc_sum_b += hadc1.Instance->JDR2;
		adc_sum_c += hadc1.Instance->JDR3;

		if (--measCntCopy <= 0)
		{
			phase_a_adc_offset = adc_sum_a / measCnt;
			phase_b_adc_offset = adc_sum_b / measCnt;
			phase_c_adc_offset = adc_sum_c / measCnt;

			isCalcAdcOffsetOvered = TRUE;
		}
	}
	if (hadc->Instance == ADC1 && isCalcAdcOffsetOvered)
	{
        if(Encoder.direction != 0)
        {
            MCT_high_frequency_task();//FOC control
        }
	}
}

/*
*********************************************************************************************************
*	? ? ?: PrintfLogo
*	????: ?????????????, ??????,??PC??????????????
*	?    ?: ?
*	? ? ?: ?
*********************************************************************************************************
*/
void PrintfLogo(void)
{
	
	/* ??CPU ID */
	uint32_t CPU_Sn0, CPU_Sn1, CPU_Sn2;
	
	CPU_Sn0 = HAL_GetUIDw0();
	CPU_Sn1 = HAL_GetUIDw1();
	CPU_Sn2 = HAL_GetUIDw2();
	
	debug_print("\r\n-------------------------------------------------------------\r\n");
    debug_print("                                                             \r\n");
    debug_print("    ######   ######   #######      #######  #######   #####\r\n");
	debug_print(" #  #     #  #     #     #         #        #     #  #     # \r\n");
	debug_print("    #     #  #     #     #         #        #     #  #\r\n");
	debug_print(" #  ######   ######      #         #####    #     #  # \r\n");
    debug_print(" #  #   #    #     #     #         #        #     #  #\r\n");
    debug_print(" #  #    #   #     #     #         #        #     #  #     #\r\n");
    debug_print(" #  #     #  ######      #         #        #######   #####\r\n");                                                                
	debug_print("                                                            \r\n");
    
	debug_print("Build:       "__DATE__" "__TIME__"\r\n");
    debug_print("CPU: STM32G474RET6, SYSCLK: %dMHz, RAM: 128KB, ROM: 512KB\r\n", SystemCoreClock / 1000000);
	debug_print("UID: %08X %08X %08X\r\n", CPU_Sn2, CPU_Sn1, CPU_Sn0);
	debug_print("STM32G4xx_HAL_Driver:STM32Cube_FW_G4 V1.2.5\r\n");
    debug_print("Current version of Matlab FOC Algorithm: %s\r\n", MATLAB_SOFTWARE_VERSION);
	debug_print("-------------------------------------------------------------\r\n");
    
	debug_print("HW-Version:   V%d.%d.%d\r\n",USER_HW_VERSION_MAIN,USER_HW_VERSION_SUB1,USER_HW_VERSION_SUB2);
    debug_print("SW-Version:   V%d.%d.%d\r\n",USER_SW_VERSION_MAIN,USER_SW_VERSION_SUB1,USER_SW_VERSION_SUB2);
    debug_print("Motor Select: %s\r\n", MOTOR_NAME);
    debug_print("Current Project is <<iRDT_Driver_BareMetal_EtherCAT>>\r\n");
    debug_print("--------------Copyright:   (C) 2026 iRobotTribe--------------\r\n\r\n");
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

#ifdef  USE_FULL_ASSERT
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
