#include "BSP_Timer.h"
#include "main.h"
#include "tim.h"

//定时器初始化
void BSP_Timer_Init(void)
{
    HAL_TIM_Base_Start(&htim1);
    // HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_ALL);
}

void BSP_Timer_PWM_Start(void)
{
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 8350);  /* 中心对齐: ARR-DeadTime, 避开端点并在死区结束后采样, 10kHz */
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
    /* 互补通道(低侧管 CH1N/2N/3N)必须同时启动, 否则低侧永不导通 */
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
}
