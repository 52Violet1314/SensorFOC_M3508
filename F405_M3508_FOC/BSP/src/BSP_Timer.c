#include "BSP_Timer.h"
#include "main.h"
#include "tim.h"
#include "SVPWM.h"      /* PWM_PERIOD: 必须与控制频率一致(改一处改不了这里就编译期暴露) */

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
    /* 中心对齐下的采样触发点: ARR − 死区(50) = 死区结束、低侧管导通的中点,
     * 这时采样最干净。★ARR 跟着控制频率走(20kHz -> 4200; 40kHz -> 2100) */
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, PWM_PERIOD - 50u);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
    /* 互补通道(低侧管 CH1N/2N/3N)必须同时启动, 否则低侧永不导通 */
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
}
