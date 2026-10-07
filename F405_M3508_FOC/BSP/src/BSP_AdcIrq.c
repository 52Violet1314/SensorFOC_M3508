#include "BSP_AdcIrq.h"
#include "main.h"
#include "adc.h"

volatile uint16_t BSP_AdcRaw_PhaseV[3];
volatile uint16_t BSP_AdcRaw_Current[3];
volatile uint16_t BSP_AdcRaw_Power;
volatile uint16_t BSP_AdcRaw_Temp;
volatile uint16_t BSP_AdcRaw_EncSin;
volatile uint16_t BSP_AdcRaw_EncCos;

static BSP_AdcGroupHook_t adc_group_hook;
static volatile uint8_t   adc_group_mask;
static uint32_t           isr_cyc_max;

void BSP_AdcIrq_SetGroupHook(BSP_AdcGroupHook_t hook)
{
    adc_group_hook = hook;
}

uint32_t BSP_AdcIrq_GetMaxCycle(void)
{
    return isr_cyc_max;
}

/* HAL 注入组转换完成回调: 只搬原始值 + 判组齐 + 调上层回调, 没有任何应用逻辑 */
void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance == ADC1)
    {
        BSP_AdcRaw_PhaseV[0]  = HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_1);
        BSP_AdcRaw_EncSin     = HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_2);
        BSP_AdcRaw_Current[0] = HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_3);
        BSP_AdcRaw_Power      = HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_4);
        adc_group_mask |= 0x01u;
    }
    else if (hadc->Instance == ADC2)
    {
        BSP_AdcRaw_PhaseV[1]  = HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_1);
        BSP_AdcRaw_EncCos     = HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_2);
        BSP_AdcRaw_Current[1] = HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_3);
        BSP_AdcRaw_Temp       = HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_4);
        adc_group_mask |= 0x02u;
    }
    else if (hadc->Instance == ADC3)
    {
        BSP_AdcRaw_PhaseV[2]  = HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_1);
        BSP_AdcRaw_Current[2] = HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_2);
        adc_group_mask |= 0x04u;
    }

    /* 三相注入组全部转换完成后再进应用层: 保证 ia/ib/ic 取自同一采样时刻,
     * 否则 Clarke / 二阶差分会出现一拍错位 */
    if (adc_group_mask == 0x07u)
    {
        adc_group_mask = 0u;
        if (adc_group_hook != 0)
        {
            uint32_t cyc0 = DWT->CYCCNT;
            adc_group_hook();
            cyc0 = DWT->CYCCNT - cyc0;
            if (cyc0 > isr_cyc_max) isr_cyc_max = cyc0;
        }
    }
}
