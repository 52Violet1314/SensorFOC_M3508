#include "FOC_Param.h"

#include "FOC_Config.h"
#include "adc.h"

volatile FOC_Param_t foc_param = {
    .bus_voltage = FOC_DEFAULT_BUS_VOLTAGE
};

static float adc_to_voltage(uint16_t raw)
{
    return (float)raw * FOC_ADC_VOLTAGE_PER_COUNT;
}

void FOC_ParamUpdateAdc1(void)
{
    float ia_voltage;
    float ib_voltage;
    float ic_voltage;

    /* ADC1 rank order: PA0=IC, PA1=IB, PA2=IA. */
    foc_param.raw.ic = (uint16_t)HAL_ADCEx_InjectedGetValue(&hadc1,
                                                            ADC_INJECTED_RANK_1);
    foc_param.raw.ib = (uint16_t)HAL_ADCEx_InjectedGetValue(&hadc1,
                                                            ADC_INJECTED_RANK_2);
    foc_param.raw.ia = (uint16_t)HAL_ADCEx_InjectedGetValue(&hadc1,
                                                            ADC_INJECTED_RANK_3);

    ia_voltage = adc_to_voltage(foc_param.raw.ia) - FOC_CURRENT_OFFSET_V;
    ib_voltage = adc_to_voltage(foc_param.raw.ib) - FOC_CURRENT_OFFSET_V;
    ic_voltage = adc_to_voltage(foc_param.raw.ic) - FOC_CURRENT_OFFSET_V;
    foc_param.current.a = ia_voltage * FOC_CURRENT_A_PER_V;
    foc_param.current.b = ib_voltage * FOC_CURRENT_A_PER_V;
    foc_param.current.c = ic_voltage * FOC_CURRENT_A_PER_V;
}

void FOC_ParamUpdateAdc2(void)
{
    foc_param.raw.va = (uint16_t)HAL_ADCEx_InjectedGetValue(&hadc2,
                                                            ADC_INJECTED_RANK_1);
    foc_param.raw.vb = (uint16_t)HAL_ADCEx_InjectedGetValue(&hadc2,
                                                            ADC_INJECTED_RANK_2);
    foc_param.raw.vc = (uint16_t)HAL_ADCEx_InjectedGetValue(&hadc2,
                                                            ADC_INJECTED_RANK_3);
    foc_param.raw.vbus = (uint16_t)HAL_ADCEx_InjectedGetValue(&hadc2,
                                                              ADC_INJECTED_RANK_4);

    foc_param.voltage.a = adc_to_voltage(foc_param.raw.va) * FOC_PHASE_VOLTAGE_SCALE;
    foc_param.voltage.b = adc_to_voltage(foc_param.raw.vb) * FOC_PHASE_VOLTAGE_SCALE;
    foc_param.voltage.c = adc_to_voltage(foc_param.raw.vc) * FOC_PHASE_VOLTAGE_SCALE;
    if ((float)foc_param.raw.vbus * FOC_BUS_VOLTAGE_SCALE >= 1.0f)
    {
        foc_param.bus_voltage = (float)foc_param.raw.vbus * FOC_BUS_VOLTAGE_SCALE;
    }
}
