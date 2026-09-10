#ifndef VF_APP_H
#define VF_APP_H

#include "adc.h"

/* Hardware-facing V/F application layer. Call these after CubeMX peripheral
 * initialization, from the main loop, and from the ADC injected callback. */
void VF_AppInit(void);
void VF_AppSetOutputEnabled(uint8_t enable);
void VF_AppTask(void);
void VF_AppAdcTick(ADC_HandleTypeDef *hadc);

#endif
