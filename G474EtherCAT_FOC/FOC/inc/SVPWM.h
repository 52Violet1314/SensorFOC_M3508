#ifndef FOC_SVPWM_H
#define FOC_SVPWM_H

#include <stdint.h>

/* TIM1 is center-aligned with ARR = 2099 in the G474 project. */
#define SVPWM_PWM_PERIOD 2099u

/* Convert alpha/beta phase voltage (V) to TIM1 compare values. */
void SVPWM(float valpha, float vbeta, float vdc,
           uint16_t *ccr1, uint16_t *ccr2, uint16_t *ccr3);

#endif
