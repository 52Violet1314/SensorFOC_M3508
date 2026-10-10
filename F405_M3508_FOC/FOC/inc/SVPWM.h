#ifndef __SVPWM_H__
#define __SVPWM_H__

#include <stdint.h>

/* TIM1 自动重载值(中心对齐, 168MHz/2/4200 = 20kHz)
 *   ★必须与 APP_CTRL_HZ / tim.c 的 htim1.Init.Period / BSP_Timer.c 的 CCR4 一致 */
#define PWM_PERIOD       4200u

/* vdc 为实测母线电压 V, 由调用方传入(ADC 采样) */
void SVPWM(float valpha, float vbeta, float vdc, uint16_t *ccr1, uint16_t *ccr2, uint16_t *ccr3);

#endif
