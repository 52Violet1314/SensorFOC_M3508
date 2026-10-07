#ifndef BSP_ADCIRQ_H
#define BSP_ADCIRQ_H

#include <stdint.h>

/* ============================================================================
 * BSP_AdcIrq —— ADC 注入组中断的"薄壳"(底层)
 *
 * 只做三件事, 不含任何应用逻辑(不换算、不做 Clarke/Park、不做 SVPWM、不 include 应用层头):
 *   ① 把每次注入转换的原始值搬进下面的 BSP_AdcRaw_* 变量
 *   ② 三个 ADC 都转换完成后判定"三相齐"
 *   ③ 调用上层注册的组回调(函数指针) —— 换算/变换/PI/SVPWM/观测器 全在那边做
 *
 * 好处: 底层不依赖应用层; 应用层可以整体替换(换控制算法、换观测器)而不用碰中断。
 * ========================================================================== */

typedef void (*BSP_AdcGroupHook_t)(void);

void BSP_AdcIrq_SetGroupHook(BSP_AdcGroupHook_t hook);

/* 中断本体的最坏耗时(CPU 周期) —— 含上层回调, 用来盯 40kHz 的预算 */
uint32_t BSP_AdcIrq_GetMaxCycle(void);

/* 原始采样值(中断里搬运, 应用层只读) */
extern volatile uint16_t BSP_AdcRaw_PhaseV[3];   /* 三相电压: ADC1/2/3 rank1 */
extern volatile uint16_t BSP_AdcRaw_Current[3];  /* 三相电流: ADC1 rank3, ADC2 rank3, ADC3 rank2 */
extern volatile uint16_t BSP_AdcRaw_Power;       /* 母线电压: ADC1 rank4 */
extern volatile uint16_t BSP_AdcRaw_Temp;        /* 温度:     ADC2 rank4 */
extern volatile uint16_t BSP_AdcRaw_EncSin;      /* 编码器 sin: ADC1 rank2 */
extern volatile uint16_t BSP_AdcRaw_EncCos;      /* 编码器 cos: ADC2 rank2 */

#endif
