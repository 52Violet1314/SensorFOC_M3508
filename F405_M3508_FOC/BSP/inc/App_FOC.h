#ifndef APP_FOC_H
#define APP_FOC_H

#include "stm32f4xx_hal.h"

void App_FOC_Init(void);
void App_FOC_MainLoop(void);

/* State exported for CAN feedback and optional diagnostics. */
extern float Encoder_Angle;
extern float Encoder_Elec_Angle;
extern float Encoder_Mech_Position;
extern float iq_filt;
extern float Temp;
extern float Power;
extern float spd_meas_rpm;

/* 你自己的观测器估计结果可以在这里 extern 出来, 供 CAN 上报或打印对比 */

#endif
