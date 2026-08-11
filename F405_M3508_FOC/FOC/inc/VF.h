#ifndef __VF_H__
#define __VF_H__

#include <stdint.h>

/* 电机参数 */
#define VF_POLE_PAIRS     7u          /* 极对数(磁对数) */
#define VF_DT             0.00005f    /* 控制周期 s 初始值(ADC 注入中断), 上电后自校准 */

/* V/F 曲线: V = V_BOOST + V_SLOPE * fe, fe 为电频率 Hz */
#define VF_V_BOOST        0.15f       /* 低速补偿电压 V, 用于克服死区与电阻压降 */
#define VF_V_SLOPE        0.030f      /* 电压/频率斜率 V/Hz, M3508 KV100: 100rpm 反电动势约 0.82V */
#define VF_V_MAX          2.0f        /* 输出电压峰值上限 V */

/* 加减速与限幅 */
#define VF_ACC_RPM_PER_S  50.0f       /* 转速斜坡斜率 rpm/s */
#define VF_MAX_RPM        500.0f      /* 目标转速限幅 rpm */

extern float vf_theta;

void VF_SetTargetRPM(float rpm);
void VF_Tick(float vdc);

#endif
