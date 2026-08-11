#include "VF.h"
#include "SVPWM.h"
#include "main.h"
#include "tim.h"
#include <math.h>

#define TWO_PI  6.28318530718f

static float vf_speed_rpm  = 0.0f;    /* 当前斜坡转速 rpm */
static float vf_target_rpm = 0.0f;    /* 目标转速 rpm */
float vf_theta      = 0.0f;    /* 软件模拟电角度 rad */
static float vf_dt         = VF_DT;   /* 控制步长 s, 上电后自校准 */

/* 回调率自校准: 统计 200ms 内调用次数, dt = 1/rate */
static uint8_t  vf_calib    = 1u;
static uint32_t vf_calib_t0 = 0u;
static uint32_t vf_calib_n  = 0u;

void VF_SetTargetRPM(float rpm)
{
    if (rpm >  VF_MAX_RPM) rpm =  VF_MAX_RPM;
    if (rpm < -VF_MAX_RPM) rpm = -VF_MAX_RPM;
    vf_target_rpm = rpm;
}

/* 控制环, 在 ADC 注入转换完成中断中调用
   电角度由软件积分模拟, 与硬件编码器无关
   vdc: 实测母线电压 V, 由 ADC 换算后传入 */
void VF_Tick(float vdc)
{
    /* 0. 控制步长自校准(上电 200ms 内完成, 之后电角度频率精确) */
    if (vf_calib)
    {
        if (vf_calib_n == 0u) vf_calib_t0 = HAL_GetTick();
        vf_calib_n++;
        if (HAL_GetTick() - vf_calib_t0 >= 200u)
        {
            vf_dt = 0.001f * 200.0f / (float)vf_calib_n;
            vf_calib = 0u;
        }
    }

    /* 1. 转速斜坡: 防止起步阶跃, 避免失步 */
    float diff = vf_target_rpm - vf_speed_rpm;
    float step = VF_ACC_RPM_PER_S * vf_dt;
    if      (diff >  step) vf_speed_rpm += step;
    else if (diff < -step) vf_speed_rpm -= step;
    else                   vf_speed_rpm  = vf_target_rpm;

    /* 2. 软件模拟电角度积分: fe = rpm/60 * 极对数 */
    float fe  = vf_speed_rpm / 60.0f * VF_POLE_PAIRS;
    float we  = TWO_PI * fe;
    vf_theta += we * vf_dt;
    if      (vf_theta > TWO_PI) vf_theta -= TWO_PI;
    else if (vf_theta < 0.0f)   vf_theta += TWO_PI;

    /* 3. V/F 电压: 低速补偿 + 斜率, 限幅 */
    float v = VF_V_BOOST + VF_V_SLOPE * fabsf(fe);
    if (v > VF_V_MAX) v = VF_V_MAX;

    /* 4. αβ 电压 → SVPWM → 更新 CCR */
    uint16_t ccr1, ccr2, ccr3;
    SVPWM(v * cosf(vf_theta), v * sinf(vf_theta), vdc, &ccr1, &ccr2, &ccr3);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, ccr1);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, ccr2);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, ccr3);
}
