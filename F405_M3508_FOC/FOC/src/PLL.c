#include "PLL.h"

#define PLL_PI      3.14159265359f
#define PLL_TWO_PI  6.28318530718f

void PLL_Init(PLL_t *p, float fn_hz, float zeta, float ts, float w_max)
{
    float wn = PLL_TWO_PI * fn_hz;

    p->kp = 2.0f * zeta * wn;      /* [1/s]  */
    p->ki = wn * wn;               /* [1/s²] */
    p->ts = (ts > 0.0f) ? ts : 0.000025f;
    p->w_max = (w_max > 0.0f) ? w_max : 10000.0f;
    PLL_Reset(p);
}

void PLL_Reset(PLL_t *p)
{
    p->theta = 0.0f;
    p->omega = 0.0f;
    p->init  = 0u;
}

float PLL_Step(PLL_t *p, float theta_meas)
{
    float err;

    /* 首帧: 直接把角度对齐到测量值, 避免从 0 抢锁造成的大冲击 */
    if (p->init == 0u)
    {
        p->theta = theta_meas;
        while (p->theta >= PLL_TWO_PI) p->theta -= PLL_TWO_PI;
        while (p->theta <  0.0f)       p->theta += PLL_TWO_PI;
        p->omega = 0.0f;
        p->init  = 1u;
        return p->theta;
    }

    /* 鉴相: 归一化到 [-π, π) */
    err = theta_meas - p->theta;
    while (err >=  PLL_PI) err -= PLL_TWO_PI;
    while (err <  -PLL_PI) err += PLL_TWO_PI;

    /* 环路: 积分器给速度, 比例项直接修相位 */
    p->omega += p->ki * err * p->ts;
    if (p->omega >  p->w_max) p->omega =  p->w_max;
    if (p->omega < -p->w_max) p->omega = -p->w_max;

    p->theta += (p->omega + p->kp * err) * p->ts;
    while (p->theta >= PLL_TWO_PI) p->theta -= PLL_TWO_PI;
    while (p->theta <  0.0f)       p->theta += PLL_TWO_PI;

    return p->theta;
}

float PLL_GetTheta(const PLL_t *p) { return p->theta; }
float PLL_GetOmega(const PLL_t *p) { return p->omega; }
