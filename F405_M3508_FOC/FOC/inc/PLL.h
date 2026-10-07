#ifndef PLL_H
#define PLL_H

/* ============================================================================
 * PLL.h —— 二阶(类型 II)锁相环: 把"角度测量"变成平滑角度 + 速度
 *
 *   用途: 滑模观测器给的是每拍一个 atan2 角度(抖、且速度要差分), 经过 PLL 后:
 *      - 角度平滑(等效一个带通跟踪滤波器)
 *      - 速度直接由积分器给出, 不用差分, 没有差分噪声放大
 *      - 稳态速度误差为 0(类型 II), 只有加速度下才有 ∝ a/ωn² 的滞后
 *
 *   结构(连续域):
 *      ε = wrap(θ_meas − θ̂)          ← 鉴相器(线性范围 ±π, 比 sin 鉴相更宽)
 *      ω̂ = Ki·∫ε dt                   ← 环路积分器 = 速度估计
 *      θ̂ = ∫(ω̂ + Kp·ε) dt             ← 比例项 + 速度积分
 *   整定:  ωn = 2π·fn,  Kp = 2ζ·ωn,  Ki = ωn²
 *
 *   注意:
 *    1. 这是"跟踪角度测量"的 PLL, 不含符号/方向判定。EMF 类观测器在反转时
 *       角度会偏 π(反电动势矢量反向), 所以本 PLL 默认正转使用; 需要四象限请
 *       在调用前把 θ_meas 的 π 模糊处理掉(比如用已知转向或 HFI 融合)。
 *    2. 测量无效时(如 |E| 太小)**不要调用** PLL_Step, 环路会自己保持。
 *    3. 同一个 PLL 也能用在 HFI 上: 传入 2·θ_sal, 并把 ω 除以 2 即为电角速度。
 * ========================================================================== */

#include <stdint.h>

typedef struct
{
    float theta;     /* 估计电角度 rad, [0, 2π) */
    float omega;     /* 估计电角速度 rad/s */
    float kp;        /* 比例增益 [1/s] */
    float ki;        /* 积分增益 [1/s²] */
    float ts;        /* 控制周期 s */
    float w_max;     /* 速度限幅 rad/s(防止首次锁定/失锁时跑飞) */
    uint8_t init;    /* 0 = 还没首帧(首帧直接对齐角度) */
} PLL_t;

void  PLL_Init(PLL_t *p, float fn_hz, float zeta, float ts, float w_max);
void  PLL_Reset(PLL_t *p);
float PLL_Step(PLL_t *p, float theta_meas);   /* 返回平滑后的角度 */
float PLL_GetTheta(const PLL_t *p);
float PLL_GetOmega(const PLL_t *p);

#endif
