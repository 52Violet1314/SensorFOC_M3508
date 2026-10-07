#ifndef LPF_H
#define LPF_H

/* ============================================================================
 * LPF.h —— 一阶低通(指数滑动平均 EMA)
 *
 *      y(k) = y(k-1) + α·[ x(k) − y(k-1) ]
 *      α = 1 − exp(−2π·fc/fs) ≈ 2π·fc/fs     (fc << fs)
 *      fc: 截止频率 Hz,  fs: 采样频率 Hz
 *
 *   注意: 一阶低通的相位滞后在"两个通道过同一个 α"时, 会在 atan2 里相互抵消,
 *   所以观测器里的 E_alpha / E_beta 必须用同一个 α(这是有意的, 不是巧合)。
 *
 *   两种用法(数学完全一样, 只是状态放哪):
 *     A. 状态自己保存(适合结构体里已有字段的, 如 SMO):
 *          smo->E_alpha = LPF_Step(smo->E_alpha, raw, SMO_EMF_LPF_ALPHA);
 *     B. 状态用 LPF_t 打包:
 *          LPF_Init(&f, LPF_Alpha(500.0f, FOC_FS));
 *          y = LPF_Update(&f, x);
 * ========================================================================== */

#include <math.h>

/* --- A. 无状态形式: 上一拍输出由调用方保存 --- */
static inline float LPF_Step(float y_prev, float x, float alpha)
{
    return y_prev + alpha * (x - y_prev);
}

/* 由截止频率反算每拍系数(建议初始化时算一次存起来, 别每拍调) */
static inline float LPF_Alpha(float fc_hz, float fs_hz)
{
    if (fs_hz <= 0.0f) return 1.0f;
    return 1.0f - expf(-6.28318530718f * fc_hz / fs_hz);
}

/* --- B. 打包形式 --- */
typedef struct
{
    float y;      /* 输出 = 状态 */
    float alpha;  /* 每拍系数 [0,1] */
} LPF_t;

static inline void LPF_Init(LPF_t *f, float alpha)
{
    f->y = 0.0f;
    f->alpha = alpha;
}

static inline void LPF_Reset(LPF_t *f)
{
    f->y = 0.0f;
}

static inline float LPF_Update(LPF_t *f, float x)
{
    f->y += f->alpha * (x - f->y);
    return f->y;
}

#endif
