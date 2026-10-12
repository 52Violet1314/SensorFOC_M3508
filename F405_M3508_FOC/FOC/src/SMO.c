#include "SMO.h"
#include "LPF.h"
#include <math.h>

#define SMO_PI      3.14159265359f
#define SMO_TWO_PI  6.28318530718f

/* ============================================================================
 * 滑模观测器(αβ 坐标系) —— 推导/整定约束见 SMO.h 顶部注释
 *
 * 本文件只做三件事:
 *   ① 用离散电流模型 î(k+1) = F·î(k) + G·[u(k) + w(k)] 估计电流
 *   ② 滑模项 w = k·sat((i − î)/δ), 滑模面上 w = −e ⇒ e_hat = −w
 *   ③ e_hat 低通 → 角度 atan2 / 电角速度差分
 * ========================================================================== */

/* --- 离散模型系数(由 R/L/Ts 算出, 改参数只需改 SMO_RS_DEF/SMO_LS_DEF/SMO_TS_DEF) --- */
static float smo_F;             /* exp(−Ts·R/L) */
static float smo_G;             /* (1−F)/R  [A/V] */
static float smo_Ts;            /* 观测周期 s */

/* --- 可调参数 --- */
static float smo_H = SMO_SLIDE_GAIN;      /* 滑模增益 k [V] */
static float smo_delta = SMO_BOUNDARY;    /* 边界层宽度 δ [A] */
static float smo_lag_w = SMO_LAG_WDEF;    /* 滞后补偿: 观测器等效带宽 rad/s */

/* --- 观测器状态 --- */
static float i_hat[2];          /* 估计电流 [A] */
static float e_hat[2];          /* 反电动势原始估计(=−w, 含抖振) [V] */
static float e_lpf[2];          /* 低通后的反电动势 [V] */
static float theta_prev;        /* 上一拍电角度, 用于差分求速度 */
static float omega_e;           /* 电角速度估计 [rad/s] */
static uint8_t theta_valid;

/* 观测器输出(供 App_FOC.c 的 Park/流式打点读取) */
SMOTypedef_t SMO;

/* 快速 arctan 近似: 两次半角精化 + 13 阶奇数级数, 全程只有乘加 + 2 次 sqrtf。
 *   atan(x) = 2·atan(x / (1 + sqrt(1+x²)))
 *   两次精化后 |x| ≤ 0.199, 13 阶级数误差 < 1e-8 rad —— 用 20 万个随机点
 *   与 math.atan2 逐点比对, 最大误差 0.000000°(见 tools 里同款验证脚本)。
 *   ★纯数学近似, 不改变角度口径, 与 tools/verify_smo.py 的结论一致。 */
static float SMO_AtanPos(float x)
{
    float t;

    x = x / (1.0f + sqrtf(1.0f + x * x));    /* 第一次半角精化 */
    x = x / (1.0f + sqrtf(1.0f + x * x));    /* 第二次半角精化 */

    t = x * x;
    /* 4·[ x - x³/3 + x⁵/5 - x⁷/7 + x⁹/9 - x¹¹/11 + x¹³/13 ] */
    return 4.0f * x * (1.0f - t * (0.3333333333f - t * (0.2f - t * (0.1428571429f
             - t * (0.1111111111f - t * (0.0909090909f - t * 0.0769230769f))))));
}

/* 等价于 atan2f(y, x), 但只用乘加和一个比较 */
static float SMO_Atan2Fast(float y, float x)
{
    float ax = fabsf(x);
    float ay = fabsf(y);
    float a;
    float r;

    if (ax < 1.0e-12f && ay < 1.0e-12f) return 0.0f;

    if (ay <= ax) r = SMO_AtanPos(ay / ax);
    else          r = 1.57079632679f - SMO_AtanPos(ax / ay);

    a = (x >= 0.0f) ? r : (3.14159265359f - r);      /* 第二象限 */
    if (y < 0.0f) a = -a;                            /* 第三/四象限 */
    return a;
}

static float SMO_Sat(float x)
{
    if (x >  1.0f) return  1.0f;
    if (x < -1.0f) return -1.0f;
    return x;
}

static void SMO_UpdateCoeffs(void)
{
    smo_F  = expf(-smo_Ts * SMO_RS_DEF / SMO_LS_DEF);
    smo_G  = (1.0f - smo_F) / SMO_RS_DEF;
}

void SMO_SetTs(float ts_s)
{
    /* 抽取运行(每 SMO_DECIM 拍跑一次)时必须把 Ts 设成抽取后的周期 */
    if (ts_s > 1.0e-6f && ts_s < 1.0e-2f) smo_Ts = ts_s;
}

float SMO_GetTs(void)
{
    return smo_Ts;
}

void SMO_Init(void)
{
    if (smo_Ts <= 0.0f) smo_Ts = SMO_TS_DEF;
    SMO_UpdateCoeffs();
    smo_H     = SMO_SLIDE_GAIN;
    smo_delta = SMO_BOUNDARY;
    smo_lag_w = SMO_LAG_WDEF;
    SMO_Reset();
}

void SMO_Reset(void)
{
    i_hat[0] = 0.0f;
    i_hat[1] = 0.0f;
    e_hat[0] = 0.0f;
    e_hat[1] = 0.0f;
    e_lpf[0] = 0.0f;
    e_lpf[1] = 0.0f;
    theta_prev  = 0.0f;
    omega_e     = 0.0f;
    theta_valid = 0u;

    SMO.Input.Ialpha = 0.0f;
    SMO.Input.Ibeat  = 0.0f;
    SMO.Input.Valpha = 0.0f;
    SMO.Input.Vbeta  = 0.0f;
    SMO.Output.Theta    = 0.0f;
    SMO.Output.Speed    = 0.0f;
    SMO.Output.Rpm      = 0.0f;
    SMO.Output.Ealpha   = 0.0f;
    SMO.Output.Ebeta    = 0.0f;
    SMO.Output.Emf      = 0.0f;
    SMO.Output.ThetaEnc = 0.0f;
    SMO.Output.Valid    = 0u;
}

void SMO_SetGain(float k)
{
    if (k > 0.0f && k < 1000.0f) smo_H = k;
}

void SMO_SetBoundary(float delta)
{
    if (delta > 0.1f && delta < 100.0f) smo_delta = delta;
}

void SMO_SetLagW(float rad_per_s)
{
    if (rad_per_s >= 0.0f && rad_per_s < 1.0e6f) smo_lag_w = rad_per_s;
}

float SMO_GetGain(void)     { return smo_H; }
float SMO_GetBoundary(void) { return smo_delta; }
float SMO_GetLagW(void)     { return smo_lag_w; }

/**
 * @brief 滑模观测器运行函数(每拍调一次, Ts = 25us @40kHz)
 *
 * @param smo 滑模观测器结构体指针(Input 由调用方填好)
 */
void SMO_Run(SMOTypedef_t *smo)
{
    float s[2];     /* ε = i − î */
    float w[2];     /* 滑模项 */

    if (smo == 0) return;

    /* 1. 电流误差 ε = 实测 − 估计 */
    s[0] = smo->Input.Ialpha - i_hat[0];
    s[1] = smo->Input.Ibeat  - i_hat[1];

    /* 2. 滑模项 w = k·sat(ε/δ)
     *    ★符号: 观测器方程是 î(k+1) = F·î + G·(u + w), w 取正。
     *      若写成 î(k+1) = F·î + G·(u − w)(或对 ε 取反), 就成了负反馈,
     *      î 会被推到 i±δ, 滑模面根本不存在, e_hat 恒为 0、角度冻结。 */
    w[0] = smo_H * SMO_Sat(s[0] / smo_delta);
    w[1] = smo_H * SMO_Sat(s[1] / smo_delta);

    /* 3. 离散电流观测器(零阶保持) */
    i_hat[0] = smo_F * i_hat[0] + smo_G * (smo->Input.Valpha + w[0]);
    i_hat[1] = smo_F * i_hat[1] + smo_G * (smo->Input.Vbeta  + w[1]);

    /* 4. 滑模面上 w = −e ⇒ e_hat = −w */
    e_hat[0] = -w[0];
    e_hat[1] = -w[1];

    /* 5. 反电动势低通(两轴同一个 α, 相位滞后在 atan2 里抵消) */
    e_lpf[0] = LPF_Step(e_lpf[0], e_hat[0], SMO_EMF_LPF_ALPHA);
    e_lpf[1] = LPF_Step(e_lpf[1], e_hat[1], SMO_EMF_LPF_ALPHA);

    /* 6. 电角度: e = ωe·ψf·(−sinθ, cosθ)
     *    ⇒  θ = atan2(−e_alpha, e_beta)
     *    ★这个符号组合是数值验证过的(tools/verify_smo.py):
     *      它的 dθ̂/dθ = +1(转速符号正确)、波动 ≈0、|E| 与真值吻合;
     *      换成 atan2(+e_alpha, e_beta) 得到的是 −θ(镜像), 角度会随 θ 线性
     *      扫过 ±180°, 速度符号也反 —— 是本工程 Ω/αβ 口径决定的。 */
    smo->Output.Theta = SMO_Atan2Fast(-e_lpf[0], e_lpf[1]);

    /* 7. 电角速度: 角度差分 + 低通。|E| 太小时角度不可信, 速度保持不更新 */
    smo->Output.Ealpha = e_lpf[0];
    smo->Output.Ebeta  = e_lpf[1];
    smo->Output.Emf    = sqrtf(e_lpf[0] * e_lpf[0] + e_lpf[1] * e_lpf[1]);
    smo->Output.Valid  = (smo->Output.Emf >= OBS_EMF_MIN_V) ? 1u : 0u;

    if (theta_valid == 0u)
    {
        theta_prev  = smo->Output.Theta;
        theta_valid = 1u;
        omega_e     = 0.0f;
    }
    else
    {
        float dth = smo->Output.Theta - theta_prev;

        if (dth >  SMO_PI) dth -= SMO_TWO_PI;
        if (dth < -SMO_PI) dth += SMO_TWO_PI;
        theta_prev = smo->Output.Theta;

        if (smo->Output.Valid != 0u)
        {
            omega_e = LPF_Step(omega_e, dth / smo_Ts, SMO_SPD_LPF_ALPHA);
        }
    }

    smo->Output.Speed = omega_e;
    /* 电角速度 → 电机侧机械转速 rpm: ωm = ωe/p, n = ωm·60/2π */
    smo->Output.Rpm   = omega_e / (float)SMO_POLE_PAIRS * 9.54929658551f;

    /* 8. 相位滞后补偿量: 低通(ω_f = α·fs) + 观测器等效极点(ω_obs) 合成一个一阶滞后,
     *    θ̂ 滞后真值 atan(|ωe|/(ω_f+ω_obs))。用当前估计的 |ωe| 算, 带符号(反转自动反号)。
     *    ★ω_obs 是可标定参数(SMO_LAG_WDEF / 命令 lag <rad/s>), 现场用 off 把残差压到 0。 */
    {
        /* ω_f = α·fs。fs 取观测器自己的周期(Ts), 不依赖 App_Config 的控制频率宏 ——
         *   这样 SMO.c 只要 SMO.h + LPF.h 就能编译, 可以原样贴到任何版本的工程里。 */
        float wf = SMO_EMF_LPF_ALPHA / smo_Ts;
        /* 直接带符号算: 反转时自动变成负补偿量, 不用额外取反 */
        float lag = atanf(omega_e / (wf + smo_lag_w));

        smo->Output.Lag = lag;
    }
}
