#include "SMO.h"
#include "FOC_Param.h"
#include "LPF.h"
#include <math.h>

#define SMO_PI      3.14159265359f
#define SMO_TWO_PI  6.28318530718f

/* ============================================================================
 * 滑模观测器(SMO, αβ 坐标系, 观测反电动势 -> 转子电角度/电角速度)
 *
 * 电机模型(每相量, 峰值约定):
 *      L·di/dt = u − R·i − e            e = 反电动势矢量
 *
 * 电流观测器(把滑模项 w 加到电压上):
 *      L·dî/dt = u − R·î + w ,   w = k·sat(ε/δ),   ε = i − î  (实测 − 估计)
 *      离散化(一阶保持):  î(k+1) = F·î(k) + G·[ u(k) + w(k) ]
 *                          F = exp(−Ts·R/L),  G = (1−F)/R
 *
 * 符号说明(重要, 别写反):
 *      电流误差动态 L·dε/dt = −R·ε − e − w
 *      ⇒ ε>0 时 w>0 会把 î 拉上去(收敛); 滑模面 ε≈0 上满足 w = −e
 *      ⇒ 反电动势估计  e_hat = −w
 *      又因 e = ωe·ψf·(−sinθ, cosθ)  ⇒  θ = atan2(−E_alpha, E_beta)
 *
 * 整定约束:
 *  ① 滑模存在条件:  k > |e|_max = ωe_max·ψf
 *       满速 ωe≈6500 rad/s, ψf≈2.1e-3 ⇒ |e|_max ≈ 13.7 V, 取 k = 50 V ✓
 *  ② 边界层离散稳定: |F − G·k/δ| < 1  ⇒  δ > G·k/(1+F)
 *       本例 F≈0.988, G≈0.1 A/V, k=50 ⇒ δ > 2.5 A; 现取 δ = 6 A ✓(留 2.4 倍余量)
 *       想更"硬"的滑模就把 k 和 δ 一起调, 保持 k/δ ≤ (1+F)/G ≈ 20 V/A。
 *  ③ 反电动势低通: α≈0.0785 @40kHz ⇒ fc≈500 Hz; 两个轴共用同一 α, atan2 里相位抵消。
 *
 * 使用注意:
 *  - 每拍调一次(Ts = FOC_TS = 25 µs @40kHz)。
 *  - ★HFI 在跑的时候, 喂进来的 i/u 必须先去注入: 用"相邻两拍平均"的基波电流和
 *    不含注入的指令电压。否则 20 kHz 的注入正好落在 Nyquist 上, 观测器会一直抖。
 *  - 低速(|e| 小)时角度不可信, 建议 >300~500 rpm(电机侧)才用, 或与 HFI 融合。
 * ========================================================================== */

/* 饱和函数: 带边界层的 sign(), 用来消除抖振 */
static float SMO_Sat(float x)
{
    if (x >  1.0f) return  1.0f;
    if (x < -1.0f) return -1.0f;
    return x;
}

void SMO_Init(SMO_Config_t *smo, float Rs, float Ls, float Ts)
{
    if (Rs < 1.0e-3f) Rs = FOC_RS;   /* 参数非法时退回 FOC_Param.h 里的值 */
    if (Ls < 1.0e-6f) Ls = FOC_LS;
    if (Ts <= 0.0f)   Ts = FOC_TS;

    smo->Rs = Rs;
    smo->Ls = Ls;
    smo->Ts = Ts;
    smo->F  = expf(-Ts * Rs / Ls);
    smo->G  = (1.0f - smo->F) / Rs;
    smo->Kslide    = SMO_GAIN;       /* k (V) */
    smo->Err_Limit = SMO_BOUNDARY;   /* δ (A), 边界层宽度 */
    SMO_Reset(smo);
}

void SMO_Reset(SMO_Config_t *smo)
{
    smo->I_alpha_Est = 0.0f;
    smo->I_beta_Est  = 0.0f;
    smo->E_alpha_Est = 0.0f;
    smo->E_beta_Est  = 0.0f;
    smo->E_alpha     = 0.0f;   /* 同时是反电动势低通的状态 */
    smo->E_beta      = 0.0f;
    smo->theta_e     = 0.0f;
    smo->omega_e     = 0.0f;   /* 同时是速度低通的状态 */
    smo->theta_prev  = 0.0f;
    smo->theta_valid = 0u;
}

void SMO_Update(SMO_Config_t *smo,
                float I_alpha, float I_beta,
                float U_alpha, float U_beta)
{
    float eps_a = I_alpha - smo->I_alpha_Est;   /* ε = 实测 − 估计 */
    float eps_b = I_beta  - smo->I_beta_Est;
    float w_a, w_b, dth;

    /* 1. 滑模校正项 w = k·sat(ε/δ) */
    w_a = smo->Kslide * SMO_Sat(eps_a / smo->Err_Limit);
    w_b = smo->Kslide * SMO_Sat(eps_b / smo->Err_Limit);

    /* 2. 离散电流观测器: î(k+1) = F·î + G·(u + w), w 迫使 î 追上 i */
    smo->I_alpha_Est = smo->F * smo->I_alpha_Est + smo->G * (U_alpha + w_a);
    smo->I_beta_Est  = smo->F * smo->I_beta_Est  + smo->G * (U_beta  + w_b);

    /* 3. 滑模面上 w = −e, 于是反电动势原始估计 e = −w(含抖振, 只用于打点观察) */
    smo->E_alpha_Est = -w_a;
    smo->E_beta_Est  = -w_b;

    /* 4. 一阶低通取出反电动势基波(两轴同一 α, 相位滞后在 atan2 里抵消) */
    smo->E_alpha = LPF_Step(smo->E_alpha, smo->E_alpha_Est, SMO_EMF_LPF_ALPHA);
    smo->E_beta  = LPF_Step(smo->E_beta,  smo->E_beta_Est,  SMO_EMF_LPF_ALPHA);

    /* 5. 电角度: e = ωe·ψf·(−sinθ, cosθ)  ⇒  θ = atan2(−E_alpha, E_beta) */
    smo->theta_e = atan2f(-smo->E_alpha, smo->E_beta);

    /* 6. 电角速度: 电角度差分 + 一阶低通(无 PLL) */
    if (smo->theta_valid == 0u)
    {
        smo->theta_prev  = smo->theta_e;
        smo->theta_valid = 1u;
        smo->omega_e     = 0.0f;
    }
    else
    {
        dth = smo->theta_e - smo->theta_prev;
        if (dth >  SMO_PI) dth -= SMO_TWO_PI;   /* 归一化到 [-π, π) */
        if (dth < -SMO_PI) dth += SMO_TWO_PI;
        smo->theta_prev = smo->theta_e;
        smo->omega_e = LPF_Step(smo->omega_e, dth / smo->Ts, SMO_W_LPF_ALPHA);
    }
}

float SMO_GetTheta(const SMO_Config_t *smo)
{
    return smo->theta_e;
}

float SMO_GetOmega(const SMO_Config_t *smo)
{
    return smo->omega_e;
}
