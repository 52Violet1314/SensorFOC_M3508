#ifndef __SMO_H__
#define __SMO_H__

#include <stdint.h>

/* ======================= M3508 电机参数 =======================
 * 以下参数请按实测标定, 不同批次电机会有差异。
 * 注意: 之前把相电感误写成 25 uH, 导致观测器增益 G=(1-F)/Rs 过大,
 * 电流估计进入 bang-bang 极限环, 反电动势被 ±k 的开关量淹没,
 * 角度直接冻结。这里改用更接近实测的 0.5 mH(约 0.2~1 mH 量级)。
 */
#define SMO_POLE_PAIRS     7u          /* 极对数(14 极 = 7 对) */
#define SMO_RS             0.194f      /* 相电阻 Ω (实测) */
#define SMO_LS             0.000097f   /* 相电感 H (实测 0.097 mH) */
#define SMO_TS             0.000025f   /* 观测周期 s (40 kHz 电流环) */
/* 实测 R/L 带来的离散系数: Ts*R/L = 25us*0.194/97uH = 0.05
 *   F = exp(-0.05) = 0.9512 ,  G = (1-F)/R = 0.2514 A/V  (比旧 0.5mH 时大 2.5 倍!) */

/* ======================= 滑模观测器参数 ======================= */
/* 滑模增益 k(V): 需大于最大反电动势幅值 ωe_max * ψf ≈ 24 V(M3508 满速)。
 * 纯符号函数会把电流估计打成长度为 k/R ≈ 208 A 的极限环, 所以配合
 * 边界层(饱和函数)使用, 消除抖振。 */
#define SMO_GAIN           50.0f
/* 边界层宽度 δ(A): |ε|<δ 时用线性增益 k/δ 代替符号函数, 消除抖振。
 * 离散稳定条件: G*k/δ < 1+F  ⇒  δ > G*k/(1+F) = 0.2514*50/1.9512 = 6.44 A
 * ★实测 L 变小后 G 变大 2.5 倍, 原来的 δ=6A 已经越界(会发散), 故取 8A。
 *   此时闭环极点 |F - G*k/δ| = |0.9512 - 1.571| = 0.62 (稳定, 约 2.6 拍收敛) */
#define SMO_BOUNDARY       8.0f
/* 反电动势一阶低通系数: 保持约 500 Hz 截止 @ 40 kHz (原 0.30 @ 10 kHz)。
 * E_alpha/E_beta 经过同一低通, 相位滞后在 atan2 中相互抵消, 不影响角度。 */
#define SMO_EMF_LPF_ALPHA  0.0785f
/* 电角速度估计一阶低通系数(角度差分, 非 PLL), 保持原时间常数 */
#define SMO_W_LPF_ALPHA    0.0125f

/* ======================= 使用说明 =======================
 * 1. SMO 依靠反电动势观测转子角度, 反电动势幅值 = ωe*ψf。
 *    低速(反电动势太小, 被电流噪声淹没)时角度不可信。
 *    M3508 在电机侧约 > 300~500 rpm(电频率 > 30~50 Hz)时才有较好效果。
 * 2. 该实现不含 PLL, 角度由 atan2(-E_alpha, E_beta) 直接得到,
 *    转速由角度差分 + 低通得到。
 */

typedef struct
{
    /* 电机/离散化参数 */
    float Rs;    /* 相电阻 Ω */
    float Ls;    /* 相电感 H */
    float Ts;    /* 观测周期 s */
    float F;     /* exp(-Ts*Rs/Ls) */
    float G;     /* (1-F)/Rs */
    float Kslide; /* 滑模增益 k(V) */
    /* 电流估计 */
    float I_alpha_Est;
    float I_beta_Est;

    /* 电压估计 */
    float E_alpha_Est;
    float E_beta_Est;

    /* 反电动势估计(低通后) */
    float E_alpha;
    float E_beta;

    float Err_Limit;

    /* 估计转子电角度 rad / 电角速度 rad/s */
    float theta_e;
    float omega_e;

    /* 角度差分(无 PLL)辅助 */
    float theta_prev;
    uint8_t theta_valid;
} SMO_Config_t;

void  SMO_Init(SMO_Config_t *smo, float Rs, float Ls, float Ts);
void  SMO_Reset(SMO_Config_t *smo);
void  SMO_Update(SMO_Config_t *smo,
                 float I_alpha, float I_beta,
                 float U_alpha, float U_beta);
float SMO_GetTheta(const SMO_Config_t *smo);
float SMO_GetOmega(const SMO_Config_t *smo);

#endif
