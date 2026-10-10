#ifndef __SMO_H__
#define __SMO_H__

#include <stdint.h>

/* ============================================================================
 * SMO.h —— 滑模观测器(Sliding Mode Observer)
 *
 *   αβ 坐标系下的电流观测器, 用滑模项强迫估计电流跟随实测电流,
 *   滑模面上的等效控制量 = 反电动势 → 转子电角度 / 电角速度。
 *
 *   电机模型:      L·di/dt = u − R·i − e      (每相量, 峰值约定)
 *   电流观测器:    L·dî/dt = u − R·î + w      w = k·sat(ε/δ),  ε = i − î
 *   误差动态:      L·dε/dt = −R·ε − e − w     ⇒ 滑模面上 w → −e
 *   反电动势估计:  e_hat = −w(取反!), 低通后 e_hat ≈ e
 *   角度:          e = ωe·ψf·(−sinθ, cosθ)
 *                  ⇒ 本工程(以及 M3508 磁编码器)口径下 θ = atan2(−e_alpha, e_beta)
 *                     ★已用 tools/verify_smo.py 逐点数值验证: 该式的 dθ̂/dθ = +1
 *                       (转速符号正确)、角度波动 ~0、|E| 与真值一致;
 *                       写成 atan2(+e_alpha, e_beta) 会得到 −θ(镜像), dθ̂/dθ = −1,
 *                       角度随 θ 线性扫过 ±180°, 转速符号也是反的 —— 别"顺手改符号"。
 *   相位滞后:      θ̂ 比真值滞后 atan(|ωe|/(ω_f+ω_obs)), 由 SMO_Output.Lag 给出,
 *                  补偿量已按数值实验标定(极差 < 0.05° @600~6000rpm)
 *
 *   ★符号是这条链路上最容易写错的地方(写反的后果不是"精度差", 而是
 *     e_hat 恒为 0 / 角度冻结), 改动前先看上面的推导。
 *
 *   参数整定约束:
 *     ① 滑模存在条件: k > |e|max = ωe_max·ψf。12V 台架满速 |e| ≈ 5V,
 *        24V 满速 ≈ 14V → 取 k = 50V 足够。
 *     ② 边界层离散稳定: |F − G·k/δ| < 1。F=0.9512, G=0.2514 A/V,
 *        k/δ = 50/8 = 6.25 → 极点 0.62(稳定)。δ 调小要重新核算!
 *     ③ 两个轴的 EMF 低通必须用同一个 α(相位滞后在 atan2 里抵消)。
 * ========================================================================== */

/* 离散模型: F = exp(−Ts·R/L), G = (1−F)/R。
 *   R = 0.194Ω / L = 97uH / Ts = 25us(@40kHz)
 *     → Ts·R/L = 0.05,  F = 0.9512,  G = 0.2514 A/V
 *   ★若把电流环频率从 40kHz 改掉, F/G 必须跟着变(在 SMO_Init 里按 Ts 算) */
#define SMO_POLE_PAIRS      7u          /* 极对数(= FOC_POLE_PAIRS) */
#define SMO_RS_DEF          0.194f      /* 相电阻 Ω(实测, 与 FOC_RS 一致) */
#define SMO_LS_DEF          0.000097f   /* 相电感 H(实测 0.097mH) */
/* 兜底观测周期(实际由 App_FOC.c 调 SMO_SetTs(抽取周期) 覆盖, 见那里的 SMO_DECIM)。
 *   ★必须与 APP_CTRL_HZ 一致: 20kHz -> 50us; 40kHz -> 25us。
 *     SMO.h 故意不包含 App_Config.h(保持头文件自包含), 所以这里写数字。 */
#define SMO_TS_DEF          (1.0f / 20000.0f)

/* 滑模增益 k(V): 必须大于最大反电动势幅值 */
#define SMO_SLIDE_GAIN      50.0f
/* 边界层宽度 δ(A): 饱和函数代替符号函数, 消除抖振。
 *   ★δ 必须随观测周期 Ts 等比放大 —— 前向欧拉离散下 G=(1−F)/R 随 Ts 增大,
 *     决定稳定性的无量纲量是 G·k/δ, 线性区闭环极点是 F − G·k/δ, 必须落在单位圆内。
 *     (已经用 50/12/16/20 四组值算过, 不是拍脑袋)
 *
 *     控制频率   Ts      F        G       δ      G·k/δ   闭环极点   判定
 *     40kHz      25us    0.9512   0.2514  8A     1.571   −0.620    ✅
 *     20kHz      50us    0.9048   0.4905  8A     3.066   −2.161    ★发散
 *     20kHz      50us    0.9048   0.4905  12A    2.044   −1.139    ★发散
 *     20kHz      50us    0.9048   0.4905  16A    1.533   −0.628    ✅
 *     20kHz      400us*  0.4493   2.8385  16A    8.871  −8.42      ★发散
 *       (* 400us = 抽取 8, 那种配置下 δ 需要 90A 以上, 已经放弃抽取)
 *
 *   稳定下限 δ > G·k/(1+F): 40kHz 是 6.4A, 20kHz(50us) 是 12.9A。
 *   ⇒ 20kHz 保持观测器每拍运行(SMO_DECIM=1), δ 用 16A。 */
#define SMO_BOUNDARY        16.0f
/* EMF 一阶低通系数(40kHz): α = 1−exp(−2π·fc/fs), fc≈900Hz → 0.1317
 *   取 0.14 → fc ≈ 960Hz, 基波(最高约 250Hz)相位滞后 ~15°, 两轴抵消不影响角度 */
#define SMO_EMF_LPF_ALPHA   0.14f
/* 电角速度一阶低通系数(40kHz): fc ≈ α·fs/2π = 90Hz, 时间常数 ~1.8ms */
#define SMO_SPD_LPF_ALPHA   0.014f
/* 滞后补偿用的观测器等效带宽 rad/s: 观测器整体等效一个一阶滞后,
 *   θ_true − θ_est ≈ atan(|ωe| / (ω_f + ω_obs)),  ω_f = α·fs = 5600 rad/s
 *   ★400 rad/s 是按 tools/verify_smo.py 在 600~6000rpm 逐点标定出来的
 *     (补偿后偏置极差 < 0.05°); 现场若发现残差仍随转速漂, 用 lag 命令微调 */
#define SMO_LAG_WDEF        400.0f

typedef struct{
    float Ialpha;
    float Ibeat;
    float Valpha;
    float Vbeta;
}SMO_Input;

typedef struct{
    float Theta;      /* 估计电角度 rad, [-pi, pi] */
    float Lag;        /* 需要补的相位滞后 rad(≥0), 加在 Theta 上与真值对齐 */
    float Speed;      /* 估计电角速度 rad/s(带符号) */
    float Rpm;        /* 估计电机侧机械转速 rpm(供打点用) */
    float Ealpha;     /* 低通后的反电动势 V */
    float Ebeta;
    float Emf;        /* |E| 幅值 V, 用于有效性判断 */
    float ThetaEnc;   /* 同一拍的编码器电角度(对照真值) */
    uint8_t Valid;    /* 1 = |E| 足够大, 角度可信 */
}SMO_Output_t;

typedef struct{
    SMO_Input  Input;
    SMO_Output_t Output;
}SMOTypedef_t;

extern SMOTypedef_t SMO;

void  SMO_Init(void);
void  SMO_Reset(void);
void  SMO_Run(SMOTypedef_t *smo);
/* 观测周期可配: 抽取运行(App_FOC.c 的 SMO_DECIM)时必须把 Ts 设成抽取后的周期,
 * 否则 F/G 系数和"角度差分求速度"都会算错。要在 SMO_Init() 之前调用。 */
void  SMO_SetTs(float ts_s);
float SMO_GetTs(void);
void  SMO_SetGain(float k);
void  SMO_SetBoundary(float delta);
void  SMO_SetLagW(float rad_per_s);   /* 标定用: 补偿公式里的观测器等效带宽 */
float SMO_GetGain(void);
float SMO_GetBoundary(void);
float SMO_GetLagW(void);

#endif
