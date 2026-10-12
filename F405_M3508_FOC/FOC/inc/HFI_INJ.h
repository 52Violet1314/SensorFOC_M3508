#ifndef __HFI_INJ_H__
#define __HFI_INJ_H__

#include <stdint.h>

/* ============================================================================
 * HFI_INJ.h —— 凸极法高频方波注入(Vd 脉振) + 半周期同步解调 + PLL
 *
 *   算法来源: 参考工程的 HFI_INJ.c —— Vd 上叠方波高频电压, 半周期同步平均解调,
 *   PI + 积分出角(PLL), 误差上用 2·ωe 自适应陷波。本文件把它适配到本工程:
 *
 *     · 本工程 10 kHz(APP_CURRENT_PID_DT = 1e-4)。注入极性每 5 拍翻一次
 *       ⇒ 方波基频 1 kHz, 解调/环路更新率 2 kHz(与原代码注释里的"更新率 2kHz"一致)。
 *     · 浮点、伏特/安培单位(原工程是 Q10 定点 + ControllerStruct, 本工程没有这两样)。
 *     · ★**只观测不驱动**: 注入叠加在编码器坐标系的 Vd 上, PLL 自己估角,
 *       不参与 Park/DePark, 不改电流环给定 —— 与 SMO 同一个原则(驱动角永远来自编码器)。
 *
 *   为什么要有它:
 *     SMO 靠反电动势, |E| < 0.3V(≈478 电机 rpm)以下观测角不可信;
 *     HFI 靠凸极(Ld≠Lq), **静止就能出角**, 正好补这一段。而且它是第三条独立
 *     测量路径(编码器 / SMO / HFI), 三方对上才算真对齐。
 *
 *   ⚠ 物理前提(先读再上电):
 *     ① 这套方法完全靠 Ld≠Lq。表贴磁钢的 M3508 凸极率通常只有几个百分点 ——
 *        太小就出不来角。使能后先看 `# hfi` 行的 kth/sal/snr, **snr < 3 就别信它的角**。
 *     ② 本机 L/R 时间常数 τ = L/R = 97uH/0.194Ω ≈ 0.5 ms, 与半周期 T=0.5 ms 同量级:
 *        已经不是"纯感性"区。半周期平均解调量 F(τ) = 1 − (τ/T)(1−e^(−T/τ)) = 0.368 @T=τ,
 *        对电感的相对灵敏度 dF/F = −0.718·(ΔL/L)(纯感性区是 −0.5·(ΔL/L)) —— 反而略高,
 *        但绝对幅度受 R 限制。`sal%` 就是按这个 0.718 反推的(见 HFI_SAL_K)。
 *        这些常数由 tools/verify_hfi.py 离线核对, 不要手改。
 *     ③ 凸极法只能定到**模 π 的轴**(sin2Δθ 在 Δ=0 与 Δ=π/2 都为零)。
 *        报告里与编码器比较时会折到 ±π/2 —— 只影响显示, 不碰控制。
 *
 *   符号约定(两次踩过的地方, 别"顺手改符号"):
 *     注入: 方波电压矢量沿 **PLL 自己估计的 d 轴 θ̂** 注入(与 mf_HFI_Observer.m 一致)!
 *           ★不能沿编码器轴注入 —— 那样电流矢量就跟着编码器轴走, 解调量变成
 *             ∝sin(Δθ)(各向同性项), 凸极根本没参与, 观测器等于在跟踪自己注入的轴,
 *             什么也验证不了。判据见 tools/verify_hfi.py 的 [1b] 节(err(Δ) 幅度
 *             必须与凸极率成正比: 20% vs 2% 应是 10 倍)。
 *     解调: Park 到 θ̂, 取 q 轴: Iq = −Iα·sinθ̂ + Iβ·cosθ̂
 *           (与 Transformer.c 的 ParkTransformer 口径一致);
 *     误差: err_raw = sign_prev · mean(Iq over half period), err_raw ≈ Kth·sin(2Δθ)
 *           (Δθ = θenc − θ̂) ⇒ Δθ>0 时 ω>0 ⇒ θ̂ 追上 θenc = 负反馈 ✓
 *     tools/verify_hfi.py 的第一项自检就是这条极性(写反会一路飞掉)。
 * ========================================================================== */

/* ---------------------------------------------------------------------------
 * 1. 时基
 * ------------------------------------------------------------------------- */
#define HFI_FS              10000.0f    /* 采样/控制频率 Hz(= APP_CURRENT_PID_DT_INV) */
#define HFI_TS              0.0001f     /* 1/HFI_FS s */
#define HFI_HALF_TICKS      5u          /* 极性每 5 拍翻一次 */
#define HFI_TS_LOOP         0.0005f     /* 解调/环路周期 = HALF_TICKS·TS = 0.5ms → 2kHz */
#define HFI_DEMOD_HZ        (1.0f / HFI_TS_LOOP)

/* ---------------------------------------------------------------------------
 * 2. 注入电压
 *   幅度估算(半周期 T, 忽略基波):
 *     I_hf ≈ (U/R)·F(τ),  F(τ) = 1 − (τ/T)(1−e^(−T/τ)),  τ = L/R = 0.5ms, T = 0.5ms
 *          = 0.368  ⇒  I_hf ≈ 1.9·U  [A]      (U=0.30V → 约 0.57A 高频电流)
 *   太小 = 信噪比不够; 太大 = 电流纹波/发热/噪声都上来。0.30V 起步, 按 `# hfi` 的
 *   id(A) 与实测电流纹波再调。硬上限还受 SVPWM 线性区(≤ 0.5·vdc/√3)限制。
 * ------------------------------------------------------------------------- */
#define HFI_INJ_V_AMP       0.30f       /* 默认注入幅值 V */
#define HFI_INJ_V_MIN       0.05f
#define HFI_INJ_V_MAX       1.20f       /* 硬上限(≈2.3A 高频电流) */
#define HFI_INJ_V_SPAN      0.50f       /* 另受 0.5·vdc/√3 限制 */
#define HFI_I_LIMIT         2.50f       /* 解调出的高频电流 |id| 超过它就撤注入+报故障 */

/* ---------------------------------------------------------------------------
 * 3. 前端带通(RBJ biquad, 中心 = 注入频率 1kHz, fs = 10kHz)
 *     w0 = 2π·f0/fs = 0.6283185,  Q = 3
 *     α  = sin(w0)/(2Q) = 0.0979642,  a0 = 1+α = 1.0979642
 *     b0 = α/a0, b1 = 0, b2 = −α/a0, a1 = −2cos(w0)/a0, a2 = (1−α)/a0
 *   中心处 |H| = 1、相位 = 0(带通的意义就在这: 解调不会因滤波而偏相)。
 *   ★两级级联(与 mf_HFI_Observer.m 一致): 单级 Q=3 在 116Hz(1000rpm 基波)处
 *     只压 ~15dB, 转起来基波会淹掉解调; 级联后 ~30dB。
 *   系数用 tools/verify_hfi.py 反算核对(公式→数值必须一致)。
 * ------------------------------------------------------------------------- */
#define HFI_BPF_Q           3.0f
#define HFI_BPF_B0          0.08922350f
#define HFI_BPF_B1          0.0f
#define HFI_BPF_B2         -0.08922350f
#define HFI_BPF_A1         -1.47366733f
#define HFI_BPF_A2          0.82155300f

/* ---------------------------------------------------------------------------
 * 4. PLL
 *   误差先按 Kth 归一化成 sin(2Δθ) ∈ [−1,1], 再进 PI:
 *       ω = Kp·ε + ∫Ki·ε dt
 *   闭环特征方程 s² + 2Kp·s + 2Ki = 0 ⇒ ωn = √(2Ki) = 223.6 rad/s(35.6Hz),
 *       ζ = Kp/ωn = 1.12  —— 增益直接取自 mf_HFI_Observer.m(KP_H=250/KI_H=25000),
 *       它那边也是"归一化后进 PI", 所以数值可以照搬。
 *   ★环路只在**有新解调值**时跑一步(2kHz)。原代码每拍都进 PI 而 err 每 5 拍才更新,
 *     等效增益 ×5 —— 这是本文件相对参考实现的修正之一。
 * ------------------------------------------------------------------------- */
#define HFI_PLL_KP          250.0f
#define HFI_PLL_KI          25000.0f
#define HFI_PLL_I_LIMIT     200.0f      /* 积分限幅 rad/s(保证 |ω| 有界, 便于失锁后恢复) */
#define HFI_OMEGA_LPF_ALPHA 0.4665f     /* fc=200Hz @2kHz: 1−exp(−2π·200/2000) */

/* ---------------------------------------------------------------------------
 * 5. 2·ωe 自适应陷波(去 HFI 固有谐波扰动)
 *   中心频率 = 2·|ωe|/(2π) = |Omega_e_F|/π Hz; 更新率 2kHz(Nyquist 1kHz)。
 *   Q = 1, 系数慢速自适应(~50ms): α = 1−exp(−1/(0.05·2000)) ≈ 0.01。
 *
 *   ★★ 默认**关**(hfi_notch_en = 0), 这是离线自检的结论, 不是偷懒:
 *     静止/低速时 ωe≈0 ⇒ 陷波被限幅压到 HFI_NOTCH_F_MIN=5Hz, 而 PLL 环路带宽是
 *     ~35Hz —— 陷波正好落在环路带宽**里面**, 把相位余量吃光, 实测(20%凸极,1LSB):
 *        陷波 on  → 末段误差 −29°, RMS 19°(振)  ← 换任何增益都一样
 *        陷波 off → 末段误差 +0.3°, RMS 0.3°(稳)
 *     它本来是用来压 2·fe 处那个 HFI 固有扰动的, 只有当 2·fe 远高于环路带宽
 *     (也就是转起来以后)才有意义 —— 而那时候该用 SMO 了。想试验: `hfin 1`。
 * ------------------------------------------------------------------------- */
#define HFI_NOTCH_Q         1.0f
#define HFI_NOTCH_F_MIN     5.0f
#define HFI_NOTCH_F_MAX     400.0f
#define HFI_NOTCH_ADAPT     0.01f

/* ---------------------------------------------------------------------------
 * 6. 解调增益 Kth [A] —— err_raw = Kth·sin(2Δθ)
 *   本工程是"方波 + 半周期平均", 不是正弦载波, 所以不套 mf_HFI_Observer.m 的
 *   Kth = Vh(1/Ld−1/Lq)/(4ωh), 而是**实测自标定**:
 *     固件每次解调时用编码器角算出真实的 sin(2Δθ), 对 err_raw = Kth·sin(2Δθ)
 *     做递推最小二乘 —— 无模型(不依赖 Ld/Lq/R/滤波器任何假设), 顺带给出拟合
 *     残差=噪声, 于是 snr = Kth/噪声 就能直接判"这点凸极够不够用"。
 *
 *   ★三个必须注意的点(离线自检 + 现场跑飞各踩出来一次):
 *     ① **两段式标定(必须)**: 使能后先在**标定窗**里把 θ̂ 钉在编码器 ±45° 两个点上
 *        (环路不闭), 用这两点把解调偏置和增益一起解出来(注意符号: err 的定义是
 *        err = +Kth·sin(2(θenc−θ̂)), 所以 +45° 点是 b−Kth、−45° 点是 b+Kth):
 *            Kth = (err₋ − err₊)/2,   b = (err₊ + err₋)/2
 *        窗内**用快系数**、窗后**冻结**(见 ②)。★没有这一步会怎样(实测): 环路拿瞬态
 *        样本算出的 Kth 直接闭环 → 45° 初始误差 + 转子在转 → 积分器被顶到限幅 →
 *        w 钉在 ±450rad/s、kth 掉到下限、角度全是垃圾。
 *     ② 标定结束后 **Kth 冻结**, 不再更新: 锁上以后 Δθ→0, 拿 0/0 去拟合会把 Kth
 *        越拟越小(增益反而放大), 失锁时的"慢速修正"同样会被垃圾数据带跑 —— 所以
 *        只在标定窗里估, 之后一律用冻结值。偏置 b 只报告、不进环路。
 *     ③ 交接时把 **θ̂ 直接对到编码器上(零误差)、积分器预置成编码器速度**(前馈):
 *        这样环路一上来就在平衡点附近, 不会 wind-up。★注意: 这一步只借编码器的
 *        "速度"做前馈, **角度仍然是 HFI 自己解调出来的**, 独立性不受影响。
 * ------------------------------------------------------------------------- */
#define HFI_CAL_OFFSET      0.7853981634f  /* 标定窗里 θ̂ 相对编码器钉住的偏置(+45°) */
#define HFI_CAL_HALF        16u           /* 每个点用多少次解调(16×0.5ms = 8ms) */
#define HFI_CAL_SKIP        8u            /* 每个点前几次丢掉(换点后电流/带通要 4ms 才稳) */
#define HFI_CAL_CYCLES      2u            /* 来回几轮(2 轮 = 48 次解调 = 24ms) */
#define HFI_CAL_DEMODS      (HFI_CAL_HALF * 2u * HFI_CAL_CYCLES)
/* 失锁检测: 环路一直在饱和(|eps|≥0.95)超过这么多拍就判失锁 → 撤注入 + flt=2。
 *   正常捕获阶段饱和只有几 ms, 所以门限给足(400×0.5ms = 200ms)。 */
#define HFI_NOLOCK_EPS      0.95f
/* ★★ 起动窗(交接后的软起动) —— 现场"交接后 8ms 就顶死在 ±450rad/s"的直接对策。
 *   机理: 标定窗把注入轴钉在 θenc±45° 上, 交接瞬间注入轴**跳 45°** ⇒ 带通(1kHz)
 *   和半周期平均都还在暂态里, 头几次解调的误差是**满量程**(±kth 以上) ⇒
 *   `I += Ki·eps·Ts` 一拍就写进 12.5rad/s, 十几拍就把积分器顶到 200 的上限;
 *   之后轴自己在转, 误差变成关于 0 对称的来回摆 ⇒ eps 均值≈0 ⇒ **积分器再也下不来**
 *   (现场: 顶在 ±450 直到撤注入, 误差一直在 ±90° 扫)。
 *   所以交接后先用**纯比例 + 小限幅**把轴拉到零误差附近(暂态那几拍不许写积分器),
 *   40ms 后再放开 PI, 积分器从 0 起步。
 *   现场佐证: 第二次扫描转完轴后曲线已经是干净的 b+K·sin2Δ(C=0.0135, snr=10.4),
 *   环路**照样** 8ms 顶死 ⇒ 说明剩下的唯一原因就是这个暂态。 */
#define HFI_START_DEMODS    80u        /* 起动窗长度(解调次数, 2kHz ⇒ 40ms) */
#define HFI_START_W_MAX     25.0f      /* 起动窗内 |ω| 上限(rad/s); 越小越稳, 太小学不动 */
#define HFI_RAIL_RETRY      2u         /* 跑飞先"重开标定窗(=重新交接+软起动)"重试几次 */
/* 跑飞确认时长(解调次数, 2kHz ⇒ 150 ≈ 75ms)。
 *   |ω| 钉在 Kp+I_LIMIT 上时这是**无歧义**的饱和, 不需要数很久; 而正常捕获
 *   顶多饱和几 ms。下面这条路径还必须能自动"重试/转静态扫描"(见 HFI_INJ.c 4b)。 */
#define HFI_NOLOCK_DEMODS   150u
#define HFI_RAIL_FRAC       0.90f      /* |ω| 到限幅的这个比例就算"钉在限幅上"(见 4b) */
/* ★★ PLL 输出的**硬限幅**(rad/s)。
 *   原来只靠积分器限幅(PLL_I_LIMIT=200), 于是 Kp+I 能到 450rad/s = 71Hz —— 而注入
 *   载波半周期是 2kHz、环路带宽 ~35Hz: 一旦跑飞, 注入轴就以**远高于环路带宽**的速度
 *   自转, 半周期平均看到的是一个快速旋转的电压矢量(每半周期转 12.6°), 解调出来的是
 *   被这层旋转"整流"过的量 —— 现场证据: 跑飞期间 iq 一直是**同号大偏置**(+0.16…+0.52A),
 *   而静态扫描在同一点量到的是 −0.19A ⇒ 说明这个误差项**与轴的转速同向**, 起负阻尼,
 *   环路自己把自己推着转(正反馈)。
 *   所以把 ω 钉在 120rad/s(≈19 电机rpm, 覆盖 HFI 的实际适用段)以内:
 *     · 负阻尼项直接小 3.75 倍, 给"能不能锁"留出余量;
 *     · 万一还是锁不住, 失败也是有界的、并且能被下面的失锁判据认出来(不会悄悄给错角度)。 */
#define HFI_PLL_W_LIMIT     120.0f
/* ★★ 环路阻尼项 —— 对付"解调误差与注入轴转速**同向**"这一项(现场实测的负阻尼)。
 *   2026-10-11 现场日志量出来的(电机静止, 标定 bias=+0.0642A, kth=0.1826A):
 *     ω=+120rad/s 时 iq≈+0.19A ⇒ eps≈+0.69 |  ω=−120rad/s 时 iq≈−0.08A ⇒ eps≈−0.79
 *   两边都是"误差推着轴往同方向转" ⇒ 环路自己给自己加力 ⇒ 积分器在 ±200 之间反复
 *   顶死、注入轴在 ±120 之间来回扫(日志里 w 恒为 ±120、err 一直扫就是这个)。
 *   物理来源: 注入轴在转时 RL 负载(ωL/R≈3)的**正交分量**会按 ∝dθ̂/dt 漏进同相解调
 *   通道, 同步解调于是量到一个与轴转速成正比的假误差。
 *   对策 = 在误差里**减掉 Kd·ω**(纯阻尼): 只要 Kd 大于实测那个系数, 这一项就从负阻尼
 *   变正阻尼。实测系数 c ≈ 0.085A/120 ≈ 7.1e-4 A·s/rad, 默认取 2 倍留余量。
 *   ★这一项随机器/母线电压/注入幅值变, 所以留了在线旋钮 `hfikd <值>`(0 = 关)。 */
#define HFI_PLL_KD          1.5e-3f     /* A·s/rad */
/* ★适用转速上限(电角速度 rad/s)。实测(tools/verify_hfi.py [0b], 20% 凸极, 现标定窗):
 *     0Hz(静止) RMS 0.26°/均值 −0.03°  |  5Hz(43rpm) −2.3°  |  10Hz(86rpm) −4.4°
 *     24Hz(205rpm) −10.3°
 *   偏差随转速长 —— 那是解调的正交泄漏(注入轴在转, 半周期平均不再是纯载波),
 *   标定窗只能测掉其中的常数部分。**超过门限就明确拒绝**(flt=3 + 撤注入),
 *   不给一堆"看着像角度"的垃圾。HFI 的意义本来就在 SMO 的 478rpm 以下。
 *   (门限处偏差只有 −4.4°, 余量还有; 想放宽就改这里, 并重跑 [0b] 看那张表。) */
#define HFI_MAX_W           62.8f       /* ≈10Hz 电 ≈ 86 电机 rpm */
/* 编码器角速度估计的单拍跳变上限(rad): 用来挡"编码器毛刺/长期没跑导致的陈旧差分"。
 *   正常最高转速(电机侧 3000rpm → 电 2200rad/s)单拍只走 0.22rad, 取 0.30 留余量;
 *   超过就丢掉这一拍并重新起算(现场日志里 w=-1361rad/s 就是这么来的)。 */
#define HFI_WENC_DW_MAX     0.30f
#define HFI_KTH_DEF         0.0500f
#define HFI_KTH_MIN         0.003f
#define HFI_KTH_MAX         1.000f
#define HFI_KTH_FORGET      0.05f       /* 未冻结时的 RLS 遗忘系数(2kHz 下窗 ~20 拍 = 10ms) */
#define HFI_KTH_FORGET_HOLD 0.0005f     /* 冻结后失锁时的慢速系数 */
#define HFI_KTH_S2_GATE     0.30f       /* |sin2Δθ| 门限: 小于它不参与拟合 */
#define HFI_KTH_DEN_MIN     0.10f       /* Σsin²(2Δθ) 累积到这么多才发布 Kth */
/* sal% 反推系数: 凸极率 ΔL/L ≈ (Kth/|id|) / HFI_SAL_K。
 *   由 tools/verify_hfi.py 仿真按"当前这两点法"标定(2%~20% 平均 0.80)。
 *   ⚠ 各凸极率下单点值 0.62~1.19(小的凸极率偏高) —— 所以 sal% 只能当**数量级**
 *     看, 判"够不够用"要看 snr, 别拿 sal% 当真值。 */
#define HFI_SAL_K           0.80f

/* ---------------------------------------------------------------------------
 * 7. 截止/时基常量
 * ------------------------------------------------------------------------- */
#define HFI_PI              3.14159265359f
#define HFI_TWO_PI          6.28318530718f
#define HFI_HALF_PI         1.57079632679f
#define HFI_NPP             7.0f        /* 极对数(= SMO_POLE_PAIRS) */

/* ===========================================================================
 * 类型
 * ======================================================================== */

/* 二阶 IIR(RBJ biquad, Direct-Form I) */
typedef struct
{
    float b0, b1, b2, a1, a2;
    float x1, x2, y1, y2;
} BPF_2nd_t;

/* 自适应陷波(中心频率随 ωe 变) */
typedef struct
{
    float Q;
    float w_cos;      /* 当前 cos(w0), 慢速自适应 */
    float alpha;      /* 当前 sin(w0)/(2Q) */
    float a0_inv;
    float x1, x2, y1, y2;
} adaptive_notch_t;

/* PLL 状态。字段名沿用参考实现, 便于对照阅读 */
typedef struct
{
    float Kp, Ki;
    float Omega_e;          /* PI 输出角速度 rad/s */
    float Omega_e_F;        /* 低通后角速度 rad/s */
    float I_Partern;        /* 积分项 */
    float P_Partern;        /* 比例项 */
    float Thetae;           /* 估计电角度 rad [0,2π) */
    float sin_PLL, cos_PLL;
    float x;                /* Park 中间量 */
    float Iq_pll_hf;        /* 本拍 q 轴高频电流 A */
    float Iq_pll_hf_prev;
    float err_raw;          /* 半周期平均 + 极性加权后的误差 A */
    float err_lpf;          /* = err_raw(半周期平均本身就是低通) */
    float err_notched;      /* 陷波后, 进 PI */
    float notch_fc_hz;      /* 当前陷波中心 Hz */
    float Iq_acc;           /* 半周期累加 */
    float Id_acc;
    uint16_t acc_cnt;
    int8_t sign_prev;
    int32_t speed;          /* Q10 电机侧 rpm(沿用参考实现的口径) */
    /* ---- 本工程新增: 诊断/自标定 ---- */
    float Id_demod;         /* 解调出的 d 轴高频电流 A(载波电平) */
    float Iq_demod;         /* 解调出的 q 轴高频电流 A(同一时刻的信号) */
    float theta_err;        /* θ̂ − 编码器角, 已折到 ±π/2 rad(模 π 口径) */
    float kth_num, kth_den; /* RLS: Kth = Σ(err·sin2Δθ)/Σsin²2Δθ(仅标定窗内用) */
    float kth_res;          /* RLS 残差(噪声功率) */
    uint8_t kth_done;       /* 1 = 标定窗结束, Kth 已冻结 */
    uint8_t cal_run;        /* 1 = 正在标定窗内(θ̂ 被钉在编码器 ±45° 上) */
} PLL_control;

/* 对外状态(命令/打点/报告用) */
extern volatile uint8_t hfi_enable;    /* 1 = 注入开 */
extern volatile uint8_t hfi_fault;     /* 0=正常; 1=高频过流; 2=失锁(已撤注入) */
extern volatile uint8_t hfi_notch_en;  /* 1 = 陷波开(★默认 0, 见上面第 5 节) */
extern float    hfi_kth_bias;          /* 标定窗量到的解调偏置 A(诊断用, 不进环路) */
extern float    hfi_w_enc;             /* 编码器电角速度 rad/s(交接前馈 + 对照) */
extern float    hfi_v_inj;             /* 注入幅值(V, 命令可改) */
extern float    hfi_v_lim;             /* 本拍实际用的幅值(可能被 vdc 限幅) */
extern float    hfi_kth;               /* 环路归一化用的 Kth [A] */
extern float    hfi_kth_meas;          /* 实测自标定的 Kth [A] */
extern float    hfi_kth_rms;           /* 拟合残差 RMS [A] = 噪声 */
extern float    hfi_snr;               /* hfi_kth_meas / 噪声 */
extern float    hfi_sal_pct;           /* 反推的凸极率 %(模型估计, 看数量级用) */
extern float    hfi_us_peak;           /* HFI 每拍耗时峰值 us(DWT CYCCNT 实测) */
extern uint32_t hfi_clamp_cnt;         /* 被 vdc 限幅的拍数 */
extern uint32_t hfi_ticks;             /* 注入开启后累计拍数 */

/* ===========================================================================
 * 接口
 * ======================================================================== */
void  HFI_Init(void);
void  HFI_SetEnable(uint8_t on);
void  HFI_SetAmpl(float volts);
void  HFI_SetKth(float amps);
void  HFI_SetNotch(uint8_t on);
uint8_t HFI_SpeedOk(void);     /* 1 = 转速在适用范围内(命令里用它先拒绝, 免得刚开就 flt=3) */
const PLL_control *HFI_GetPLL(void);

/* ---- 静态扫描(免锁相): 把 θ̂ 钉在"编码器角 + 固定偏置"上逐个扫, 直接量 err(Δ) ----
 *   为什么需要它: PLL 一旦跑飞, 所有数字都是它自己转出来的, 分不清到底是
 *   "这台电机没凸极"还是"解调不对称/采样相位错"。扫描时**环路完全不闭**, 不可能跑飞,
 *   出来的 err(Δ) 曲线一句话定性:
 *      干净 ∝ sin(2Δ)  ⇒ 真凸极, 问题在环路/标定;
 *      像常数或 ∝ cos(2Δ) ⇒ 解调不对称/相位错(死区、采样点、注入轴与解调轴不齐),
 *                            凸极信号被它盖住了。
 *   它同时给出**免锁相**的 snr = Kth/残差, 比锁相后那个 snr 可信得多。 */
#define HFI_SCAN_STEPS      12u
#define HFI_SCAN_STEP_DEG   15.0f     /* 0,15,…,165° 正好覆盖 sin2Δ 的一个周期(π) */
#define HFI_SCAN_DWELL      400u      /* 每档解调次数(×0.5ms = 200ms) */
#define HFI_SCAN_SETTLE     100u      /* 每档丢前几次(换偏置后电流/带通要稳) */
/* ★扫描期间每档的"轴在动"判据: 用**净转角**(rad), 不用瞬时角速度。
 *   ⚠ 踩过的坑: 先写成 |ω_enc|>5rad/s, 结果一档都过不了 —— 码盘角是 LUT 查表来的,
 *     1 个点 ≈1.26° 电角度, 差分除以 Ts=100µs 就是 ±10rad/s 的量化噪声,
 *     静止时 |ω_enc| 本来就常在 ±10rad/s 跳(现场日志 nsc=7 但 step 一直 0 ⇒ flt=6)。
 *   净转角对量化噪声免疫: 200ms 内真的转了多少, 才影响这一档的可信度。
 *   34° 的容差是折中: 2Δ 分量被这样平均掉的比例 ≈ sinc(34°)≈0.85, 曲线形状仍在;
 *   而注入本身带着转子缓慢爬行(注在 ±45° 附近有对准力矩, 会推着转子往回走)也过得去。
 *   要更严(曲线更干净)就把这个值调小, 代价是更难"按得住"。 */
#define HFI_SCAN_DTH_MAX    0.60f
#define HFI_SCAN_RETRY      4u        /* 同一档最多重来几次; 还不行就判"轴按不住" */
extern volatile uint8_t hfi_scan_en;
void    HFI_ScanStart(void);
void    HFI_ScanReport(void);         /* 主循环调用: 扫完把 12 点表 + 拟合/定性打出来 */
uint8_t HFI_ScanBusy(void);

/* ---- 解调不对称的 cos2Δ 分量补偿(★现场问题的解法) ----
 *   硬件不对称(死区/PWM/传感器)在解调量上是 `b_dc + b2·cos(2Δθ)`:
 *     · b_dc 在标定窗(±45°, cos=0)能测到, 已经在环路里减掉了;
 *     · b2 在 ±45° 处**测不到**(那里 cos2Δ 恒为 0), 但锁定时 Δθ≈0 它会整个冒出来,
 *       被 II 型环当"频差"吃掉 ⇒ 丢转速跟踪 ⇒ 跑飞。
 *   好消息(仿真 [7] 节实测): b2 完全落在 cos2Δ 分量上, **不污染 sin2Δ 幅值 K**
 *   (干净: K=−0.0308/C=0.0002; 加不对称: K=−0.0308/C=+0.2502)。
 *   ⇒ 用 hfiscan 量出 C 填进 `hfi_b2_comp`, 环路里显式减掉 `b2·cos(2(θenc−θ̂))`,
 *     环路就只剩 K·sin(2Δ) 了 —— 这是"把已知的硬件缺陷标定掉", 和死区补偿同一性质,
 *     **不影响 HFI 角的独立性**(角度仍然由 HFI 自己解调)。 */
extern float hfi_b2_comp;
void  HFI_SetB2(float a);
/* 解调轴整体旋转(rad): hfiscan 量出 φ=atan2(C,K) 后自动设成 ρ=−φ。
 *   推导: 解调轴转 ρ 后量到的曲线是 |E|·sin(2Δ−ρ), 而拟合给出 |E|·sin(2Δ+φ)
 *   ⇒ **ρ = −φ**。两种情形都对:
 *     · K≈0、C 大(现场实测 kth 撞下限 0.003) ⇒ φ≈±90° ⇒ ρ=∓90° ⇒ **等价于换到另一根轴**,
 *       说明注入轴与解调轴的参考差了 90°(约定/接线/采样相位); 这时"减掉 C"会把凸极信号
 *       一起减掉, 只有**转轴**才对(转完 C→0、K→|E|)。
 *     · K 与 C 都不小 ⇒ 同样转 ρ, 转完回到 |E|·sin(2Δ) ✓。
 *   实现上用角度和公式展开(不引入额外三角函数): sin(θ̂+ρ)=sinθ̂cosρ+cosθ̂sinρ。 */
extern float hfi_demod_rot;
void  HFI_SetDemodRot(float rad);
/* 环路阻尼系数(A·s/rad): 环路误差里减掉 kd·ω, 用来抵消"与注入轴转速同向的假误差"
 *   (现场实测的负阻尼自持振荡, 见 HFI_PLL_KD 的说明)。在线可调, 0 = 关掉。 */
extern float hfi_kd;
void  HFI_SetKd(float kd);

/* 滤波器/解调原语(与参考实现同名同义, .c 里非 static, 需要时也可单独调) */
void  BPF_2nd_Init(BPF_2nd_t *f, float b0, float b1, float b2, float a1, float a2);
float BPF_2nd_Update(BPF_2nd_t *f, float x);
void  adaptive_notch_init(adaptive_notch_t *n, float Q);
float adaptive_notch_update(adaptive_notch_t *n, float x, float freq_hz);
void  PLL_Init(PLL_control *p);
void  PLL_Proc(PLL_control *p, float Ialpha_hf, float Ibeta_hf, float theta_enc);

/* 每拍一次(10kHz, 在电流环里): 翻转极性, 输出本拍要叠加到 αβ 电压上的注入矢量[V]。
 *   对应参考实现的 `controller->V_d += Voltage_inj * 1024.0f`, 但**方向**改成了
 *   PLL 估计的 d 轴(见上面"符号约定"的★): 即 (vi·cosθ̂, vi·sinθ̂)。
 *   用的是上一拍的 θ̂(电流环在 ADC1 分支跑, θ̂ 在 ADC2 分支更新)—— 差 100us,
 *   静止/低速下可忽略。
 *   ★必须在 DePark 之后、SVPWM 之前叠加, 且与采样同一拍(本工程 CCR 在采样后立刻写,
 *     所以"这一拍写的电压"正好作用在下一个采样区间上)。 */
void  HFI_INJ(float *v_alpha, float *v_beta);

/* 每拍一次(10kHz, ADC2 分支, 编码器角刚更新完): 带通 → 解调 → PLL → 保护/诊断 */
void  HFI_Tick(float ialpha, float ibeta, float vdc, float theta_enc);

/* 主循环 20Hz: 打一行 `# hfi ...`(只在使能或报过故障时打) */
void  HFI_Report(void);

/* 供打点用的快照(角度已按模 π 口径折到离编码器最近的一支; err 单位 rad) */
float HFI_GetStreamTheta(void);
float HFI_GetStreamErr(void);

#endif
