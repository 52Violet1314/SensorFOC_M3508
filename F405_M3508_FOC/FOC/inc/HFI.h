#ifndef __HFI_H__
#define __HFI_H__

#include <stdint.h>

/* ================== HFI 高频方波注入: 静止凸极辨识(阶段1) ==================
 * 目标: 电机完全不转、不输出任何转矩, 只做高频注入, 辨识定子电感矩阵
 *         L(a,b) = [[L0 + L1*cos2th, L1*sin2th],
 *                   [L1*sin2th,     L0 - L1*cos2th]]
 *       并解算出电角度 th, 与编码器电角度对比 —— 判定 M3508(表贴式)到底有没有
 *       足够凸极做 HFI, 以及注入/采样/解调链路是否正确。
 *
 * 注入: 每拍翻转极性的方波电压矢量, 幅值 U, 方向 phi 缓慢旋转(0.5 Hz)。
 * 解调(方波二阶差分, 不需要任何滤波器):
 *     i_k - 2*i_{k-1} + i_{k-2} = 2*U*Ts*L(th)^-1*n(phi)      (见下方时序说明)
 *   基波电流/反电动势项在二阶差分中互相抵消。因此:
 *     ① 三相电流必须同源(见 App_FOC.c 的 adc_group_mask);
 *     ② 绝对不能用 App_FilterCurrent 滤波后的值。
 *
 * 注入幅值自适应(解决"电感未知"问题):
 *   上电先用 HFI_U_PROBE(0.5V) 打探针, 跳过前 5ms 瞬态后量出纹波峰值,
 *   按 HFI_TARGET_PEAK_A 反算注入电压; 运行中若电流再超 HFI_CURRENT_SOFT
 *   就折半(HFI_CURRENT_HARD 才封锁驱动)。
 *
 * 时序(关键): CC4 事件既触发注入组 ADC(采样时刻), 又在中断里把上一拍算好的
 *   CCR 立刻写入 TIM1 —— 即"注入电压切换"与"电流采样"发生在同一时刻。
 *   否则两次采样之间的实际作用电压是"新旧两段"的混合: 设新命令占 (1-a),
 *   解调增益就变成 (1-2a); 由于 ADC 转换 + ISR 在 -O0 下约 30~60 us, a 恰好
 *   落在 0.3~0.6, 增益可能只剩 0.1 倍甚至反号 —— 这是本设计必须避免的坑。
 *   电压在下一个 CC4 事件(采样时刻)才生效, 所以本拍解调要用"两拍前"的
 *   极性/方向(代码里的 sig_2 / n2x / n2y)。
 *
 * 对 phi 转一圈的数据做带遗忘的最小二乘拟合 m = g*L^-1*n (g = 2*U*Ts),
 * 再对 2x2 矩阵求逆即得 L0 / L1 / th。g 的符号若与硬件不符(理论上不该),
 * 由 pol 自动翻转纠正。
 *
 * 安全: 注入电压平均值为 0, 不产生平均转矩、不产生平均电流, 转子静止不动。
 *       但若外部把转子转起来, 反电动势会灌入近似短路的绕组 -> 过流,
 *       故有 HFI_CURRENT_HARD 保护(触发后封锁驱动, 需 CAN 0x10 重新使能)。
 *       做本实验时请勿用手快速拨动转子。
 */
#define HFI_TWO_PI          6.28318530718f

#define HFI_DT              0.000025f /* PWM/电流环周期 s (40 kHz -> 注入 20 kHz) */
#define HFI_U_INJ_DEFAULT   1.0f      /* 非自适应时的注入电压幅值 V */
#define HFI_U_INJ_MIN       0.2f
#define HFI_U_INJ_MAX       6.0f
#define HFI_SWEEP_HZ        0.5f      /* 注入方向 phi 旋转频率 Hz(一圈 2s) */

#define HFI_AUTOSCALE       1u        /* 1 = 上电用探针自动标定注入幅值 */
#define HFI_U_PROBE         0.5f      /* 探针注入幅值 V */
#define HFI_PROBE_SKIP_TICKS 2000u    /* 探针前 5ms 不计峰值: 避开使能瞬间的电流冲击 */
#define HFI_PROBE_TICKS     800u      /* 探针有效时长: 800 拍 = 20 ms */
#define HFI_TARGET_PEAK_A   1.5f      /* 期望的电流纹波峰值 A */
#define HFI_U_INJ_SPAN      0.5f      /* 注入幅值上限 = 该系数 * SVPWM 线性区半径 */

#define HFI_CURRENT_SOFT    2.5f      /* 软阈值 A: 超过则注入幅值折半, 不封锁 */
#define HFI_CURRENT_HARD    8.0f      /* 硬阈值 A: 超过则立即封锁驱动 */

#define HFI_FORGET          0.00005f  /* 最小二乘遗忘系数(等效窗约 20000 拍=0.5s) */
/* 电感绝对增益校正: 电流通道并不是在 CC4 触发瞬间采样, 而是注入组的第 3 个转换,
 * 即触发后 n*(S+12)/ADCCLK = 2*40/21MHz ≈ 3.8us 才采到(40kHz, S=28 周期)。
 *   解调增益 = 1 - 2a,  a = (0.6us + 3.8us)/25us = 0.176  ->  0.648
 * (10kHz/S=144 周期时 a≈0.155 -> 0.69; 换参数要跟着重算)
 * ★用 LCR 实测(0.097mH)复核: 现在 HFI 报 0.22~0.26mH, 正好差 ~1.5 倍, 与此吻合 */
#define HFI_GAIN_TRIM       0.648f    /* 采样时刻校正; 若与 LCR 仍差可在此微调 */
#define HFI_MIN_FIT_TICKS   40000u    /* 复位后至少累积 1s 才发布结果 */
#define HFI_PUBLISH_TICKS   2000u     /* 50 ms 求解/发布一次(与打印同频) */
#define HFI_MAX_POL_FLIPS   4u        /* 极性自动纠正最多尝试次数 */

#define HFI_AUTO_START      0u        /* 0 = 上电不注入(默认; 电机模式可直接跑)
                                      * 要用 HFI 时发 CAN 0x10 = 非0 启动, 0x10 = 0 退出 */

/* ================= 诊断模式: 固定注入方向, 判"真凸极"还是"伪各向异性" =============
 * 做法: 注入方向 φ 固定不动(不说电机也不动), 手拨转子慢慢转, 看"HF 电流矢量相对注入
 *       方向的倾斜角"是否随转子以 2θ 周期摆动。
 *   倾斜角摆幅 = 2*atan(L1/L0), 与 L 的绝对值、与 GAIN_TRIM 全都无关
 *     (公共标度因子在 atan2 里约掉) —— 所以这个数可以直接当 L1/L0 读。
 *   判定:
 *     摆幅稳定且以 14 次/电机圈(=2*极对数)重复  -> 转子相关凸极是真的
 *     纹丝不动(dx,dy 基本恒定)                  -> 之前的"33~48%凸极"是定子系伪各向异性
 *                                                  (通道失配/采样错位), 凸极法不成立
 *   同时: mag(mA) 列可以直接反推电感  L0 ≈ 2*U*Ts*(1-2a)/mag ≈ 9.0e-5/mag[A]
 *   1 = 诊断模式(φ 固定 0 = α 轴, 用固定幅值 HFI_DIAG_U, 跳过自适应标定)
 *   0 = 原扫描模式(φ 以 HFI_SWEEP_HZ 旋转, 最小二乘辨识 L0/L1/θ)                  */
#define HFI_FIX_PHI         1u
#define HFI_DIAG_U          2.0f     /* 诊断注入幅值 V(固定值, 便于顺便反推 L) */
#define HFI_DIAG_PRINT_MS   200u     /* 诊断打印/峰峰统计窗 ms(200ms 才能容下慢手转) */
#define HFI_DIAG_SMOOTH     32u      /* (dx,dy) 轻平滑拍数(0.8ms): 压噪声又不吃掉摆动 */

#if (HFI_FIX_PHI != 0u)
#define HFI_PRINT_MS        HFI_DIAG_PRINT_MS
#else
#define HFI_PRINT_MS        50u
#endif

#define HFI_FAULT_NONE      0u
#define HFI_FAULT_OVERCUR   1u        /* 硬过流, 已封锁驱动 */

extern uint8_t  hfi_enable;      /* 1 = HFI 实验模式(电流/速度/位置环全部旁路) */
extern uint8_t  hfi_probe;       /* 1 = 正在打探针(还没定注入幅值) */
extern float    hfi_u_inj;       /* 当前注入电压幅值 V */
extern float    hfi_pol;         /* 解调极性(+1/-1), 自动纠正 */
extern float    hfi_phi;         /* 当前注入方向 rad */
extern float    hfi_theta;       /* 辨识出的电角度 rad, [0, pi) */
extern float    hfi_l0;          /* 各向同性电感 H */
extern float    hfi_l1;          /* 凸极电感幅值 H */
extern float    hfi_i_peak;      /* 上次发布窗口内 |i_alphabeta| 峰值 A */
extern float    hfi_probe_peak;  /* 探针测到的纹波峰值 A */
extern uint32_t hfi_fault;
extern uint32_t hfi_foldback;    /* 软折返次数 */
extern uint32_t hfi_fit_count;   /* 有效辨识次数 */
extern uint32_t hfi_pub_count;   /* 求解次数(含无效) */
/* 诊断模式输出(见 HFI_FIX_PHI 说明) */
extern float    hfi_diag_tilt_ctr;  /* 上个统计窗: 倾斜角中心 deg */
extern float    hfi_diag_tilt_pp;   /* 上个统计窗: 倾斜角峰峰值 deg = 2*atan(L1/L0) */
extern float    hfi_diag_mag_ctr;   /* 上个统计窗: HF 电流矢量幅值中心 mA */
extern float    hfi_diag_mag_pp;    /* 上个统计窗: 幅值峰峰值 mA(= 2*(L1/L0)*中心) */

void HFI_Init(void);
void HFI_SetEnable(uint8_t on);
void HFI_SetUInj(float volts);
void HFI_Tick(float ialpha, float ibeta, float vdc);
void HFI_Cc4Irq(void);           /* 在 TIM1_CC(TIM1_CC4) 中断里调用 */

#endif
