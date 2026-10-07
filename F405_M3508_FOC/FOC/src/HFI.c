#include "HFI.h"
#include "SVPWM.h"
#include "App_Config.h"
#include "main.h"
#include "tim.h"
#include <math.h>

/* ---------------- 对外状态 ---------------- */
uint8_t  hfi_enable    = 0u;
float    hfi_u_inj     = HFI_U_INJ_DEFAULT;
float    hfi_pol       = 1.0f;
float    hfi_phi       = 0.0f;
float    hfi_theta     = 0.0f;
float    hfi_l0        = 0.0f;
float    hfi_l1        = 0.0f;
float    hfi_i_peak    = 0.0f;
uint32_t hfi_fault     = HFI_FAULT_NONE;
uint32_t hfi_foldback  = 0u;
uint32_t hfi_fit_count = 0u;
uint32_t hfi_pub_count = 0u;

/* 探针阶段: 用 HFI_U_PROBE 打 20ms, 量出纹波峰值再反算注入幅值 */
uint8_t  hfi_probe      = 0u;
float    hfi_probe_peak = 0.0f;
float    hfi_diag_tilt_ctr = 0.0f;
float    hfi_diag_tilt_pp  = 0.0f;
float    hfi_diag_mag_ctr  = 0.0f;
float    hfi_diag_mag_pp   = 0.0f;
static uint16_t hfi_probe_cnt;
static float    hfi_vdc = 12.0f;      /* 最近一次母线电压, 用于注入幅值限幅 */

/* ---------------- 内部状态 ---------------- */
static float    hfi_i1a, hfi_i1b;      /* i_{k-1} */
static float    hfi_i2a, hfi_i2b;      /* i_{k-2} */
static uint8_t  hfi_warm;              /* 已累积拍数, <2 不解调 */
static uint32_t hfi_acq;               /* 复位后已解调拍数(收敛门限) */

/* 注入命令流水线: sig_2/n2 = 两拍前的命令(刚采完的区间里真正作用的电压),
   sig_1/n1 = 上一拍命令(已由 CC4 写入, 正在作用), sig_c/dc = 本拍命令(待写入) */
static float    hfi_sig_2, hfi_n2x, hfi_n2y;
static float    hfi_sig_1, hfi_n1x, hfi_n1y;
static float    hfi_sig_c, hfi_dcx, hfi_dcy;
static volatile uint16_t hfi_ccr[3];   /* 在两个中断间传递, 必须 volatile */
static volatile uint8_t  hfi_ccr_ready;
static float    hfi_cos_step, hfi_sin_step;

/* 带遗忘最小二乘累积: [cc cs; cs ss] 为设计矩阵, 其余 4 个为相关量 */
static float    hfi_cc, hfi_cs, hfi_ss;
static float    hfi_xc, hfi_xs, hfi_yc, hfi_ys;

static uint16_t hfi_pub_cnt;
static uint8_t  hfi_pol_flips;

/* 诊断模式状态: 平滑后的 HF 电流矢量 + 统计窗内的 min/max */
#define HFI_PI              (HFI_TWO_PI * 0.5f)
#define HFI_DIAG_WIN_TICKS  (HFI_DIAG_PRINT_MS * APP_TICKS_PER_MS)  /* 统计窗 = 打印间隔 */
static float    hfi_diag_dx, hfi_diag_dy;      /* 平滑后 (dx,dy), 单位 A —— 诊断与 θ_HFI 都用它 */
static float    hfi_diag_tmax, hfi_diag_tmin;  /* 窗内倾斜角极值 deg */
static float    hfi_diag_mmax, hfi_diag_mmin;  /* 窗内幅值极值 mA */
static uint32_t hfi_diag_win;
static uint8_t  hfi_diag_seen;                 /* min/max 是否已初始化 */
static float    hfi_diag_tref;                 /* 本窗参考角: 倾斜角按它做相对跟踪, 防 ±180 跳变 */

/* φ 的有效旋转速率: 诊断模式强制 0(注入固定), 扫描模式用 HFI_SWEEP_HZ */
#if (HFI_FIX_PHI != 0u)
#define HFI_SWEEP_HZ_EFF    0.0f
#else
#define HFI_SWEEP_HZ_EFF    HFI_SWEEP_HZ
#endif

static void HFI_ResetAccum(void)
{
    hfi_i1a = 0.0f; hfi_i1b = 0.0f;
    hfi_i2a = 0.0f; hfi_i2b = 0.0f;
    hfi_warm = 0u;
    hfi_acq = 0u;
    hfi_sig_2 = 1.0f; hfi_n2x = 1.0f; hfi_n2y = 0.0f;
    hfi_sig_1 = 1.0f; hfi_n1x = 1.0f; hfi_n1y = 0.0f;
    hfi_sig_c = 1.0f; hfi_dcx = 1.0f; hfi_dcy = 0.0f;
    hfi_cc = 0.5f; hfi_ss = 0.5f; hfi_cs = 0.0f;
    hfi_xc = 0.0f; hfi_xs = 0.0f; hfi_yc = 0.0f; hfi_ys = 0.0f;
    hfi_pub_cnt = 0u;
    hfi_diag_dx = 0.0f; hfi_diag_dy = 0.0f;
    hfi_diag_win = 0u;
    hfi_diag_seen = 0u;
}

void HFI_Init(void)
{
    float step = HFI_TWO_PI * HFI_SWEEP_HZ_EFF * HFI_DT;

    hfi_enable = 0u;
    hfi_u_inj = HFI_U_INJ_DEFAULT;
    hfi_pol = 1.0f;
    hfi_phi = 0.0f;
    hfi_theta = 0.0f;
    hfi_l0 = 0.0f;
    hfi_l1 = 0.0f;
    hfi_i_peak = 0.0f;
    hfi_fault = HFI_FAULT_NONE;
    hfi_fit_count = 0u;
    hfi_pub_count = 0u;
    hfi_foldback = 0u;
    hfi_pol_flips = 0u;
    hfi_probe = 0u;
    hfi_probe_peak = 0.0f;
    hfi_probe_cnt = 0u;
    hfi_ccr_ready = 0u;
    hfi_diag_tilt_ctr = 0.0f;
    hfi_diag_tilt_pp  = 0.0f;
    hfi_diag_mag_ctr  = 0.0f;
    hfi_diag_mag_pp   = 0.0f;
    hfi_cos_step = cosf(step);
    hfi_sin_step = sinf(step);
    HFI_ResetAccum();
}

void HFI_SetEnable(uint8_t on)
{
    hfi_enable = on;
    hfi_fault = HFI_FAULT_NONE;
    hfi_i_peak = 0.0f;
    hfi_ccr_ready = 0u;
    hfi_probe_cnt = 0u;
    hfi_probe_peak = 0.0f;
    if (on != 0u)
    {
#if (HFI_FIX_PHI != 0u)
        hfi_probe = 0u;              /* 诊断模式: 幅值固定, 不做自适应标定 */
        hfi_u_inj = HFI_DIAG_U;
#else
        hfi_probe = HFI_AUTOSCALE;
        if (HFI_AUTOSCALE != 0u) hfi_u_inj = HFI_U_PROBE;
#endif
    }
    else
    {
        hfi_probe = 0u;
    }
    HFI_ResetAccum();
}

/* 注入幅值上限: 不超过 SVPWM 线性区半径(vdc/sqrt(3))的一半。
   母线只有 12V 时若硬塞 6V, 占空比会顶到限幅, 注入波形被削 -> 辨识直接失真 */
static float HFI_UInjLimit(void)
{
    float lim = HFI_U_INJ_SPAN * hfi_vdc * 0.57735026919f;
    if (lim > HFI_U_INJ_MAX) lim = HFI_U_INJ_MAX;
    if (lim < HFI_U_INJ_MIN) lim = HFI_U_INJ_MIN;
    return lim;
}

void HFI_SetUInj(float volts)
{
    float lim = HFI_UInjLimit();
    if (volts < HFI_U_INJ_MIN) volts = HFI_U_INJ_MIN;
    if (volts > lim) volts = lim;
    hfi_u_inj = volts;
    HFI_ResetAccum();   /* 解调增益变了, 旧累积值作废 */
}

/* 由带遗忘最小二乘解出 L(th) 并发布 L0/L1/th */
static void HFI_Solve(void)
{
    float g = 2.0f * hfi_u_inj * HFI_DT * HFI_GAIN_TRIM;
    float det, m11, m12, m12b, m22, det_m, l11, l12, l22, l0, l1, th;

    hfi_pub_count++;
    if (hfi_acq < HFI_MIN_FIT_TICKS) return;   /* 等累积收敛, 避免发布垃圾 */

    det = hfi_cc * hfi_ss - hfi_cs * hfi_cs;
    if (det < 1.0e-6f) return;

    /* 设计矩阵求逆解 M(m = g*M*n) */
    m11 = (hfi_ss * hfi_xc - hfi_cs * hfi_xs) / det;
    m22 = (hfi_cc * hfi_ys - hfi_cs * hfi_yc) / det;
    m12 = (hfi_cc * hfi_xs - hfi_cs * hfi_xc) / det;
    m12b = (hfi_ss * hfi_yc - hfi_cs * hfi_ys) / det;
    m12 = 0.5f * (m12 + m12b);   /* m_xy 的两个估计取平均 */

    det_m = m11 * m22 - m12 * m12;
    if (fabsf(det_m) < 1.0e-9f) return;

    l11 =  g * m22 / det_m;
    l12 = -g * m12 / det_m;
    l22 =  g * m11 / det_m;

    l0 = 0.5f * (l11 + l22);
    l1 = sqrtf(0.25f * (l11 - l22) * (l11 - l22) + l12 * l12);
    th = 0.5f * atan2f(l12, 0.5f * (l11 - l22));
    if (th < 0.0f) th += APP_PI;   /* [-pi/2, pi/2] -> [0, pi) */

    if ((l0 > 1.0e-5f) && (l0 < 0.1f))
    {
        hfi_l0 = l0;
        hfi_l1 = l1;
        hfi_theta = th;
        hfi_fit_count++;
    }
    else if ((l0 < -1.0e-5f) && (l0 > -0.1f) &&
             (hfi_fit_count == 0u) && (hfi_pol_flips < HFI_MAX_POL_FLIPS))
    {
        /* 解调整体符号与实际不符: 翻转后重新累积 */
        hfi_pol = -hfi_pol;
        hfi_pol_flips++;
        HFI_ResetAccum();
    }
}

/* 每个 PWM 周期调用一次: 解调 + 生成注入电压并暂存 CCR(等 CC4 写入) */
void HFI_Tick(float ialpha, float ibeta, float vdc)
{
    uint16_t c1, c2, c3;
    float    im, u, nx, ny;

    hfi_vdc = vdc;

    /* 1. 电流保护: 硬阈值封锁驱动; 软阈值折半注入幅值(自动适配未知电感) */
    im = sqrtf(ialpha * ialpha + ibeta * ibeta);
    if (im > hfi_i_peak) hfi_i_peak = im;
    if (im > HFI_CURRENT_HARD)
    {
        hfi_fault = HFI_FAULT_OVERCUR;
        hfi_enable = 0u;
        hfi_probe = 0u;
        hfi_ccr_ready = 0u;
        /* 硬封锁: 三相都写中点(零矢量), 且撤掉 CC4 的延迟写, 立即生效 */
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, (uint16_t)(PWM_PERIOD / 2u));
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, (uint16_t)(PWM_PERIOD / 2u));
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, (uint16_t)(PWM_PERIOD / 2u));
        return;
    }
    if (hfi_probe == 0u && im > HFI_CURRENT_SOFT)
    {
        hfi_foldback++;
        HFI_SetUInj(hfi_u_inj * 0.5f);   /* 内部限幅 + 复位累积 */
    }

    /* 1b. 探针: 先跳过失稳瞬态, 再按实测纹波峰值反算注入幅值 */
    if (hfi_probe != 0u)
    {
        hfi_probe_cnt++;
        if (hfi_probe_cnt > HFI_PROBE_SKIP_TICKS)
        {
            if (im > hfi_probe_peak) hfi_probe_peak = im;
            if (hfi_probe_cnt >= HFI_PROBE_SKIP_TICKS + HFI_PROBE_TICKS)
            {
                float ut = HFI_U_PROBE *
                           (HFI_TARGET_PEAK_A / (hfi_probe_peak + 0.02f));
                hfi_probe = 0u;
                HFI_SetUInj(ut);         /* 内部限幅 + 复位累积 */
            }
        }
    }

    /* 2. 二阶差分解调: 用两拍前的命令(那才是刚采完的区间里作用的电压) */
    if (hfi_warm >= 2u)
    {
        float dx = (ialpha - 2.0f * hfi_i1a + hfi_i2a) * hfi_sig_2 * hfi_pol;
        float dy = (ibeta  - 2.0f * hfi_i1b + hfi_i2b) * hfi_sig_2 * hfi_pol;
        float cx = hfi_n2x;
        float cy = hfi_n2y;

        if (hfi_acq < 4000000000u) hfi_acq++;

        hfi_cc += HFI_FORGET * (cx * cx - hfi_cc);
        hfi_cs += HFI_FORGET * (cx * cy - hfi_cs);
        hfi_ss += HFI_FORGET * (cy * cy - hfi_ss);
        hfi_xc += HFI_FORGET * (dx * cx - hfi_xc);
        hfi_xs += HFI_FORGET * (dx * cy - hfi_xs);
        hfi_yc += HFI_FORGET * (dy * cx - hfi_yc);
        hfi_ys += HFI_FORGET * (dy * cy - hfi_ys);

#if (HFI_FIX_PHI != 0u)
        /* 诊断: 平滑 (dx,dy) -> 幅值(mA) 与倾斜角(deg), 并统计窗内峰峰值。
         *   平滑 32 拍(0.8ms) 压掉单点噪声, 又基本不衰减手转产生的摆动。 */
        {
            float a = 1.0f / (float)HFI_DIAG_SMOOTH;
            float mag, tilt, dtl, ctr;

            hfi_diag_dx += a * (dx - hfi_diag_dx);
            hfi_diag_dy += a * (dy - hfi_diag_dy);
            mag  = sqrtf(hfi_diag_dx * hfi_diag_dx + hfi_diag_dy * hfi_diag_dy) * 1000.0f;
            tilt = atan2f(hfi_diag_dy, hfi_diag_dx) * 57.2957795f;

            if (hfi_diag_seen == 0u)
            {
                hfi_diag_seen = 1u;
                hfi_diag_tref = tilt;    /* 本窗参考角 */
                hfi_diag_tmax = hfi_diag_tmin = 0.0f;
                hfi_diag_mmax = hfi_diag_mmin = mag;
            }
            /* 相对参考角跟踪(折到 ±180): 倾斜角落在 ±180 附近时不会被跳变毁掉峰峰值 */
            dtl = tilt - hfi_diag_tref;
            while (dtl >=  180.0f) dtl -= 360.0f;
            while (dtl <  -180.0f) dtl += 360.0f;

            if (dtl > hfi_diag_tmax) hfi_diag_tmax = dtl;
            if (dtl < hfi_diag_tmin) hfi_diag_tmin = dtl;
            if (mag > hfi_diag_mmax) hfi_diag_mmax = mag;
            if (mag < hfi_diag_mmin) hfi_diag_mmin = mag;

            if (++hfi_diag_win >= HFI_DIAG_WIN_TICKS)
            {
                ctr = hfi_diag_tref + 0.5f * (hfi_diag_tmax + hfi_diag_tmin);
                while (ctr >=  180.0f) ctr -= 360.0f;
                while (ctr <  -180.0f) ctr += 360.0f;

                hfi_diag_win       = 0u;
                hfi_diag_tilt_pp   = hfi_diag_tmax - hfi_diag_tmin;
                hfi_diag_tilt_ctr  = ctr;
                hfi_diag_mag_pp    = hfi_diag_mmax - hfi_diag_mmin;
                hfi_diag_mag_ctr   = 0.5f * (hfi_diag_mmax + hfi_diag_mmin);

                hfi_diag_tref = tilt;    /* 下一窗重新取参考, tmin/tmax 归零 */
                hfi_diag_tmax = hfi_diag_tmin = 0.0f;
                hfi_diag_mmax = hfi_diag_mmin = mag;
            }
        }
#endif
    }

    /* 3. 采样历史 */
    hfi_i2a = hfi_i1a; hfi_i1a = ialpha;
    hfi_i2b = hfi_i1b; hfi_i1b = ibeta;
    if (hfi_warm < 2u) hfi_warm++;

    /* 4. 命令流水线右移 */
    hfi_sig_2 = hfi_sig_1; hfi_n2x = hfi_n1x; hfi_n2y = hfi_n1y;
    hfi_sig_1 = hfi_sig_c; hfi_n1x = hfi_dcx; hfi_n1y = hfi_dcy;

    /* 5. 产生本拍命令: 极性翻转 + 方向缓慢旋转(旋转因子递推, 不调 sin/cos) */
    hfi_phi += HFI_TWO_PI * HFI_SWEEP_HZ_EFF * HFI_DT;
    if (hfi_phi >= HFI_TWO_PI) hfi_phi -= HFI_TWO_PI;
    nx = hfi_dcx * hfi_cos_step - hfi_dcy * hfi_sin_step;
    ny = hfi_dcy * hfi_cos_step + hfi_dcx * hfi_sin_step;
    hfi_dcx = nx;
    hfi_dcy = ny;
    hfi_sig_c = -hfi_sig_c;
    u = hfi_sig_c * hfi_u_inj;

    SVPWM(u * nx, u * ny, vdc, &c1, &c2, &c3);
    hfi_ccr[0] = c1;
    hfi_ccr[1] = c2;
    hfi_ccr[2] = c3;
    hfi_ccr_ready = 1u;   /* 由下一个 CC4 中断(采样时刻)写入 TIM1 */

    /* 6. 周期求解 */
    if (++hfi_pub_cnt >= HFI_PUBLISH_TICKS)
    {
        hfi_pub_cnt = 0u;
        HFI_Solve();
        hfi_i_peak = 0.0f;   /* 峰值保持按窗口清零 */
    }
}

/* TIM1 CC4 中断(注入 ADC 采样时刻)调用: 在同一时刻切换注入电压 */
void HFI_Cc4Irq(void)
{
    if (hfi_ccr_ready == 0u) return;
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, hfi_ccr[0]);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, hfi_ccr[1]);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, hfi_ccr[2]);
}

