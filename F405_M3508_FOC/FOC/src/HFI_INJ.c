#include "HFI_INJ.h"
#include "App_Config.h"      /* APP_* 常量(只为口径一致, 本文件不依赖它的值) */
#include "Encoder.h"         /* Encoder_IsValid(): 标定窗要拿编码器当基准 */
#include "main.h"            /* HAL_GetTick + CMSIS(CoreDebug/DWT 计时) */
#include "usbd_cdc_if.h"     /* CDC_Printf */
#include <math.h>

/* ============================================================================
 * HFI_INJ.c —— 凸极法高频方波注入(Vd 脉振)诊断观测器
 *
 *   相对参考实现(HFI_INJ.c)的**全部**改动, 一眼能对完:
 *     D1. `controller->V_d += Voltage_inj*1024.0f` (Q10 + ControllerStruct) 本工程没有
 *         ⇒ 改成 `float HFI_INJ()` 返回要叠加到 Vd 的电压[V], 由 App_CurrentLoopTick 相加。
 *     D2. 参考代码里 BPF 只 init 没被调用(解调需要的是"带通后的 αβ 高频电流")。
 *         这里把前端补齐: αβ 各两级级联带通(与 mf_HFI_Observer.m 一致), 再进 PLL_Proc。
 *     D3. 环路(陷波/PI/积分)只在新解调值时跑一步(2kHz), 不再每拍都跑 ——
 *         原代码每拍进 PI 但 err 每 5 拍才更新, 等效增益 ×5(注释写的 2kHz 才是本意)。
 *     D4. 删掉 Idq_LPF / Idq_to_Idqf(定点低通): 全文件没有任何调用点, 本工程也有 LPF.h。
 *     D5. PLL_LPF 原来用函数内 static 保存状态(多实例会串), 改成用结构体字段。
 *     D6. 新增: 观测模式的诊断量(实测 Kth/SNR/凸极率估计/真实角误差) + 过流保护 + DWT 计时。
 *     D7. Kth 自标定必须"有激励才拟合、估出来就冻结": 环路锁定后 Δθ→0, 继续拟合
 *         会把 Kth 越拟越小直到撞下限(增益反而放大 25 倍) —— 离线自检抓出来的。
 *     D8. 使能时把 PLL 初值故意放到离编码器 45° 处(sin2Δθ=1, 灵敏度最大),
 *         既保证 D7 的激励, 又只是改观测器初值、不碰驱动角。
 *
 *   数据流(每拍 10kHz):
 *     HFI_INJ()            翻转极性 → 返回叠加到 Vd 的电压(在电流环里, DePark 之前)
 *     HFI_Tick(iα,iβ,vdc,θenc)  两级带通 → PLL_Proc(解调+PI 2kHz) → 保护/计时
 * ========================================================================== */

/* ---------------- 对外状态 ---------------- */
volatile uint8_t hfi_enable   = 0u;
volatile uint8_t hfi_fault    = 0u;
volatile uint8_t hfi_notch_en = 0u;   /* ★默认关: 静止时陷波落在环路带宽内会振, 见头文件 */
float    hfi_v_inj            = HFI_INJ_V_AMP;
float    hfi_v_lim            = HFI_INJ_V_AMP;
float    hfi_kth              = HFI_KTH_DEF;
float    hfi_kth_meas         = 0.0f;
float    hfi_kth_rms          = 0.0f;
float    hfi_snr              = 0.0f;
float    hfi_sal_pct          = 0.0f;
float    hfi_kth_bias         = 0.0f;
float    hfi_b2_comp          = 0.0f;   /* 解调不对称 cos2Δ 分量补偿(hfiscan 自动填/可手改) */
float    hfi_kd               = HFI_PLL_KD;  /* ★环路阻尼系数(A·s/rad): 误差里减掉 kd·ω */
float    hfi_demod_rot        = 0.0f;   /* 解调轴相对 θ̂ 的整体旋转(rad, 见 PLL_Proc 第2步) */
static float hfi_rot_sin       = 0.0f;  /* = sin(hfi_demod_rot), SetDemodRot 里更新 */
static float hfi_rot_cos       = 1.0f;  /* = cos(hfi_demod_rot) */
static uint8_t kth_from_scan   = 0u;    /* 1 = 环路增益来自 hfiscan(免锁相), 标定窗别覆盖 */
static uint8_t scan_auto_req   = 0u;    /* 1 = HFI_Tick 里启动自动扫描(不在 PLL_Proc 里调) */
static uint8_t scan_auto_done  = 0u;    /* 本次使能里已经自动扫过一轮(避免死循环) */
float    hfi_w_enc            = 0.0f;
float    hfi_us_peak          = 0.0f;
uint32_t hfi_clamp_cnt        = 0u;
uint32_t hfi_ticks            = 0u;

/* ---------------- 内部状态 ---------------- */
static PLL_control      PLL;
static adaptive_notch_t Notch_2we;
static BPF_2nd_t        BPF_A1, BPF_A2;   /* α 轴两级级联 */
static BPF_2nd_t        BPF_B1, BPF_B2;   /* β 轴两级级联 */

/* 注入状态(对应参考实现的 count / Voltage_inj / sign / inj_tick) */
static uint32_t count = 0u;
static uint32_t inj_tick = 0u;
static int8_t   sign = 1;
static float    Voltage_inj = 0.0f;

/* 打点/报告用的快照 */
static float hfi_theta_stream = 0.0f;   /* PLL 角, [0,2π) */
static float hfi_err_stream   = 0.0f;   /* θ̂ − θenc, 折到 ±π/2 rad */

/* ---------------- 静态扫描状态 ---------------- */
typedef struct
{
    float    off_deg;      /* θ̂ = θenc + off_deg */
    float    err;          /* 该档解调误差均值 A */
    float    id;           /* 该档载波电平均值 A(判"注入是否真加上了") */
    float    wmax;         /* 该档期间最大 |ω_enc|(质量指标, 越小越可信) */
    uint16_t n;            /* 有效样本数 */
} hfi_scan_pt_t;

volatile uint8_t hfi_scan_en   = 0u;
static hfi_scan_pt_t scan_pt[HFI_SCAN_STEPS];
static uint8_t  scan_step;
static uint16_t scan_dwell;
static uint16_t scan_settle;
static uint16_t scan_n;
static float    scan_err_sum;
static float    scan_id_sum;
static float    scan_w_max;      /* 本档期间最大 |ω_enc|(仅记录, 判据用净转角) */
static float    scan_th0;        /* 本档净转角的起点(θenc) */
static uint8_t  scan_pt_ready;   /* >0: 刚完成第 (该值) 档, 主循环打一行进度 */
static uint8_t  scan_retry;      /* 本档因"轴在动"重来的次数 */
static uint8_t  scan_done_flag;
static uint8_t  flt2_hinted;
static uint8_t  rail_note;       /* 跑飞事件: 交给主循环打一次解释 */
static uint8_t  flt4_hinted;     /* flt=4/flt=5 的"下一步该干嘛"只提示一次(主循环里打) */
static uint8_t  flt5_hinted;
static uint8_t  flt6_hinted;
/* 状态可见性: "标定窗启动过几次 / 扫描启动过几次" —— 用来一眼看出是不是在死循环重来
 *   (现场出现过 cal=1 持续 2s + err 以 15° 步进, 只看报告分不清是在扫描还是在重标定) */
static uint16_t cal_start_cnt;
static uint16_t scan_start_cnt;

/* 标定窗状态机(使能后先跑它, 见 PLL_Proc 的 3b) */
static uint16_t cal_cnt;                /* 剩余解调次数 */
static uint8_t  cal_phase;              /* 0 = +45° 点, 1 = −45° 点 */
static uint16_t cal_half;               /* 本点还剩几次解调 */
static uint8_t  cal_skip;               /* 换点后丢几次 */
static float    cal_p_sum, cal_m_sum;
static uint16_t cal_p_n, cal_m_n;
/* 失锁检测 */
static uint16_t nolock_cnt;
static uint16_t startup_cnt;     /* >0: 交接后的软起动窗(纯比例)剩余解调次数 */
static uint8_t  rail_retry;      /* 跑飞后"重开标定窗重试"已用次数 */
/* 偏置来源: 0 = 标定窗两点平均, 1 = 扫描 12 点拟合的常数项。 *   标定窗的两点是用**编码器角**摆出来的(±45°), 而这台电机编码器 LUT 本身
 *   有 +17° DC / 16.5° p-p 的角误差 ⇒ ±45° 在转子坐标里并不是 ±45°,
 *   cos2Δ 项会漏进来 ⇒ 两点平均出来的 b 被污染(现场 kth 反倒偏大 0.21A 就是这个)。
 *   扫描的 b 是 180° 全程 12 点的均值, 对角度误差一阶免疫 ⇒ 有扫描值就用它。 */
static uint8_t  bias_from_scan;
/* 编码器电角速度(交接前馈 + 对照) */
static float    hfi_th_prev;
static uint8_t  hfi_th_first;
static void     hfi_cal_start(void);

/* ---------------- 小工具 ---------------- */
static float Range_to_2PI(float Thetae)
{
    float temp = Thetae;

    while (temp >= HFI_TWO_PI) temp -= HFI_TWO_PI;
    while (temp < 0.0f)        temp += HFI_TWO_PI;
    return temp;
}

static float Wrap_pm(float x)
{
    while (x >  HFI_PI) x -= HFI_TWO_PI;
    while (x < -HFI_PI) x += HFI_TWO_PI;
    return x;
}

/* 折到 ±π/2: 凸极法只有模 π 的轴, sin(2Δθ) 在 Δ 与 Δ+π 上取值相同 */
static float Fold_half_pi(float x)
{
    while (x >  HFI_HALF_PI) x -= HFI_PI;
    while (x < -HFI_HALF_PI) x += HFI_PI;
    return x;
}

/* 本拍实际可用的注入幅值: 硬上限 + SVPWM 线性区(0.5·vdc/√3) */
static float HFI_VLim(float vdc)
{
    float lim = HFI_INJ_V_MAX;
    float span;

    if (vdc > 1.0f)                      /* vdc 无效(<1V)时只用硬上限 */
    {
        span = HFI_INJ_V_SPAN * vdc * 0.57735026919f;
        if (span < lim) lim = span;
    }
    if (hfi_v_inj < HFI_INJ_V_MIN) return HFI_INJ_V_MIN;
    if (hfi_v_inj > lim)
    {
        return lim;
    }
    return hfi_v_inj;
}

/* DWT 计时(实测中断里这段代码花多少 us) —— 出问题第一个要看的就是它 */
static void HFI_DwtInit(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0u;
    DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;
}

static float HFI_Us(uint32_t cyc)
{
    float mhz = (float)SystemCoreClock / 1000000.0f;

    if (mhz < 1.0f) mhz = 168.0f;
    return (float)cyc / mhz;
}

/* ============================================================================
 * 2 阶 IIR(RBJ biquad, Direct-Form I)  —— 与参考实现逐行一致
 * ========================================================================== */
void BPF_2nd_Init(BPF_2nd_t *f, float b0, float b1, float b2, float a1, float a2)
{
    f->b0 = b0; f->b1 = b1; f->b2 = b2; f->a1 = a1; f->a2 = a2;
    f->x1 = f->x2 = f->y1 = f->y2 = 0.0f;
}

float BPF_2nd_Update(BPF_2nd_t *f, float x)
{
    float y = f->b0 * x + f->b1 * f->x1 + f->b2 * f->x2
              - f->a1 * f->y1 - f->a2 * f->y2;

    f->x2 = f->x1; f->x1 = x;
    f->y2 = f->y1; f->y1 = y;
    return y;
}

/* ============================================================================
 * 自适应陷波(二阶 RBJ Notch biquad, 中心随 2·ωe 走) —— 与参考实现一致
 *   H(s) = (s²+ω₀²)/(s²+(ω₀/Q)s+ω₀²), 更新率 HFI_DEMOD_HZ(2kHz)
 * ========================================================================== */
void adaptive_notch_init(adaptive_notch_t *n, float Q)
{
    float w0;

    n->Q = Q;
    w0 = HFI_TWO_PI * HFI_NOTCH_F_MIN * HFI_TS_LOOP;   /* 从最低频起步, 免得系数从 0 起跳 */
    n->w_cos  = cosf(w0);
    n->alpha  = sinf(w0) / (2.0f * Q);
    n->a0_inv = 1.0f / (1.0f + n->alpha);
    n->x1 = n->x2 = 0.0f;
    n->y1 = n->y2 = 0.0f;
}

float adaptive_notch_update(adaptive_notch_t *n, float x, float freq_hz)
{
    float w0, cos_w0, alpha_new;
    float b0, b1, b2, a1, a2, y;

    if (freq_hz < HFI_NOTCH_F_MIN)      freq_hz = HFI_NOTCH_F_MIN;
    else if (freq_hz > HFI_NOTCH_F_MAX) freq_hz = HFI_NOTCH_F_MAX;

    w0        = HFI_TWO_PI * freq_hz * HFI_TS_LOOP;
    cos_w0    = cosf(w0);
    alpha_new = sinf(w0) / (2.0f * n->Q);

    n->w_cos  = HFI_NOTCH_ADAPT * cos_w0    + (1.0f - HFI_NOTCH_ADAPT) * n->w_cos;
    n->alpha  = HFI_NOTCH_ADAPT * alpha_new + (1.0f - HFI_NOTCH_ADAPT) * n->alpha;
    n->a0_inv = 1.0f / (1.0f + n->alpha);

    b0 = n->a0_inv;
    b1 = -2.0f * n->w_cos * n->a0_inv;
    b2 = n->a0_inv;
    a1 = b1;                                        /* a1/a0 == b1/a0 */
    a2 = (1.0f - n->alpha) * n->a0_inv;

    y = b0 * x + b1 * n->x1 + b2 * n->x2 - a1 * n->y1 - a2 * n->y2;

    n->x2 = n->x1; n->x1 = x;
    n->y2 = n->y1; n->y1 = y;
    return y;
}

/* ============================================================================
 * PLL 初始化 —— 字段与参考实现一一对应
 * ========================================================================== */
void PLL_Init(PLL_control *p)
{
    p->Kp = HFI_PLL_KP;
    p->Ki = HFI_PLL_KI;
    p->Omega_e = 0.0f;
    p->Omega_e_F = 0.0f;
    p->I_Partern = 0.0f;
    p->P_Partern = 0.0f;
    p->Thetae = 0.0f;
    p->sin_PLL = 0.0f;
    p->cos_PLL = 1.0f;
    p->x = 0.0f;
    p->Iq_pll_hf = 0.0f;
    p->Iq_pll_hf_prev = 0.0f;
    p->err_raw = 0.0f;
    p->err_lpf = 0.0f;
    p->err_notched = 0.0f;
    p->notch_fc_hz = 0.0f;
    p->Iq_acc = 0.0f;
    p->Id_acc = 0.0f;
    p->acc_cnt = 0u;
    p->sign_prev = 0;
    p->speed = 0;
    p->Id_demod = 0.0f;
    p->Iq_demod = 0.0f;
    p->theta_err = 0.0f;
    p->kth_num = 0.0f;
    p->kth_den = 0.0f;
    p->kth_res = 0.0f;
    p->kth_done = 0u;
    p->cal_run = 0u;
}

/* ============================================================================
 * 初始化 / 设置
 * ========================================================================== */
void HFI_Init(void)
{
    BPF_2nd_Init(&BPF_A1, HFI_BPF_B0, HFI_BPF_B1, HFI_BPF_B2, HFI_BPF_A1, HFI_BPF_A2);
    BPF_2nd_Init(&BPF_A2, HFI_BPF_B0, HFI_BPF_B1, HFI_BPF_B2, HFI_BPF_A1, HFI_BPF_A2);
    BPF_2nd_Init(&BPF_B1, HFI_BPF_B0, HFI_BPF_B1, HFI_BPF_B2, HFI_BPF_A1, HFI_BPF_A2);
    BPF_2nd_Init(&BPF_B2, HFI_BPF_B0, HFI_BPF_B1, HFI_BPF_B2, HFI_BPF_A1, HFI_BPF_A2);
    PLL_Init(&PLL);
    adaptive_notch_init(&Notch_2we, HFI_NOTCH_Q);
    HFI_DwtInit();

    /* 默认关: 注入是"真给电机加电压", 必须显式 hfi 1 才开 */
    hfi_enable = 0u;
    hfi_fault  = 0u;
    hfi_notch_en = 0u;      /* ★默认关(离线自检结论), 要试验用 hfin 1 */
    hfi_v_inj  = HFI_INJ_V_AMP;
    hfi_v_lim  = HFI_INJ_V_AMP;
    hfi_kth    = HFI_KTH_DEF;
    hfi_kth_meas = 0.0f;
    hfi_kth_rms  = 0.0f;
    hfi_snr      = 0.0f;
    hfi_sal_pct  = 0.0f;
    hfi_us_peak  = 0.0f;
    hfi_clamp_cnt = 0u;
    hfi_ticks     = 0u;
    count = 0u;      /* 让第一次翻转就真的换极性(参考实现从 1 起会白等一个半周期) */
    inj_tick = 0u;
    sign = 1;
    Voltage_inj = 0.0f;
    cal_cnt = 0u;
    cal_phase = 0u;
    cal_half = HFI_CAL_HALF;
    cal_skip = HFI_CAL_SKIP;
    cal_p_sum = cal_m_sum = 0.0f;
    cal_p_n = cal_m_n = 0u;
    nolock_cnt = 0u;
    hfi_kth_bias = 0.0f;
    hfi_w_enc = 0.0f;
    hfi_th_prev = 0.0f;
    hfi_th_first = 0u;
}

void HFI_SetEnable(uint8_t on)
{
    /* ★顺序很讲究: hfi_enable 必须**最后**置位。
     *   中断(10kHz)随时会闯进来跑 HFI_Tick 里的转速门限, 若先置 en=1、复位完再
     *   hfi_cal_start(), 门限可能在中途把 cal_run 清 0 并关掉使能, 而 SetEnable 接着
     *   又把 cal_run 置 1 —— 就留下 en=0/flt=3/cal=1 这种再也清不掉的状态(现场日志)。 */
    hfi_fault  = 0u;                 /* 重新使能时清故障(与参考实现的 0x10 命令一致) */
    flt2_hinted = 0u;
    flt4_hinted = 0u;
    flt5_hinted = 0u;
    flt6_hinted = 0u;
    scan_auto_done = 0u;             /* 每次使能允许自动补救一轮(避免死循环) */
    rail_retry     = 0u;
    startup_cnt    = 0u;
    count = 0u;                      /* 第一次翻转就换极性(从 1 起会白等一个半周期) */
    inj_tick = 0u;
    sign = 1;
    Voltage_inj = 0.0f;
    hfi_ticks = 0u;

    /* 解调链路复位: 滤波器/累积/环路全部从干净状态起步, 否则会拿旧残值去锁相 */
    BPF_2nd_Init(&BPF_A1, HFI_BPF_B0, HFI_BPF_B1, HFI_BPF_B2, HFI_BPF_A1, HFI_BPF_A2);
    BPF_2nd_Init(&BPF_A2, HFI_BPF_B0, HFI_BPF_B1, HFI_BPF_B2, HFI_BPF_A1, HFI_BPF_A2);
    BPF_2nd_Init(&BPF_B1, HFI_BPF_B0, HFI_BPF_B1, HFI_BPF_B2, HFI_BPF_A1, HFI_BPF_A2);
    BPF_2nd_Init(&BPF_B2, HFI_BPF_B0, HFI_BPF_B1, HFI_BPF_B2, HFI_BPF_A1, HFI_BPF_A2);
    adaptive_notch_init(&Notch_2we, HFI_NOTCH_Q);
    {
        float th = PLL.Thetae;

        PLL_Init(&PLL);
        PLL.Thetae = th;             /* 相位保持连续, 免得重新锁一遍 */
        PLL.cos_PLL = cosf(th);
        PLL.sin_PLL = sinf(th);
    }
    if (on == 0u)
    {
        PLL.cal_run = 0u;            /* 关掉时别留一个"标定中"的假状态 */
        cal_cnt = 0u;
        hfi_enable = 0u;
    }
    else
    {
        hfi_cal_start();             /* 先备好标定窗, 再开门让中断跑 */
        hfi_enable = 1u;
    }
}

/* 转速是否在适用范围内(命令里先问一句, 免得"刚开就 flt=3"让人以为坏了) */
uint8_t HFI_SpeedOk(void)
{
    return (fabsf(hfi_w_enc) > HFI_MAX_W) ? 0u : 1u;
}

uint8_t HFI_ScanBusy(void)
{
    return hfi_scan_en;
}

void HFI_SetB2(float a)
{
    hfi_b2_comp = a;
}

/* 在线改环路阻尼系数(见 HFI_PLL_KD)。限幅在 ±0.05 A·s/rad: 再大就等于把环路增益
 *   砍没了(eps 会被 kd·ω 主导), 那不是"阻尼"而是"瘫痪"。 */
void HFI_SetKd(float kd)
{
    if (kd >  0.05f) kd =  0.05f;
    if (kd < -0.05f) kd = -0.05f;
    hfi_kd = kd;
}

void HFI_SetDemodRot(float rad)
{
    hfi_demod_rot = rad;
    hfi_rot_sin = sinf(rad);
    hfi_rot_cos = cosf(rad);
}

/* ============================================================================
 * HFI_ScanStart —— 启动静态扫描(免锁相): θ̂ = θenc + Δ, Δ 从 0 扫到 165°
 *   ★与 HFI_SetEnable 同一条教训: hfi_enable 必须**最后**置位。
 * ========================================================================== */
void HFI_ScanStart(void)
{
    uint8_t i;

    for (i = 0u; i < HFI_SCAN_STEPS; i++)
    {
        scan_pt[i].off_deg = 0.0f;
        scan_pt[i].err     = 0.0f;
        scan_pt[i].id      = 0.0f;
        scan_pt[i].wmax    = 0.0f;
        scan_pt[i].n       = 0u;
    }
    scan_step    = 0u;
    scan_dwell   = HFI_SCAN_DWELL;
    scan_settle  = HFI_SCAN_SETTLE;
    scan_n       = 0u;
    scan_err_sum = 0.0f;
    scan_id_sum  = 0.0f;
    scan_w_max   = 0.0f;
    scan_th0     = 0.0f;
    scan_pt_ready = 0u;
    scan_retry   = 0u;
    scan_start_cnt++;
    scan_done_flag = 0u;
    hfi_fault    = 0u;
    flt4_hinted  = 0u;
    flt5_hinted  = 0u;
    PLL.cal_run  = 0u;             /* 扫描期间不要标定窗(它会把 θ̂ 抢走) */
    PLL.kth_done = 1u;
    cal_cnt      = 0u;
    count = 0u;
    inj_tick = 0u;
    sign = 1;
    Voltage_inj = 0.0f;
    hfi_ticks = 0u;
    BPF_2nd_Init(&BPF_A1, HFI_BPF_B0, HFI_BPF_B1, HFI_BPF_B2, HFI_BPF_A1, HFI_BPF_A2);
    BPF_2nd_Init(&BPF_A2, HFI_BPF_B0, HFI_BPF_B1, HFI_BPF_B2, HFI_BPF_A1, HFI_BPF_A2);
    BPF_2nd_Init(&BPF_B1, HFI_BPF_B0, HFI_BPF_B1, HFI_BPF_B2, HFI_BPF_A1, HFI_BPF_A2);
    BPF_2nd_Init(&BPF_B2, HFI_BPF_B0, HFI_BPF_B1, HFI_BPF_B2, HFI_BPF_A1, HFI_BPF_A2);
    adaptive_notch_init(&Notch_2we, HFI_NOTCH_Q);
    {
        float th = PLL.Thetae;

        PLL_Init(&PLL);
        PLL.Thetae  = th;
        PLL.cos_PLL = cosf(th);
        PLL.sin_PLL = sinf(th);
    }
    hfi_scan_en = 1u;
    hfi_enable  = 1u;              /* ★最后置位 */
}

/* ============================================================================
 * HFI_ScanReport —— 主循环调用: 扫完把 12 点表和定性结论打出来
 *   拟合 err(Δ) = b + K·sin(2Δ) + C·cos(2Δ)(12 点均匀覆盖 π, 两基向量正交)
 *     · |K| 大、|C| 小、残差小 ⇒ 真凸极(问题在环路/标定)
 *     · |C| ≳ |K|            ⇒ 解调不对称/相位错(死区、采样点、轴不齐)
 * ========================================================================== */
void HFI_ScanReport(void)
{
    static const char *verdict;
    uint8_t i;
    float se = 0.0f, sk = 0.0f, sc = 0.0f, b, k, c, res = 0.0f, snr;
    uint8_t n = 0u;

    if (scan_pt_ready != 0u)
    {
        /* 每完成一档就打一行: 就算整轮被打断, 已有数据也能用 */
        uint8_t k = (uint8_t)(scan_pt_ready - 1u);

        scan_pt_ready = 0u;
        (void)CDC_Printf("# hfiscan step %u/%u d=%+6.1fdeg err=%+.4f id=%+.4f wmax=%.1f n=%u\r\n",
                         (unsigned)(k + 1u), (unsigned)HFI_SCAN_STEPS,
                         (double)scan_pt[k].off_deg, (double)scan_pt[k].err,
                         (double)scan_pt[k].id, (double)scan_pt[k].wmax,
                         (unsigned)scan_pt[k].n);
    }
    if (scan_done_flag == 0u)
    {
        return;
    }
    scan_done_flag = 0u;

    (void)CDC_Printf("# hfiscan 完成: θ̂=θenc+Δ 静态扫描(环路不闭) U=%.2fV, 单位 A\r\n",
                     (double)hfi_v_inj);
    for (i = 0u; i < HFI_SCAN_STEPS; i++)
    {
        (void)CDC_Printf("#   d=%+6.1fdeg err=%+.4f id=%+.4f wmax=%.1f n=%u\r\n",
                         (double)scan_pt[i].off_deg, (double)scan_pt[i].err,
                         (double)scan_pt[i].id, (double)scan_pt[i].wmax,
                         (unsigned)scan_pt[i].n);
        if (scan_pt[i].n > 0u)
        {
            float a2 = 2.0f * scan_pt[i].off_deg * 0.0174532925f;
            /* ★基向量取 −sin/−cos: 扫描自变量是"θ̂ 相对编码器的偏置" Δ_off, 而物理调制角
             *   是 (θenc−θ̂) = −Δ_off。用同一套约定才能和标定窗/环路对齐(否则 K 整体反号,
             *   推出的 ρ 直接差 180° —— 仿真抓出来的)。 */
            float s2 = -sinf(a2);
            float c2 = -cosf(a2);

            se += scan_pt[i].err;
            sk += scan_pt[i].err * s2;
            sc += scan_pt[i].err * c2;
            n++;
        }
    }
    if (n < 4u)
    {
        (void)CDC_Printf("# hfiscan: 有效档数太少(%u), 本次作废(是不是中途转速超限了?)\r\n",
                         (unsigned)n);
        return;
    }
    b = se / (float)n;               /* Σsin2Δ = Σcos2Δ = 0(均匀覆盖 π) ⇒ 均值就是常数项 */
    k = 2.0f * sk / (float)n;        /* Σsin²=Σcos²=n/2 */
    c = 2.0f * sc / (float)n;
    for (i = 0u; i < HFI_SCAN_STEPS; i++)
    {
        float a2, d;

        if (scan_pt[i].n == 0u)
        {
            continue;
        }
        a2 = 2.0f * scan_pt[i].off_deg * 0.0174532925f;
        d = scan_pt[i].err - (b - k * sinf(a2) - c * cosf(a2));   /* 基向量同上取负 */
        res += d * d;
    }
    res = sqrtf(res / (float)n);
    snr = (res > 1.0e-6f) ? (fabsf(k) / res) : 999.0f;

    if ((fabsf(k) > 3.0f * res) && (fabsf(c) < 0.5f * fabsf(k)))
    {
        verdict = "真凸极(sin2Δ): 解调链没问题";
    }
    else if (fabsf(c) >= fabsf(k))
    {
        verdict = "响应主要落在 cos2Δ 上 ⇒ 解调轴相位偏 ~90°(约定/接线/采样相位), 靠转轴修正";
    }
    else if (fabsf(c) >= 0.5f * fabsf(k))
    {
        /* ★现场(2026-10-11)就是这一档: K=0.187 / C=0.135 / 残差=0.027 / snr=6.9
         *   —— 信号其实很好, 只是轴偏了 ½atan(C/K)≈18°, 环路平衡点就被挪走同样的角度。
         *   原来这条被判成"信噪比不足", 日志在说谎 ⇒ 单独一档说清楚。 */
        verdict = "sin2Δ 与 cos2Δ 都不小 ⇒ 解调轴相位有偏(靠转轴修正, 不是噪声)";
    }
    else
    {
        verdict = "信噪比不足: 凸极信号被噪声/不对称淹没";
    }
    (void)CDC_Printf("# hfiscan 拟合: 常数 b=%+.4fA  sin2Δ幅值 K=%+.4fA  cos2Δ幅值 C=%+.4fA\r\n"
                     "#              残差=%.4fA  snr=%.1f  (判据 snr>=3)\r\n"
                     "#              结论: %s\r\n",
                     (double)b, (double)k, (double)c, (double)res, (double)snr, verdict);

    /* ---- 自动定标: 把解调轴转正 + 用免锁相量到的增益 ----
     *   理论(头文件): 解调轴转 ρ 后量到的曲线是 |E|·sin(2Δ−ρ), 拟合给 |E|·sin(2Δ+φ)
     *   ⇒ **ρ = −φ = −atan2(C,K)**。两种情形都对:
     *     · K≈0、C大(现场: 标定窗 kth 撞下限 0.003) ⇒ ρ≈∓90° = 等价于换到另一根轴,
     *       说明注入轴与解调轴的参考差了 90°; 这时**减掉 C 会把凸极信号一起减掉**,
     *       只有转轴才对(转完 C→0、K→|E|)。
     *     · K 与 C 都不小 ⇒ 转完也回到 |E|·sin(2Δ) ✓。 */
    if ((fabsf(k) > 3.0f * res) || (fabsf(c) > 3.0f * res))
    {
        float rho = -atan2f(c, k);

        HFI_SetDemodRot(rho);
        kth_from_scan = 1u;                  /* 环路增益改用免锁相量出来的 |E| */
        hfi_kth       = sqrtf(k * k + c * c);
        hfi_kth_meas  = hfi_kth;
        hfi_snr       = hfi_kth / ((res > 1.0e-6f) ? res : 1.0e-6f);
        /* ★偏置必须和"解调轴转没转"配套:
         *   转了轴 ⇒ 坐标系变了, 这次拟合出的常数项 b **在新坐标系里已经不对**
         *   (现场第一次转 −35° 后 b 从 +0.088 变成 −0.196, 旧值直接让 eps 满量程跑飞);
         *   这时把它作废, 让紧随其后的标定窗在新坐标系里重量一个。
         *   没怎么转(ρ≈0) ⇒ 坐标系没变, 用 12 点拟合的 b 比两点平均干净。 */
        if (fabsf(rho) > 0.035f)          /* >2° 就算真转了轴 */
        {
            hfi_kth_bias   = 0.0f;
            bias_from_scan = 0u;
        }
        else
        {
            hfi_kth_bias   = b;
            bias_from_scan = 1u;
        }
        (void)CDC_Printf("# 自动定标: 解调轴转 %+.1f°(ρ=−atan2(C,K)); 环路增益 |E|=%.4fA,"
                         " snr=%.1f\r\n", (double)(rho * 57.29578f), (double)hfi_kth,
                         (double)hfi_snr);
        (void)CDC_Printf("# → 已重新武装: 马上重开标定窗并接管观测。想复核就再跑一次 hfiscan,"
                         " 应看到 K≈|E|、C≈0\r\n");
        hfi_fault    = 0u;
        flt4_hinted  = 0u;
        nolock_cnt   = 0u;
        PLL.kth_done = 0u;                   /* HFI_Tick 会重开标定窗(现在只用来量 b_dc) */
        PLL.cal_run  = 0u;
        cal_cnt      = 0u;
        hfi_scan_en  = 0u;
        hfi_enable   = 1u;                   /* 注入继续 */
    }
    else
    {
        (void)CDC_Printf("# → |K| 与 |C| 都和残差(%.4fA)同量级: **这台电机/板子上脉冲注入"
                         "出不来可信角度(NO-GO)**, 不是代码问题\r\n", (double)res);
        hfi_fault   = 5u;                    /* 5 = 已扫描仍不可信 ⇒ NO-GO */
        hfi_enable  = 0u;
        hfi_scan_en = 0u;
        Voltage_inj = 0.0f;
        hfi_v_lim   = 0.0f;
    }
}

/* 启动/复位标定窗: θ̂ 会被钉在编码器 ±45° 两个点上, 环路不闭, 24ms 后交接 */
static void hfi_cal_start(void)
{
    cal_cnt    = HFI_CAL_DEMODS;
    cal_phase  = 0u;
    cal_half   = HFI_CAL_HALF;
    cal_skip   = HFI_CAL_SKIP;
    cal_p_sum  = 0.0f;
    cal_m_sum  = 0.0f;
    cal_p_n    = 0u;
    cal_m_n    = 0u;
    nolock_cnt = 0u;
    /* ★已有"扫描定标"的成果就别抹掉: 第二次 hfi 1 时若把 |E|/偏置/转轴清回默认,
     *   环路必然又跑飞一轮(白跑 2.4s 扫描)。只有没扫描过时才回到默认起步值。 */
    if (bias_from_scan == 0u)
    {
        hfi_kth_bias = 0.0f;
    }
    if (kth_from_scan == 0u)
    {
        hfi_kth      = HFI_KTH_DEF;      /* 别把上一次跑飞的假 Kth 带进来 */
        hfi_kth_meas = 0.0f;
    }
    hfi_sal_pct  = 0.0f;             /* ★别留上一次会话的陈旧值(现场 sal=1.3% 就是这么来的) */
    cal_start_cnt++;
    PLL.cal_run = 1u;
    PLL.kth_done = 0u;
}

void HFI_SetAmpl(float volts)
{
    if (volts < HFI_INJ_V_MIN) volts = HFI_INJ_V_MIN;
    if (volts > HFI_INJ_V_MAX) volts = HFI_INJ_V_MAX;
    hfi_v_inj = volts;
}

void HFI_SetKth(float amps)
{
    if (amps < HFI_KTH_MIN) amps = HFI_KTH_MIN;
    if (amps > HFI_KTH_MAX) amps = HFI_KTH_MAX;
    hfi_kth = amps;
}

void HFI_SetNotch(uint8_t on)
{
    hfi_notch_en = on;
    adaptive_notch_init(&Notch_2we, HFI_NOTCH_Q);   /* 状态清零, 免得带着旧残值起跳 */
}

const PLL_control *HFI_GetPLL(void) { return &PLL; }

/* ============================================================================
 * HFI_INJ —— 每拍(10kHz)翻转极性, 输出要叠加到 αβ 电压上的注入矢量
 *
 *   ★方向 = **PLL 估计的 d 轴 θ̂**(不是编码器轴!): 沿编码器轴注入的话电流矢量会
 *     跟着编码器轴走, 解调量变成 ∝sinΔθ 的各向同性项, 凸极根本没参与 —— 那样这个
 *     观测器只是在跟踪自己注入的轴, 验证不了任何东西。tools/verify_hfi.py 的 [1b]
 *     节就是这条的判决性实验: 沿 θ̂ 注入时 err(Δ) 幅度与凸极率成正比(20%:2% = 10:1)。
 *
 *   用的 cos/sin 是上一拍算好的(电流环在 ADC1 分支, θ̂ 在 ADC2 分支更新, 差 100us)。
 *   幅值受硬上限和 SVPWM 线性区限制。关掉时输出 0。
 * ========================================================================== */
void HFI_INJ(float *v_alpha, float *v_beta)
{
    float c, s;

    if (hfi_enable == 0u)
    {
        inj_tick = 0u;
        Voltage_inj = 0.0f;
        *v_alpha = 0.0f;
        *v_beta = 0.0f;
        return;
    }

    inj_tick++;
    if (inj_tick >= HFI_HALF_TICKS)
    {
        inj_tick = 0u;
        count++;
        sign = ((count % 2u) == 0u) ? 1 : -1;      /* 与参考实现相同的序列 */
        Voltage_inj = (float)sign * hfi_v_lim;
    }
    if (++hfi_ticks == 0u) hfi_ticks = 1u;          /* 只用来统计 */

    c = PLL.cos_PLL;
    s = PLL.sin_PLL;
    *v_alpha = Voltage_inj * c;
    *v_beta  = Voltage_inj * s;
}

/* ============================================================================
 * PLL_Proc —— 半周期同步平均解调 + PLL
 *
 *   输入: 带通后的 αβ 高频电流 [A](HFI_Tick 里做的)
 *   输出: PLL.Thetae / Omega_e_F / speed + 诊断量
 *
 *   ★环路(PI + 积分)只在**新解调值**上跑一步: 解调率 = HFI_DEMOD_HZ = 2kHz,
 *     所以 Ts 用 HFI_TS_LOOP。这是相对参考实现的修正 D3。
 * ========================================================================== */
void PLL_Proc(PLL_control *p, float Ialpha_hf, float Ibeta_hf, float theta_enc)
{
    float Iq_hf, Id_hf;
    uint8_t new_sample = 0u;

    /* ---- 1. 角度保持 [0,2π) ---- */
    p->Thetae = Range_to_2PI(p->Thetae);
    p->sin_PLL = sinf(p->Thetae);
    p->cos_PLL = cosf(p->Thetae);

    /* ---- 2. Park 到 PLL 估计坐标系(与 ParkTransformer 同口径) ----
     *   ★解调轴可以整体转 hfi_demod_rot(见头文件推导 ρ=−φ): 用角度和公式展开,
     *   不额外调三角函数(中断预算)。默认 ρ=0 时与原来的 sin_PLL/cos_PLL 完全等价。 */
    {
        float sd = p->sin_PLL * hfi_rot_cos + p->cos_PLL * hfi_rot_sin;
        float cd = p->cos_PLL * hfi_rot_cos - p->sin_PLL * hfi_rot_sin;

        p->x = Ialpha_hf * sd - Ibeta_hf * cd;              /* = −Iq(解调轴) */
        Iq_hf = -p->x;
        Id_hf = Ialpha_hf * cd + Ibeta_hf * sd;
    }
    p->Iq_pll_hf = Iq_hf;

    /* ---- 3. 半周期同步平均解调 ---- */
    p->Iq_acc += Iq_hf;
    p->Id_acc += Id_hf;
    if (p->acc_cnt < 60000u) p->acc_cnt++;

    if ((p->sign_prev != 0) && (sign != p->sign_prev))
    {
        /* 半周期刚结束。acc_cnt 含新半周期的第一个点, 要排除 */
        if (p->acc_cnt > 1u)
        {
            float k = (float)(p->acc_cnt - 1u);
            float Iq_avg = (p->Iq_acc - Iq_hf) / k;
            float Id_avg = (p->Id_acc - Id_hf) / k;

            new_sample = 1u;
            p->err_raw = (float)p->sign_prev * Iq_avg;
            /* 半周期平均本身就是低通(参考实现的判断), 直通; 陷波去 2·ωe 谐波 */
            p->err_lpf = p->err_raw;
            p->Iq_demod = p->err_lpf;                 /* 诊断: 同一时刻的信号 */
            p->Id_demod = (float)p->sign_prev * Id_avg;  /* 诊断: 载波电平 */
            /* 2·ωe 自适应陷波: ★默认旁路(见头文件第 5 节: 静止时它会落在环路带宽里) */
            if (hfi_notch_en != 0u)
            {
                p->notch_fc_hz = fabsf(p->Omega_e_F) / HFI_PI;
                p->err_notched = adaptive_notch_update(&Notch_2we, p->err_lpf,
                                                       p->notch_fc_hz);
            }
            else
            {
                p->notch_fc_hz = 0.0f;
                p->err_notched = p->err_lpf;
            }

            /* ---- 3b. 标定窗: 把 θ̂ 钉在编码器 ±45° 两个点上, 解 Kth 与偏置 b ----
             *   只观测模式才有这个条件(编码器角是真值), 用它当激励源:
             *       err(+45°) = b + Kth,   err(−45°) = b − Kth
             *   两点一减一加就把"增益"和"解调偏置"分开 —— 偏置是死区/PWM 不对称/
             *   传感器失配的直接量度, 只报告不进环路; 增益 Kth 冻结后给环路用。
             *   ★为什么必须这么做: 直接闭环让环路自己去"标定", 它几毫秒就把 Δθ 拉到
             *     0(没有激励), 再把 BPF 起动瞬态一起拟合进去 —— 实测(2026-10 现场)
             *     结果就是 w 钉在 ±450rad/s、kth 掉到下限、角度全垃圾。
             *   窗内 θ̂ 跟着编码器走(所以转子转不转都不影响), 偏置固定 ±45°。 */
            {
                float d  = Fold_half_pi(Wrap_pm(theta_enc - p->Thetae));
                float s2 = sinf(2.0f * d);
                float e  = p->err_raw;

                p->theta_err = Wrap_pm(p->Thetae - theta_enc);
                /* 残差(噪声功率): 锁上之后 err_raw 里只剩噪声, 正好当噪声底 */
                p->kth_res += HFI_KTH_FORGET *
                              ((e - hfi_kth * s2) * (e - hfi_kth * s2) - p->kth_res);
                if (hfi_scan_en != 0u)
                {
                    /* ---- 静态扫描: θ̂ 钉在 θenc + 固定偏置, **环路完全不参与** ----
                     *   ⇒ 不可能跑飞, 量到的 err 就是解调链本身在这个夹角下的输出。 */
                    if (scan_settle > 0u)
                    {
                        scan_settle--;              /* 换偏置后先让电流/带通稳下来 */
                    }
                    else
                    {
                        scan_err_sum += p->err_raw;
                        scan_id_sum  += p->Id_demod;
                        if (scan_n == 0u)
                        {
                            scan_th0 = theta_enc;           /* 这一档净转角的起点 */
                        }
                        scan_n++;
                        if (fabsf(hfi_w_enc) > scan_w_max)      /* 质量指标(仅记录) */
                        {
                            scan_w_max = fabsf(hfi_w_enc);
                        }
                    }
                    if (scan_dwell > 0u)
                    {
                        scan_dwell--;
                    }
                    else if (scan_n > 0u)
                    {
                        if (fabsf(Wrap_pm(theta_enc - scan_th0)) > HFI_SCAN_DTH_MAX)
                        {
                            /* ★这一档期间轴**净转过**这么多 ⇒ 该档数据作废, 只重来这一档
                             *   (判据用净转角, 不用瞬时角速度: 后者被码盘量化噪声淹没) */
                            if (++scan_retry <= HFI_SCAN_RETRY)
                            {
                                scan_dwell   = HFI_SCAN_DWELL;
                                scan_settle  = HFI_SCAN_SETTLE;
                                scan_err_sum = 0.0f;
                                scan_id_sum  = 0.0f;
                                scan_n       = 0u;
                                scan_w_max   = 0.0f;
                            }
                            else
                            {
                                /* 重来这么多次还按不住 ⇒ 明确报"轴在动", 别给半截表 */
                                hfi_fault   = 6u;       /* 6 = 扫描期间轴在动 */
                                hfi_enable  = 0u;
                                hfi_scan_en = 0u;
                                Voltage_inj = 0.0f;
                                hfi_v_lim   = 0.0f;
                            }
                        }
                        else
                        {
                            scan_pt[scan_step].off_deg = HFI_SCAN_STEP_DEG * (float)scan_step;
                            scan_pt[scan_step].err = scan_err_sum / (float)scan_n;
                            scan_pt[scan_step].id  = scan_id_sum  / (float)scan_n;
                            scan_pt[scan_step].wmax = scan_w_max;
                            scan_pt[scan_step].n   = scan_n;
                            scan_pt_ready = (uint8_t)(scan_step + 1u);  /* 主循环: 每档打一行 */
                            scan_err_sum = 0.0f;
                            scan_id_sum  = 0.0f;
                            scan_n       = 0u;
                            scan_w_max   = 0.0f;
                            scan_retry   = 0u;
                            scan_step++;
                        }
                        if ((scan_step >= HFI_SCAN_STEPS) && (hfi_scan_en != 0u))
                        {
                            scan_done_flag = 1u;    /* 主循环去打印(ISR 里不能打) */
                            hfi_scan_en = 0u;
                            hfi_enable  = 0u;       /* 扫完自动撤注入(安全) */
                            Voltage_inj = 0.0f;
                            hfi_v_lim   = 0.0f;
                        }
                        else if (hfi_scan_en != 0u)
                        {
                            scan_dwell  = HFI_SCAN_DWELL;
                            scan_settle = HFI_SCAN_SETTLE;
                        }
                    }
                    p->Thetae = Range_to_2PI(theta_enc + (HFI_SCAN_STEP_DEG * 0.0174532925f)
                                                        * (float)scan_step);
                }
                else if (p->cal_run != 0u)
                {
                    if (cal_skip > 0u)
                    {
                        cal_skip--;              /* 换点后先让电流/带通稳下来, 这些样本不进统计 */
                    }
                    else if (cal_phase == 0u)
                    {
                        cal_p_sum += e; cal_p_n++;   /* +45° 点 */
                    }
                    else
                    {
                        cal_m_sum += e; cal_m_n++;   /* −45° 点 */
                    }
                    if (--cal_half == 0u)
                    {
                        cal_half = HFI_CAL_HALF;
                        cal_phase ^= 1u;
                        cal_skip = HFI_CAL_SKIP;
                    }
                    /* 窗内: θ̂ 钉在编码器 ±45° 上(注入方向随之固定) */
                    p->Thetae = Range_to_2PI(theta_enc +
                                             ((cal_phase != 0u) ? -HFI_CAL_OFFSET
                                                                :  HFI_CAL_OFFSET));
                    if (--cal_cnt == 0u)
                    {
                        /* ---- 标定结束: 出 Kth / 偏置, 并做零误差+速度前馈的交接 ---- */
                        if ((cal_p_n > 0u) && (cal_m_n > 0u))
                        {
                            float ep = cal_p_sum / (float)cal_p_n;   /* θ̂ = θenc+45° */
                            float em = cal_m_sum / (float)cal_m_n;   /* θ̂ = θenc−45° */
                            /* ★符号: err = +Kth·sin(2(θenc−θ̂)) ⇒
                             *   +45° 点 err = b − Kth,  −45° 点 err = b + Kth
                             *   ⇒ Kth = (em − ep)/2,  b = (em + ep)/2
                             *   (第一版写反了, 结果 Kth 拟成负数撞下限 —— 仿真抓出来的) */
                            float km = 0.5f * (em - ep);
                            float bm = 0.5f * (em + ep);

                            if (km < HFI_KTH_MIN) km = HFI_KTH_MIN;
                            if (km > HFI_KTH_MAX) km = HFI_KTH_MAX;
                            if (kth_from_scan == 0u)
                            {
                                hfi_kth = km;            /* 增益只在没有扫描值时才用标定窗的 */
                                hfi_kth_meas = km;
                            }
                            if (bias_from_scan == 0u)
                            {
                                hfi_kth_bias = bm;   /* 有扫描值就别用这两点平均的(被角误差污染) */
                            }
                            hfi_kth_rms = sqrtf(p->kth_res);
                            if (hfi_kth_rms > 1.0e-6f)
                            {
                                hfi_snr = hfi_kth_meas / hfi_kth_rms;
                            }
                            if (fabsf(p->Id_demod) > 1.0e-4f)
                            {
                                hfi_sal_pct = 100.0f * (hfi_kth_meas / fabsf(p->Id_demod))
                                              / HFI_SAL_K;
                            }
                            /* ★★ 标定可信性判据: 只有在**完全没做过定标**时才拒绝 ——
                             *   标定窗在 ±45° 处 cos2Δ 恒为 0, 它**永远量不到**相位/不对称;
                             *   `|bias| ≥ kth` 只说明"响应被转到了 cos2Δ 上(sin2Δ 分量≈0)",
                             *   需要先 hfiscan 把解调轴转正(见 HFI_ScanReport)。
                             *   定标做完后再拒绝就是把自己锁在门外。 */
                            if ((fabsf(bm) >= km) && (hfi_demod_rot == 0.0f) &&
                                (fabsf(hfi_b2_comp) < 1.0e-5f) && (kth_from_scan == 0u))
                            {
                                hfi_fault   = 4u;      /* 标定不可信(报告里能看到) */
                                p->cal_run  = 0u;
                                p->kth_done = 0u;
                                cal_cnt     = 0u;
                                if (scan_auto_done == 0u)
                                {
                                    /* ★自动补救: 下一步本来就是"跑 hfiscan 量 K/C 并把解调轴
                                     *   转正", 这里直接自己做 —— 现场连续两次都是发成 hfi 1
                                     *   然后看到一片 flt=4, 所以不再让用户手工接力。
                                     *   ★注入**保持开**(扫描要用它); 2.4s 后由 HFI_ScanReport
                                     *   决定"转轴后继续"还是"判 NO-GO(flt=5)"。 */
                                    scan_auto_done = 1u;
                                    scan_auto_req  = 1u;   /* 在 HFI_Tick 里启动(别在此重置 PLL) */
                                }
                                else
                                {
                                    hfi_fault   = 5u;      /* 5 = 已扫描仍不可信 ⇒ NO-GO */
                                    hfi_enable  = 0u;
                                    Voltage_inj = 0.0f;
                                    hfi_v_lim   = 0.0f;
                                }
                            }
                        }
                        p->kth_done = 1u;
                        p->cal_run  = 0u;
                        /* 交接: θ̂ 直接对到编码器(零误差); 积分器/速度预置成编码器速度
                         *   ⇒ 环路一上来就在平衡点附近, 不会 wind-up。角度仍是 HFI 自己解的。 */
                        p->Thetae = Range_to_2PI(theta_enc);
                        p->I_Partern = 0.0f;      /* ★不要拿 hfi_w_enc 预置: 静止时它是
                                                   *   码盘量化噪声(现场见过 +96.8rad/s),
                                                   *   等于开局先给环路一记大扰动 */
                        p->Omega_e   = 0.0f;
                        p->Omega_e_F = 0.0f;
                        p->err_notched = 0.0f;
                        p->P_Partern = 0.0f;
                        startup_cnt  = HFI_START_DEMODS;   /* ★软起动: 先纯 P 拉正, 再放 PI */
                        nolock_cnt = 0u;
                    }
                }
            }
        }
        /* 当前点作为新半周期的第一个有效样本 */
        p->Iq_acc = Iq_hf;
        p->Id_acc = Id_hf;
        p->acc_cnt = 1u;
    }
    p->sign_prev = sign;

    /* ---- 4. 环路: 只在新解调值上推进一步(2kHz); 标定窗/静态扫描内**不闭** ---- */
    if ((new_sample != 0u) && (p->cal_run == 0u) && (hfi_scan_en == 0u))
    {
        float kth = (hfi_kth > HFI_KTH_MIN) ? hfi_kth : HFI_KTH_MIN;
        float dth = Wrap_pm(theta_enc - p->Thetae);   /* 实际 Δθ(不能折到 ±π/2: cos2Δ 会变号) */
        /* ★先减掉三样**已知的**解调缺陷, 再进 PI:
         *   ① b_dc —— 标定窗在 ±45° 量到的常数偏置;
         *   ② b2·cos(2Δθ) —— 硬件不对称的随角分量(hfiscan 量出后填 hfi_b2_comp);
         *   ③ kd·ω —— **与注入轴转速同向的假误差**(现场实测的负阻尼, 见 HFI_PLL_KD)。
         *   不减的话 II 型环会把它们当"频差"用积分器吃掉 ⇒ 跑飞(现场)。 */
        float eps = (p->err_notched - hfi_kth_bias
                     - hfi_b2_comp * cosf(2.0f * dth)
                     - hfi_kd * p->Omega_e_F) / kth;

        if (eps >  1.0f) eps =  1.0f;
        if (eps < -1.0f) eps = -1.0f;

        if (startup_cnt != 0u)
        {
            /* ---- 起动窗: **纯比例 + 小限幅**, 积分器保持 0 ----
             *   交接瞬间注入轴跳了 45°, 带通/半周期平均还在暂态里, 头几拍 eps 是满量程;
             *   若这时积分, 十几拍就把 I 顶到上限, 之后再也下不来(现场: 交接后 8ms
             *   ω 就钉在 ±450, 直到撤注入)。这里让比例项用有限速度把轴拉到零误差附近,
             *   暂态过去后再放开 PI(积分器从 0 起步)。 */
            startup_cnt--;
            p->P_Partern = eps * p->Kp;
            p->I_Partern = 0.0f;
            p->Omega_e   = p->P_Partern;
            if (p->Omega_e >  HFI_START_W_MAX) p->Omega_e =  HFI_START_W_MAX;
            if (p->Omega_e < -HFI_START_W_MAX) p->Omega_e = -HFI_START_W_MAX;
        }
        else
        {
            p->P_Partern = eps * p->Kp;
            p->I_Partern += eps * p->Ki * HFI_TS_LOOP;
            if (p->I_Partern >=  HFI_PLL_I_LIMIT) p->I_Partern =  HFI_PLL_I_LIMIT;
            else if (p->I_Partern <= -HFI_PLL_I_LIMIT) p->I_Partern = -HFI_PLL_I_LIMIT;
            p->Omega_e = p->P_Partern + p->I_Partern;
        }
        /* ★硬限幅: 跑飞时不许让注入轴以远超环路带宽的速度自转(见 HFI_PLL_W_LIMIT 的说明) */
        if (p->Omega_e >  HFI_PLL_W_LIMIT) p->Omega_e =  HFI_PLL_W_LIMIT;
        if (p->Omega_e < -HFI_PLL_W_LIMIT) p->Omega_e = -HFI_PLL_W_LIMIT;
        /* 角速度低通(状态放结构体里, 不再用函数内 static) */
        p->Omega_e_F += HFI_OMEGA_LPF_ALPHA * (p->Omega_e - p->Omega_e_F);

        /* 电机侧 rpm × 1024(Q10, 沿用参考实现的口径) */
        p->speed = (int32_t)((p->Omega_e_F * 60.0f * 1024.0f) / (HFI_TWO_PI * HFI_NPP));

        /* 角度积分用滤波后角速度 */
        p->Thetae += p->Omega_e_F * HFI_TS_LOOP;

        /* ---- 4b. 失锁/自持振荡检测 ----
         *   ① |eps| 长时间饱和; ② |ω_F| 贴着限幅。
         *   判据用**慢衰减计数器**(见下), 因为跑飞是"积分器在 ±200 之间来回顶死、
         *   轴在 ±限幅之间反复扫"的自持振荡, 过零点会把连续计数打断。 */
        /* ★起动窗内不判失锁: 交接暂态本来就是满量程饱和, 判了会立刻误报 */
        if ((startup_cnt == 0u) &&
            ((fabsf(eps) >= HFI_NOLOCK_EPS) ||
             (fabsf(p->Omega_e_F) >= HFI_RAIL_FRAC * HFI_PLL_W_LIMIT)))
        {
            if (nolock_cnt < 60000u) nolock_cnt++;
        }
        else if (nolock_cnt > 0u)
        {
            /* ★慢衰减, 不是清 0: 跑飞时积分器在 ±200 之间来回顶死, 轴也在 ±限幅之间
             *   反复扫(**负阻尼自持振荡**, 见 HFI_PLL_KD), 每次过零点都有 1~2 拍
             *   掉回阈值内 —— 直接清 0 的话计数器永远到不了门限(现场 45s 一次 flt=2
             *   都没报, 就是这条)。 */
            nolock_cnt--;
        }
        if ((startup_cnt == 0u) && (nolock_cnt >= HFI_NOLOCK_DEMODS))
        {
                /* ★★ 环路跑飞/自持振荡。处置顺序: 重开标定窗(重新交接+软起动)重试 → 静态扫描
                 *   量 K/C 并转解调轴 → 都无效才判死。见 HFI_START_DEMODS / HFI_PLL_KD。
                 *   ⚠ 不能在 PLL_Proc 里直接调 HFI_ScanStart(它会重置 PLL 状态,
                 *     而这个结构体正被本函数用着) ⇒ 打请求, HFI_Tick 顶部执行。 */
                nolock_cnt  = 0u;
                p->cal_run  = 0u;
                p->kth_done = 0u;
                cal_cnt     = 0u;
                rail_note   = 1u;         /* 主循环打一条解释 */
                if (rail_retry < HFI_RAIL_RETRY)
                {
                    /* ★先重试: 跑飞多半是交接暂态把积分器顶死了(见 HFI_START_DEMODS),
                     *   重开标定窗 = 重新交接 + 软起动, 代价只有 32ms。
                     *   ⚠ 这里**不能置 hfi_fault**: 标定窗的启动条件要求 flt==0。 */
                    rail_retry++;
                }
                else if (scan_auto_done == 0u)
                {
                    hfi_fault      = 2u;
                    scan_auto_done = 1u;
                    scan_auto_req  = 1u;  /* 重试都不行 ⇒ 转开环静态扫描量 K/C */
                }
                else
                {
                    hfi_fault   = 2u;     /* 扫过一轮仍跑飞 ⇒ 不再自动重试 */
                    hfi_enable  = 0u;
                    Voltage_inj = 0.0f;
                    hfi_v_lim   = 0.0f;
                }
        }
    }
}

/* ============================================================================
 * HFI_Tick —— 每拍一次: 前端带通 → 解调 → 保护/计时
 * ========================================================================== */
void HFI_Tick(float ialpha, float ibeta, float vdc, float theta_enc)
{
    uint32_t t0;
    float a_hf, b_hf;

    /* ---- 0) 编码器电角速度估计: ★必须**每拍都跑**, 哪怕注入关着 ----
     *   否则 hfi_th_prev 会陈旧: 关着的时候转子转了, 下次使能第一拍算出的 dw 是
     *   "整段位移"的卷绕值 → 速度估计炸成上千 rad/s → 转速门限立刻误判 flt=3。
     *   (现场日志 en=0 flt=3 w=-1361rad/s 就是这么来的)
     *   fc≈80Hz 低通; |dw| 超 HFI_WENC_DW_MAX 判为毛刺/陈旧, 丢掉这拍重新起算。 */
    {
        float dw = Wrap_pm(theta_enc - hfi_th_prev);

        if ((hfi_th_first != 0u) && (fabsf(dw) < HFI_WENC_DW_MAX))
        {
            hfi_w_enc += 0.05f * (dw / HFI_TS - hfi_w_enc);
        }
        else
        {
            hfi_w_enc = 0.0f;
        }
        hfi_th_prev  = theta_enc;
        hfi_th_first = 1u;
    }

    if (scan_auto_req != 0u)
    {
        /* 标定窗判定"不可信"后自动跑一轮静态扫描(见标定交接处)。
         * ★必须在这里(不在 PLL_Proc 里)调用: HFI_ScanStart 会重置 PLL,
         *   而 PLL_Proc 正在用那个结构体。 */
        scan_auto_req = 0u;
        HFI_ScanStart();
    }
    if (hfi_enable == 0u)
    {
        /* 扫描中途被撤注入(过流/超速): 别留一个"scan_en=1 但没数据"的挂起状态 */
        if (hfi_scan_en != 0u)
        {
            hfi_scan_en = 0u;
        }
        hfi_v_lim = 0.0f;
        return;
    }
    if (Encoder_IsValid() == 0u)
    {
        /* 编码器无效: 标定窗要拿它当激励基准, 先别跑(注入那边也会被关掉) */
        return;
    }

    t0 = DWT->CYCCNT;

    /* 本拍可用幅值(受 vdc 限制时记一笔) */
    {
        float lim = HFI_VLim(vdc);

        if (lim < hfi_v_inj) hfi_clamp_cnt++;
        hfi_v_lim = lim;
    }

    /* ---- 转速门限: 超出适用范围就明确拒绝, 不给"看着像角度的垃圾" ----
     *   实测偏差随转速线性长(头文件 HFI_MAX_W 有实测表): 10Hz(86rpm) −12°、
     *   24Hz(205rpm) −25°。所以 ① 转子在转时不启动标定窗(两点会被正交泄漏污染);
     *   ② 标定好了但转速超限 → 撤注入 + flt=3, 让用户知道是"超范围"不是"坏了"。
     *   ★扫描期间**不套这个门限**: 扫描是静态测量, 每档自己判"轴动没动"
     *     (超了只作废那一档重来, 见 HFI_SCAN_W_MAX), 犯不着因为手碰一下把 2.4s 白跑。 */
    if ((hfi_scan_en == 0u) && (fabsf(hfi_w_enc) > HFI_MAX_W))
    {
        /* 超出适用范围: 明确拒绝 + 撤注入。正在标定窗里也一并作废(半截数据不能用),
         * 否则会拿着被转速污染的两点算出一个假 Kth。 */
        hfi_fault   = 3u;            /* 3 = 超出适用转速 */
        hfi_enable  = 0u;
        hfi_scan_en = 0u;            /* 扫描中被打断: 作废(表会只有半截) */
        PLL.cal_run = 0u;
        PLL.kth_done = 0u;
        cal_cnt     = 0u;
        Voltage_inj = 0.0f;
        hfi_v_lim   = 0.0f;
        return;
    }

    /* ---- 使能后第一拍: 启动标定窗 ----
     *   窗内 θ̂ 会被钉在编码器 ±45° 两个点上(环路不闭), 24ms 后带着实测 Kth
     *   零误差交接。★只改观测器初值/状态, 不碰驱动角。 */
    if ((PLL.cal_run == 0u) && (PLL.kth_done == 0u) && (hfi_fault == 0u))
    {
        hfi_cal_start();
    }

    /* ---- 前端: αβ 各两级级联带通(中心 = 注入 1kHz) ----
     *   必须用**未滤波**的 αβ 电流: 注入是 1kHz 方波, 中值/LPF 都会把它修掉。 */
    a_hf = BPF_2nd_Update(&BPF_A2, BPF_2nd_Update(&BPF_A1, ialpha));
    b_hf = BPF_2nd_Update(&BPF_B2, BPF_2nd_Update(&BPF_B1, ibeta));

    /* ---- 解调 + PLL ---- */
    PLL_Proc(&PLL, a_hf, b_hf, theta_enc);

    /* ---- 打点快照: PLL 角折到 [0,2π)(可直接和 corr 叠加看), err 折到 ±π/2 ---- */
    hfi_theta_stream = Range_to_2PI(PLL.Thetae);
    hfi_err_stream   = Fold_half_pi(PLL.theta_err);

    /* ---- 过流保护: 解调出的高频电流超限 ⇒ 立刻撤注入 + 报故障 ----
     *   (注入是外加电压, 不能只靠总电流保护: 高频分量会被电流环的滤波器吃掉) */
    if (fabsf(PLL.Id_demod) > HFI_I_LIMIT)
    {
        hfi_fault = 1u;
        hfi_enable = 0u;
        Voltage_inj = 0.0f;
        hfi_v_lim = 0.0f;
    }

    /* ---- 耗时峰值(DWT, us) ---- */
    {
        float us = HFI_Us((uint32_t)(DWT->CYCCNT - t0));

        if (us > hfi_us_peak) hfi_us_peak = us;
    }
}

/* ============================================================================
 * 主循环 20Hz 报告
 * ========================================================================== */
void HFI_Report(void)
{
    static uint32_t last_ms;
    static uint8_t  tag_done;
    uint32_t now = HAL_GetTick();

    /* ★构建指纹: 每次上电打一次(在"注入没开就 return"之前)。
     *   现场多次出现"日志行为像是旧固件", 有了这一行就能和 .elf 的时间戳对账,
     *   不必再猜烧的是哪一版。 */
    if (tag_done == 0u)
    {
        tag_done = 1u;
        (void)CDC_Printf("# hfi build: %s %s (对上 .elf 的编译时间就说明烧的是这一版)\r\n",
                         __DATE__, __TIME__);
    }

    if ((hfi_enable == 0u) && (hfi_fault == 0u) && (hfi_ticks == 0u)) return;
    if ((now - last_ms) < 50u) return;
    last_ms = now;

    if (rail_note != 0u)
    {
        /* ★事件标志, 不看 hfi_fault: 扫描一旦启动就会把 flt 清 0, 靠 flt 打这条会漏掉 */
        rail_note = 0u;
        if (flt2_hinted == 0u)
        {
            flt2_hinted = 1u;
            (void)CDC_Printf("# ⚠ 环路跑飞(ω 钉在 ±%.0f rad/s 限幅上)。多为**交接暂态**把积分器顶死"
                             "(注入轴刚跳过 45°), 已自动重开标定窗重试(软起动); 重试仍跑飞才转"
                             "静态扫描。请保持静止。若反复如此, 试 `hfik 4` 把环路增益降下来\r\n",
                             (double)HFI_PLL_W_LIMIT);
        }
    }

    /* flt=4 只提示一次"下一步该干嘛"(在主循环里打; 中断里绝对不能 CDC_Printf) */
    if ((hfi_fault == 4u) && (flt4_hinted == 0u))
    {
        flt4_hinted = 1u;
        (void)CDC_Printf("# ⚠ flt=4: 标定窗量到 |bias|>=kth ⇒ 响应被转到 cos2Δ 上(sin2Δ≈0)。"
                         "已自动转入静态扫描(2.4s, 环路不闭); **请保持静止**, 扫完自动转轴并重标\r\n");
    }
    if ((hfi_fault == 5u) && (flt5_hinted == 0u))
    {
        flt5_hinted = 1u;
        (void)CDC_Printf("# ⚠ flt=5: 自动扫描也没量到可信凸极(K/C 与残差同量级) ⇒ 这套在这块"
                         "板子上出不来可信角度(NO-GO)。可试 hfiu 1.0 后重跑 hfiscan 复核\r\n");
    }
    if ((hfi_fault == 6u) && (flt6_hinted == 0u))
    {
        flt6_hinted = 1u;
        (void)CDC_Printf("# ⚠ flt=6: 扫描期间轴一直按不住(单档重来 %u 次净转角仍超 %.0f°)。"
                         "把轴夹住/用手扶稳再 hfi 1; 注入本身会让转子缓慢爬行, 属正常\r\n",
                         (unsigned)HFI_SCAN_RETRY, (double)(HFI_SCAN_DTH_MAX * 57.29578f));
    }

    (void)CDC_Printf("# hfi en=%u flt=%u nt=%u cal=%u U=%.2f/%.2fV id=%+.3f iq=%+.4f "
                     "kth=%.4f(meas %.4f) bias=%+.4f snr=%.1f sal=%.1f%% "
                     "err=%+.1fdeg w=%+.1f/%+.0frad/s rpm=%+.0f "
                     "notch=%.1fHz us=%.2f clamps=%u ncal=%u nsc=%u step=%u\r\n",
                     (unsigned)hfi_enable, (unsigned)hfi_fault,
                     (unsigned)hfi_notch_en, (unsigned)PLL.cal_run,
                     (double)hfi_v_lim, (double)hfi_v_inj,
                     (double)PLL.Id_demod, (double)PLL.Iq_demod,
                     (double)hfi_kth, (double)hfi_kth_meas,
                     (double)hfi_kth_bias,
                     (double)hfi_snr, (double)hfi_sal_pct,
                     (double)(hfi_err_stream * 57.29578f),
                     (double)PLL.Omega_e_F, (double)hfi_w_enc,
                     (double)(PLL.Omega_e_F * 60.0f / (HFI_TWO_PI * HFI_NPP)),
                     (double)PLL.notch_fc_hz, (double)hfi_us_peak,
                     (unsigned)hfi_clamp_cnt,
                     (unsigned)cal_start_cnt, (unsigned)scan_start_cnt,
                     (unsigned)scan_step);
}

float HFI_GetStreamTheta(void) { return hfi_theta_stream; }
float HFI_GetStreamErr(void)   { return hfi_err_stream; }
