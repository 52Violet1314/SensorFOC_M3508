#include "App_FOC.h"
#include "BSP_AdcIrq.h"
#include "App_Config.h"
#include "adc.h"
#include "BSP_CAN.h"
#include "BSP_Timer.h"
#include "Control.h"
#include "Encoder.h"
#include "FOC_Param.h"
#include "gpio.h"
#include "SVPWM.h"
#include "SMO.h"
#include "tim.h"
#include "Transformer.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"
#include "VF.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

float ADC_Current[3];
float ADC_Voltage[3];
float ADC_Power;
float ADC_Temp;
float Current[3];
float ialpha;
float ibeta;
float ia_filt;
float ib_filt;
float cur_mag;
float cur_mag_filt;
float id;
float iq;
float id_filt;
float iq_filt;
float Voltage[3];
float Power;
float Temp;
int Encoder_Sin;
int Encoder_Cos;
float Encoder_Angle;
float Encoder_Elec_Angle;
float Encoder_Mech_Position;
float csa_vmid[3] = {2047.0f, 2047.0f, 2047.0f};
float spd_meas_rpm;

/* 观测角对照用的常数偏置(诊断): 观测角 + Lag + 偏置 应等于编码器角 */
float obs_angle_off = OL_ANGLE_OFF_DEF;

static volatile uint8_t adc_cal_done;
static uint32_t cal_sum[3];
static uint32_t cal_count[3];
static volatile uint8_t adc_group_mask;
static uint8_t isr_cyc_reported;
static uint32_t encoder_last_valid_ms;
static uint32_t can_feedback_ms;
static uint32_t heartbeat_ms;

static float cur_id_ref;
static float cur_iq_ref;
static float cur_id_err;
static float cur_iq_err;
static float cur_vd;
static float cur_vq;
static float speed_ref_cmd_rpm;
static float pos_err_rad;
static float cur_mech_prev;
static float cur_spd_accum;
static float spd_err_rpm;
static uint16_t cur_spd_count;
static uint16_t speed_loop_count;
static uint8_t cur_mech_valid;
static Control_Mode_t previous_mode = CONTROL_MODE_SPEED;
static volatile uint8_t vdc_fault;
static uint32_t vdc_low_count;
static uint32_t vdc_ok_count;

typedef struct
{
    float sample[APP_CURRENT_FILTER_WINDOW];
    uint8_t index;
    float low_pass;
} CurrentFilter_t;

static CurrentFilter_t ia_filter;
static CurrentFilter_t ib_filter;
static CurrentFilter_t id_filter;
static CurrentFilter_t iq_filter;

/* ==========================================================================
 * SMO 打点: 1kHz 采样 -> 环形缓冲 -> 主循环格式化走 USB CDC
 *
 *   为什么中断里不直接 printf: vsnprintf 里 13 个浮点格式化要 50~100us,
 *   40kHz 中断预算只有 25us, 必超。所以中断只搬 13 个 float(约 30 拍),
 *   格式化和 USB 发送全放主循环(1ms 一次, 排空缓冲)。
 * ========================================================================== */
#define SMO_STREAM_HZ        1000u
#define SMO_STREAM_BUF_LEN   64u          /* 2 的幂, 便于取模 */
#define SMO_STREAM_BUF_MASK  (SMO_STREAM_BUF_LEN - 1u)

typedef struct
{
    uint32_t t_ms;
    float theta_obs;   /* 观测电角度 rad */
    float theta_enc;   /* 编码器电角度 rad(真值) */
    float err;         /* 观测 − 编码器, 归一化到 [-pi,pi) */
    float emf;         /* |E| V */
    float omega_obs;   /* 观测电角速度 rad/s */
    float rpm_obs;     /* 观测电机侧转速 rpm */
    float rpm_enc;     /* 编码器差分转速 rpm */
    float ialpha;
    float ibeta;
    float valpha;
    float vbeta;
    float vdc;
    float imag;        /* 电流矢量幅值 A  ← 判断堵转/限流的关键列 */
    float vcmd;        /* 本拍输出电压幅值 V */
    uint8_t flags;     /* bit0 running, bit1 boost_lo, bit2 stall, bit3 aligned */
} Smo_Sample_t;

static volatile Smo_Sample_t smo_buf[SMO_STREAM_BUF_LEN];
static volatile uint8_t  smo_head;        /* 中断写 */
static volatile uint8_t  smo_tail;        /* 主循环读 */
static volatile uint8_t  smo_stream_on;
static volatile uint32_t smo_dropped;
static uint32_t smo_last_us;
static uint32_t smo_stat_ms;

/* 中断统计(供 DIAG 块使用, 定义提前到这里以免被下面的函数看不到) */
static uint32_t prof_ticks_total;   /* 累计进入中断的次数 */
static uint32_t isr_over_cnt;       /* 中断本体超过预算的次数(丢拍证据) */

/* 方向自检: 编码器在转时, 观测角每拍增量是否与编码器同号(同号才说明符号口径对)。
 *   观测角是镜像形式(atan2(+e_alpha,...))时, 这两个计数会一边倒 —— 一次上电测试就能判定。 */
static uint32_t smo_dir_ok;
static uint32_t smo_dir_bad;
static float    smo_theta_prev_isr;

static void App_PrintStatus(void);

/* DWT 周期计数 -> us(打点时间戳用, 32 位计数器 25.6s 回绕, 只用于间隔判断) */
static inline uint32_t App_NowUs(void)
{
    return DWT->CYCCNT / (SystemCoreClock / 1000000u);
}

/* 观测角与编码器角的差, 归一化到 [-pi, pi)
 * 观测角要补上相位滞后(SMO.Output.Lag)再和编码器角比, 否则会看到一个随转速漂的偏置 */
static float App_AngleDiff(float a, float b)
{
    float d = a - b;
    while (d >= APP_PI) d -= APP_TWO_PI;
    while (d < -APP_PI) d += APP_TWO_PI;
    return d;
}

static float App_ObsCompensated(void)
{
    return SMO.Output.Theta + SMO.Output.Lag + obs_angle_off;
}

static void App_SmoStreamTick(uint32_t now_ms, uint32_t now_us)
{
    Smo_Sample_t *s;
    uint8_t next;

    if (smo_stream_on == 0u) return;
    if ((uint32_t)(now_us - smo_last_us) < (1000000u / SMO_STREAM_HZ)) return;
    smo_last_us = now_us;

    next = (uint8_t)((smo_head + 1u) & SMO_STREAM_BUF_MASK);
    if (next == smo_tail)
    {
        smo_dropped++;      /* 主循环没跟上, 丢一帧(摘要行里会报出来) */
        return;
    }

    s = (Smo_Sample_t *)&smo_buf[smo_head];
    s->t_ms      = now_ms;
    s->theta_obs = SMO.Output.Theta;
    s->theta_enc = SMO.Output.ThetaEnc;
    s->err       = App_AngleDiff(App_ObsCompensated(), SMO.Output.ThetaEnc);
    s->emf       = SMO.Output.Emf;
    s->omega_obs = SMO.Output.Speed;
    s->rpm_obs   = SMO.Output.Rpm;
    s->rpm_enc   = vf_speed_meas_rpm;
    s->ialpha    = SMO.Input.Ialpha;
    s->ibeta     = SMO.Input.Ibeat;
    s->valpha    = SMO.Input.Valpha;
    s->vbeta     = SMO.Input.Vbeta;
    s->vdc       = Power;
    s->imag      = cur_mag_filt;
    s->vcmd      = vf_v_out;
    s->flags     = (uint8_t)((vf_running != 0u ? 1u : 0u) |
                             (vf_boost_lo_on != 0u ? 2u : 0u) |
                             (vf_stall != 0u ? 4u : 0u) |
                             (vf_aligned != 0u ? 8u : 0u));
    smo_head     = next;
}

/* 主循环: 每次尽量把缓冲排空(受 USB 端点忙限制) */
static void App_SmoStreamDrain(void)
{
    uint8_t budget = 4u;

    while (budget-- > 0u && smo_tail != smo_head)
    {
        volatile Smo_Sample_t *s = &smo_buf[smo_tail];

        if (CDC_Printf("%u,%.4f,%.4f,%.4f,%.4f,%.2f,%.1f,%.1f,%.3f,%.3f,%.3f,%.3f,%.2f,%.2f,%.2f,%u\r\n",
                       (unsigned)s->t_ms, (double)s->theta_obs, (double)s->theta_enc,
                       (double)s->err, (double)s->emf, (double)s->omega_obs,
                       (double)s->rpm_obs, (double)s->rpm_enc,
                       (double)s->ialpha, (double)s->ibeta,
                       (double)s->valpha, (double)s->vbeta, (double)s->vdc,
                       (double)s->imag, (double)s->vcmd, (unsigned)s->flags) < 0)
        {
            break;      /* USB 忙, 下一轮再来 */
        }
        smo_tail = (uint8_t)((smo_tail + 1u) & SMO_STREAM_BUF_MASK);
    }
}

static void App_PrintStreamHeader(void)
{
    (void)CDC_Printf("# fs=%uHz drv=%s enc=%u vf=%u rpm_cmd=%.0f |E|_min=%.2f "
                     "k=%.0f delta=%.1f slope=%.3f vmax=%.2f fstart=%.2f valign=%.2f off=%.3f\r\n",
                     (unsigned)APP_CTRL_HZ,
                     (VF_GetDriveMode() == VF_DRV_ENCODER) ? "encoder" : "openloop",
                     (unsigned)Encoder_IsValid(), (unsigned)VF_IsEnabled(),
                     (double)vf_speed_rpm, (double)OBS_EMF_MIN_V,
                     (double)SMO_GetGain(), (double)SMO_GetBoundary(),
                     (double)VF_GetConfig()->v_slope,
                     (double)VF_GetConfig()->v_max,
                     (double)VF_GetConfig()->f_start,
                     (double)VF_GetConfig()->v_align,
                     (double)obs_angle_off);
    (void)CDC_Printf("#t_ms,theta_obs,theta_enc,err,emf,omega_obs,rpm_obs,rpm_enc,"
                     "ialpha,ibeta,valpha,vbeta,vdc,imag,vcmd,flags\r\n");
    if (VF_GetDriveMode() != VF_DRV_ENCODER)
    {
        (void)CDC_Printf("# 开环模式(无编码器): theta_enc / err / rpm_enc 三列无意义, "
                         "只对 theta_obs / emf / rpm_obs 与 rpm_cmd 感兴趣\r\n");
    }
}

/* 每 500ms 一行摘要: 最近这段 |err| 的均值/标准差, 用来判断"是否真在跟踪" */
static void App_PrintSmoStat(void)
{
    static float    sum_err;
    static float    sum_sq;
    static float    sum_emf;
    static uint32_t n;
    uint32_t now = HAL_GetTick();

    if (smo_tail != smo_head)
    {
        uint8_t last = (uint8_t)((smo_head + SMO_STREAM_BUF_LEN - 1u) & SMO_STREAM_BUF_MASK);
        float e = smo_buf[last].err;

        sum_err += e;
        sum_sq  += e * e;
        sum_emf += smo_buf[last].emf;
        n++;
    }

    if (now - smo_stat_ms >= 500u && n > 0u)
    {
        float mean = sum_err / (float)n;
        float var  = sum_sq / (float)n - mean * mean;

        smo_stat_ms = now;
        (void)CDC_Printf("# smo n=%u err_mean=%+.3f err_std=%.3f |E|=%.2f "
                         "rpm_obs=%.0f rpm_enc=%.0f drop=%u\r\n",
                         (unsigned)n, (double)mean,
                         (double)((var > 0.0f) ? sqrtf(var) : 0.0f),
                         (double)(sum_emf / (float)n),
                         (double)SMO.Output.Rpm, (double)vf_speed_meas_rpm,
                         (unsigned)smo_dropped);
        sum_err = 0.0f;
        sum_sq  = 0.0f;
        sum_emf = 0.0f;
        n       = 0u;
    }
}

static float App_WrapAngle(float angle)
{
    angle = fmodf(angle, APP_TWO_PI);
    if (angle > APP_PI) angle -= APP_TWO_PI;
    if (angle < -APP_PI) angle += APP_TWO_PI;
    return angle;
}

/* 电流滤波: 原来每拍做"5 点中值 + 低通", 中值那 4 趟插入排序在
 *   40kHz/20kHz 中断里是纯浪费 —— 它只是为了显示/CAN 上报好看。
 *   改成一阶低通(两次乘加), 效果等价, 中断耗时大降。
 *   若确实需要中值(强脉冲干扰场合), 把 APP_CUR_FILTER_MEDIAN 改 1。 */
#define APP_CUR_FILTER_MEDIAN 0

static float App_FilterCurrent(CurrentFilter_t *filter, float input)
{
#if (APP_CUR_FILTER_MEDIAN != 0)
    float sorted[APP_CURRENT_FILTER_WINDOW];
    float median;

    filter->sample[filter->index] = input;
    filter->index = (uint8_t)((filter->index + 1u) % APP_CURRENT_FILTER_WINDOW);
    for (uint8_t i = 0u; i < APP_CURRENT_FILTER_WINDOW; i++)
    {
        sorted[i] = filter->sample[i];
    }
    for (uint8_t i = 1u; i < APP_CURRENT_FILTER_WINDOW; i++)
    {
        float key = sorted[i];
        uint8_t j = i;
        while (j > 0u && sorted[j - 1u] > key)
        {
            sorted[j] = sorted[j - 1u];
            j--;
        }
        sorted[j] = key;
    }
    median = sorted[APP_CURRENT_FILTER_WINDOW / 2u];
    filter->low_pass += APP_CURRENT_LPF_ALPHA * (median - filter->low_pass);
#else
    (void)filter->sample;
    (void)filter->index;
    filter->low_pass += APP_CURRENT_LPF_ALPHA * (input - filter->low_pass);
#endif
    return filter->low_pass;
}

static int16_t App_SaturateI16(float value)
{
    if (value >= 32767.0f) return 32767;
    if (value <= -32768.0f) return -32768;
    return (int16_t)((value >= 0.0f) ? (value + 0.5f) : (value - 0.5f));
}

static void App_PutI16(uint8_t *dst, int16_t value)
{
    uint16_t raw = (uint16_t)value;
    dst[0] = (uint8_t)(raw >> 8);
    dst[1] = (uint8_t)raw;
}

static uint8_t App_GetErrorCode(void)
{
    if (Power < APP_VDC_MIN) return 0x01u;
    if (Power >= 27.0f) return 0x02u;
    /* 开环测试时驱动角也来自编码器, 编码器失效仍需报错 */
    if (Encoder_IsValid() == 0u ||
        (HAL_GetTick() - encoder_last_valid_ms) > 20u) return 0x03u;
    if (Temp > 60.0f) return 0x04u;
    return 0x00u;
}

static void App_SendCanFeedback(void)
{
    uint8_t data[8] = {0u};
    float current = fminf(fmaxf(iq_filt, -30.0f), 30.0f);
    float speed = fminf(fmaxf(spd_meas_rpm,
                              -APP_CAN_SPEED_LIMIT_RPM),
                        APP_CAN_SPEED_LIMIT_RPM);
    float position = App_WrapAngle(Encoder_Mech_Position);
    float temperature = fminf(fmaxf(Temp, 0.0f), 255.0f);

    App_PutI16(&data[0], App_SaturateI16(current * APP_CAN_IQ_SCALE));
    App_PutI16(&data[2], App_SaturateI16(speed));
    App_PutI16(&data[4], App_SaturateI16(position *
                                         APP_CAN_POSITION_TO_RAW));
    data[6] = (uint8_t)(temperature + 0.5f);
    data[7] = App_GetErrorCode();
    (void)BSP_CAN_SendStd(0x78u, data, 8u);
}

/* 开环运行时的实时诊断: 每 500ms 一行, 不开打点流也能看清驱动在干什么。
 *   这一行是排查"不转"的第一手材料: v_out 有没有给出去、I 有没有上来、|E| 有没有建立。 */
static void App_PrintOpenLoopLive(void)
{
    static uint32_t next_ms;
    static uint32_t cnt;
    uint32_t now = HAL_GetTick();

    if (VF_IsEnabled() == 0u)
    {
        if (cnt != 0u)
        {
            (void)CDC_Printf("# openloop OFF (跑了 %u 行诊断)\r\n", (unsigned)cnt);
            cnt = 0u;
        }
        next_ms = 0u;
        return;
    }
    if (now < next_ms) return;
    next_ms = now + 500u;
    cnt++;

    /* 诊断块: 一行把关键状态全带上(合并成一次 CDC_Printf, 降低出错面)。
     *   ★格式串与实参一一对应, 改这里务必跑 python tools/audit_printf.py */
    (void)CDC_Printf("# DIAG t=%u over=%u TIM1[CR1=%04X CCER=%04X ARR=%u C1=%u C2=%u C3=%u C4=%u] "
                     "PARK[%.1fV/%.0fdeg] EN=%u vdc=%.2f I=%.2fA vout=%.2f ia=%.2f ib=%.2f ic=%.2f "
                     "th_drv=%+.3f vf=%u align=%u\r\n",
                     (unsigned)now, (unsigned)isr_over_cnt,
                     (unsigned)TIM1->CR1, (unsigned)TIM1->CCER, (unsigned)TIM1->ARR,
                     (unsigned)TIM1->CCR1, (unsigned)TIM1->CCR2, (unsigned)TIM1->CCR3,
                     (unsigned)TIM1->CCR4,
                     (double)VF_GetParkV(), (double)VF_GetParkDeg(),
                     (unsigned)((GPIOB->ODR >> 6) & 1u),
                     (double)Power, (double)cur_mag_filt, (double)vf_v_out,
                     (double)Current[0], (double)Current[1], (double)Current[2],
                     (double)vf_theta_cmd, (unsigned)VF_IsEnabled(),
                     (unsigned)vf_aligned);

    /* 点动模式单独一行, 免得和 V/F 状态混淆 */
    if (VF_GetParkV() > 0.05f)
    {
        (void)CDC_Printf("# PARK deg=%.1f V=%.2f I=%.1fA vdc=%.1f (固定角恒压: "
                         "有力=功率级OK; park 0 0 退出)\r\n",
                         (double)VF_GetParkDeg(), (double)VF_GetParkV(),
                         (double)cur_mag_filt, (double)Power);
        return;
    }

#if (APP_SMO_ENABLE != 0)
    (void)CDC_Printf("# OL t=%u v_out=%.2fV I=%.1fA |E|=%.2fV rpm_cmd=%.0f rpm_obs=%.0f "
                     "vdc=%.1f %s%s%s | vlo=%.2f fres=%.1f fstart=%.1f prot=%u\r\n",
                     (unsigned)now, (double)vf_v_out, (double)cur_mag_filt,
                     (double)SMO.Output.Emf, (double)vf_speed_rpm,
                     (double)SMO.Output.Rpm, (double)Power,
                     (vf_aligned != 0u) ? "aligned " : "aligning ",
                     (vf_running != 0u) ? "running " : "no-emf ",
                     (vf_boost_lo_on != 0u) ? "boost_lo" : "curve",
                     (double)VF_GetConfig()->v_boost_lo,
                     (double)VF_GetConfig()->f_res,
                     (double)VF_GetConfig()->f_start,
                     (unsigned)VF_GetStallEnable());
#else
    (void)CDC_Printf("# OL t=%u v_out=%.2fV I=%.1fA rpm_cmd=%.0f vdc=%.1f | %s%s | "
                     "vlo=%.2f fres=%.1f (SMO已关)\r\n",
                     (unsigned)now, (double)vf_v_out, (double)cur_mag_filt,
                     (double)vf_speed_rpm, (double)Power,
                     (vf_aligned != 0u) ? "aligned " : "aligning ",
                     (vf_boost_lo_on != 0u) ? "boost_lo" : "curve",
                     (double)VF_GetConfig()->v_boost_lo,
                     (double)VF_GetConfig()->f_res);
#endif
    /* ★开环自检行: 频率口径对不对, 看这一行就够了
     *   fe_cmd = 指令电频率(代码算的)
     *   fe_act = 编码器实测角反推的电频率
     *   ratio  = fe_act/fe_cmd, 正常应当≈1.00(电机确实跟着转的前提下)
     *   若 ratio 差 2 倍/10 倍 -> vf_dt 标定错了(它是开环唯一的"时间基准") */
    (void)CDC_Printf("# FREQ vf_dt=%.3fus fe_cmd=%.2fHz fe_act=%.2fHz ratio=%.3f "
                     "rpm_cmd=%.0f enc_vld=%u\r\n",
                     (double)(VF_GetDbgDt() * 1.0e6),
                     (double)VF_GetDbgFeCmd(), (double)VF_GetDbgFeAct(),
                     (VF_GetDbgFeCmd() > 0.1f)
                         ? (double)(VF_GetDbgFeAct() / VF_GetDbgFeCmd()) : 0.0,
                     (double)VF_GetTargetRPM(), (unsigned)Encoder_IsValid());
}

/* --------------------------------------------------------------------------
 * 中断内分段计时(诊断用)
 *   40kHz 预算 25us, 必须知道每一段占多少才能真正优化, 不能靠猜。
 *   开机后统计 2 秒各段最大耗时, 打印一次。
 * ------------------------------------------------------------------------ */
static uint32_t prof_smo;
static uint32_t prof_vf;
static uint8_t  prof_on = 1u;
static uint32_t prof_ticks;

#define PROF_CYC() (DWT->CYCCNT)
#define PROF_CYC_US(c) ((float)(c) / (float)(SystemCoreClock / 1000000u))

/* --------------------------------------------------------------------------
 * 观测器抽取运行(降低中断耗时)
 *
 *   ★2026-10: 控制频率已改回 20kHz(预算 50us, 实测 33.16us, 余量 17us),
 *     **不需要抽取了**, 所以这里改成 1 —— 观测器每拍(20kHz)运行。
 *     抽取本来是 40kHz/25us 预算下的权宜之计, 而且抽取会引入额外延迟;
 *     每拍运行最简单、最不容易出错。
 *
 *   若哪天又必须抽取(例如把频率提回 40kHz), 记住两件事:
 *     ① 抽取后必须把观测周期告诉观测器(App_FOC_Init 里的 SMO_SetTs), 否则 F/G 全错;
 *     ② 边界层 δ 必须随 Ts 等比放大(SMO.h 里有完整推导), 否则观测器发散。
 *   抽取周期下"每拍电角度"别接近 180°(否则角度差折返、速度估计反号):
 *     20kHz 抽取 8 = 2.5kHz 时, 6000rpm 才 101°, 还安全。
 * ------------------------------------------------------------------------ */
#define SMO_DECIM 1u

#if (APP_SMO_ENABLE != 0)
static uint8_t  smo_dec_cnt;
static float    smo_ia_acc;
static float    smo_ib_acc;

static void App_SmoStep(void)
{
    /* 累积本抽取窗口的电流(抽取=1 时就是本拍值, 累积无副作用) */
    smo_ia_acc += ialpha;
    smo_ib_acc += ibeta;
    smo_dec_cnt++;

    if (smo_dec_cnt < SMO_DECIM) return;
    {
        float inv = 1.0f / (float)SMO_DECIM;
        SMO.Input.Ialpha = smo_ia_acc * inv;
        SMO.Input.Ibeat  = smo_ib_acc * inv;
    }
    smo_dec_cnt = 0u;
    smo_ia_acc  = 0.0f;
    smo_ib_acc  = 0.0f;
    SMO.Output.ThetaEnc = Encoder_Elec_Angle;
    SMO_Run(&SMO);
}
#endif  /* APP_SMO_ENABLE */

/* --------------------------------------------------------------------------
 * 开环 V/F 通路开关(0 = 回闭环电流环)
 * ------------------------------------------------------------------------ */
static void App_OpenLoopSet(uint8_t on)
{
    if (on != 0u)
    {
        VF_SetEnable(1u);
        SMO_Reset();                 /* 观测器从头拉入, 避免带旧状态 */
        Control_ResetCurrentPID();
        Control_ResetSpeedPID();
        Control_ResetPositionPID();
        cur_id_ref = 0.0f;
        cur_iq_ref = 0.0f;
        cur_vd = 0.0f;
        cur_vq = 0.0f;
        speed_ref_cmd_rpm = 0.0f;
        spd_err_rpm = 0.0f;
        pos_err_rad = 0.0f;
        vdc_fault = 0u;              /* 开环测试期间不因欠压反复封驱动 */
        vdc_low_count = 0u;
        (void)CDC_Printf("# open-loop ON\r\n");
    }
    else
    {
        VF_SetEnable(0u);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0u);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0u);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0u);
        SMO_Reset();
        (void)CDC_Printf("# open-loop OFF, driver=0\r\n");
    }
}

static void App_SmoStreamSet(uint8_t on)
{
    smo_head = 0u;
    smo_tail = 0u;
    smo_dropped = 0u;
    smo_last_us = App_NowUs();
    smo_stream_on = (on != 0u) ? 1u : 0u;
    if (smo_stream_on != 0u)
    {
        App_PrintStreamHeader();
    }
    else
    {
        (void)CDC_Printf("# stream off (drop=%u)\r\n", (unsigned)smo_dropped);
    }
}

/* --------------------------------------------------------------------------
 * 命令口
 *   CAN(0x91, data[0] = 子命令, data[1..2] = int16 大端):
 *     0x01 电流 iq(0.001A) / 0x02 转速 rpm / 0x03 位置
 *     0x20 开环 V/F 开关(非0=开)      ★开环 = 驱动角自己积分, 不需要编码器
 *     0x21 目标转速(电机侧 rpm)
 *     0x22 V/F 斜率 V/Hz = 值×0.001
 *     0x23 V/F 电压上限 V = 值×0.001
 *     0x24 编码器模式下的转矩角 = 值×0.0001 rad
 *     0x25 对照偏置 = 值×0.0001 rad
 *     0x26 斜坡 rpm/s
 *     0x27 观测器滞后补偿带宽 rad/s
 *     0x28 驱动角来源(0=开环积分角 1=编码器角)
 *     0x29 起步对齐电压 V = 值×0.001
 *     0x30 SMO 打点流开关(非0=开)
 *     0x40 打印状态(TIM1 自检 + 观测器 + V/F + 方向自检)
 * ------------------------------------------------------------------------ */
void BSP_CAN_RxCallback(const CAN_RxHeaderTypeDef *header, const uint8_t *data)
{
    int16_t command;

    if (header == 0 || data == 0 || header->IDE != CAN_ID_STD ||
        header->StdId != 0x91u || header->DLC < 3u)
    {
        return;
    }

    command = (int16_t)(((uint16_t)data[1] << 8) | data[2]);
    switch (data[0])
    {
        case 0x01u:
            if (VF_IsEnabled()) App_OpenLoopSet(0u);
            Control_SetCurrentTarget(0.0f, (float)command * 0.001f);
            break;
        case 0x02u:
            if (VF_IsEnabled()) App_OpenLoopSet(0u);
            Control_SetSpeedTarget((float)command);
            break;
        case 0x03u:
            if (VF_IsEnabled()) App_OpenLoopSet(0u);
            Control_SetPositionTarget((float)command * APP_CAN_RAW_TO_POSITION);
            break;
        case 0x20u:  /* 开环 V/F 开关 */
            App_OpenLoopSet((command != 0) ? 1u : 0u);
            break;
        case 0x21u:  /* 目标转速(电机侧 rpm) */
            VF_SetTargetRPM((float)command);
            break;
        case 0x22u:  /* 斜率 V/Hz ×1000 */
            VF_SetVSlope((float)command * 0.001f);
            break;
        case 0x23u:  /* 电压上限 V ×1000 */
            VF_SetVMax((float)command * 0.001f);
            break;
        case 0x24u:  /* 转矩角: 值×0.0001 rad(π=31416, int16 放得下) */
            VF_SetTorqueAngle((float)command * 0.0001f);
            break;
        case 0x25u:  /* 对照偏置: 值×0.0001 rad */
            obs_angle_off = (float)command * 0.0001f;
            break;
        case 0x26u:  /* 斜坡 rpm/s */
            VF_SetRamp((float)command);
            break;
        case 0x27u:  /* 滞后补偿等效带宽 rad/s */
            SMO_SetLagW((float)command);
            break;
        case 0x28u:  /* 驱动角来源: 0=开环积分角(无编码器) 1=编码器角 */
            VF_SetDriveMode((command != 0) ? VF_DRV_ENCODER : VF_DRV_OPENLOOP);
            break;
        case 0x29u:  /* 起步对齐电压 V ×1000 */
            VF_SetVAlign((float)command * 0.001f);
            break;
        case 0x2Au:  /* 低频强制启动电压 V ×1000(解决起步堵转) */
            VF_SetVBoostLo((float)command * 0.001f);
            break;
        case 0x2Bu:  /* 低频强制段上界电频率 Hz ×100 */
            VF_SetFRes((float)command * 0.01f);
            break;
        case 0x2Cu:  /* 失步保护开关: 0=关(台架调试用) */
            VF_SetStallEnable((command != 0) ? 1u : 0u);
            break;
        case 0x30u:  /* 打点流开关 */
            App_SmoStreamSet((command != 0) ? 1u : 0u);
            break;
        case 0x40u:  /* 打印状态 */
            App_PrintStatus();
            break;
        default:
            break;
    }
}

/* --------------------------------------------------------------------------
 * USB 文本命令: 串口里直接敲, 不用 CAN 盒
 *   vf <rpm>        进开环, 目标转速(电机侧 rpm), 例: vf 1500
 *   vf              只进开环, 不改目标
 *   stop            退出开环, 驱动封零
 *   cur <A>         电流模式(闭环)
 *   spd <rpm>       速度模式(闭环)
 *   slope <V/Hz> / vmax <V> / vboost <V> / ramp <rpm/s> / angle <rad> / off <rad>
 *   gain <V> / delta <A>        滑模增益 / 边界层
 *   stream <0|1>    打点流开关
 *   stat            打印状态
 *   help            帮助
 * ------------------------------------------------------------------------ */
static void App_UsbCommand(const char *line)
{
    char cmd[16] = {0};
    float value = 0.0f;
    int n = sscanf(line, "%15s %f", cmd, &value);

    if (n <= 0) return;

    if (strcmp(cmd, "vf") == 0)
    {
        if (n >= 2) VF_SetTargetRPM(value);
        App_OpenLoopSet(1u);
        App_PrintStatus();
    }
    else if (strcmp(cmd, "stop") == 0)   { App_OpenLoopSet(0u); }
    else if (strcmp(cmd, "cur") == 0)
    {
        if (VF_IsEnabled()) App_OpenLoopSet(0u);
        Control_SetCurrentTarget(0.0f, value);
        (void)CDC_Printf("# current mode iq=%.2f A\r\n", (double)value);
    }
    else if (strcmp(cmd, "spd") == 0)
    {
        if (VF_IsEnabled()) App_OpenLoopSet(0u);
        Control_SetSpeedTarget(value);
        (void)CDC_Printf("# speed mode rpm=%.0f\r\n", (double)value);
    }
    else if (strcmp(cmd, "slope") == 0)   { VF_SetVSlope(value);      App_PrintStatus(); }
    else if (strcmp(cmd, "vmax") == 0)    { VF_SetVMax(value);        App_PrintStatus(); }
    else if (strcmp(cmd, "vboost") == 0)  { VF_SetVBoost(value);      App_PrintStatus(); }
    else if (strcmp(cmd, "ramp") == 0)    { VF_SetRamp(value);        App_PrintStatus(); }
    else if (strcmp(cmd, "angle") == 0)   { VF_SetTorqueAngle(value); App_PrintStatus(); }
    else if (strcmp(cmd, "off") == 0)     { obs_angle_off = value;    App_PrintStatus(); }
    else if (strcmp(cmd, "lag") == 0)     { SMO_SetLagW(value);       App_PrintStatus(); }
    else if (strcmp(cmd, "drv") == 0)     { VF_SetDriveMode((value != 0.0f) ? VF_DRV_ENCODER : VF_DRV_OPENLOOP); App_PrintStatus(); }
    else if (strcmp(cmd, "fstart") == 0)  { VF_SetFStart(value);      App_PrintStatus(); }
    else if (strcmp(cmd, "valign") == 0)  { VF_SetVAlign(value);      App_PrintStatus(); }
    else if (strcmp(cmd, "vlo") == 0)     { VF_SetVBoostLo(value);    App_PrintStatus(); }
    else if (strcmp(cmd, "fres") == 0)    { VF_SetFRes(value);        App_PrintStatus(); }
    else if (strcmp(cmd, "prot") == 0)    { VF_SetStallEnable((value != 0.0f) ? 1u : 0u); App_PrintStatus(); }
    else if (strcmp(cmd, "park") == 0)
    {
        /* park <角度deg> <电压V>: 固定角加恒压, 转子会吸在该角度上。
         *   用来验证驱动/接线: 有力=功率级正常; 没感觉=驱动或接线有问题。
         *   park 0 0 退出。★会持续通电发热, 用完记得退! */
        char cmd2[16] = {0};
        float volts = 0.0f;
        int n2 = sscanf(line, "%15s %f %f", cmd2, &value, &volts);
        if (n2 >= 3)
        {
            VF_SetPark(value, volts);
            (void)CDC_Printf("# park: angle=%.1fdeg V=%.2fV (转子会被吸到该角度; park 0 0 退出)\r\n",
                             (double)value, (double)volts);
        }
        else
        {
            VF_SetPark(0.0f, 0.0f);
            (void)CDC_Printf("# park OFF\r\n");
        }
    }
    else if (strcmp(cmd, "gain") == 0)    { SMO_SetGain(value);       App_PrintStatus(); }
    else if (strcmp(cmd, "delta") == 0)   { SMO_SetBoundary(value);   App_PrintStatus(); }
    else if (strcmp(cmd, "stream") == 0)  { App_SmoStreamSet((value != 0.0f) ? 1u : 0u); }
    else if (strcmp(cmd, "stat") == 0)    { App_PrintStatus(); }
    else if (strcmp(cmd, "help") == 0)
    {
        (void)CDC_Printf("# vf <rpm> | stop | cur <A> | spd <rpm> | "
                         "slope <V/Hz> | vmax <V> | vboost <V> | vlo <V> | fres <Hz> | "
                         "ramp <rpm/s> | fstart <Hz> | valign <V> | drv <0|1> | prot <0|1> | "
                         "park <deg> <V> | angle <rad> | "
                         "off <rad> | lag <rad/s> | gain <V> | delta <A> | stream <0|1> | stat | help\r\n");
    }
    else
    {
        (void)CDC_Printf("# unknown cmd '%s', try help\r\n", cmd);
    }
}

/* CDC 收到的字节累积成行, 1ms 轮询一次解析 */
static char usb_line[64];
static uint8_t usb_line_len;

static void App_UsbPoll(void)
{
    uint8_t chunk[32];
    uint32_t len = CDC_RxTake(chunk, (uint16_t)sizeof(chunk));

    for (uint32_t i = 0u; i < len; i++)
    {
        char c = (char)chunk[i];

        if (c == '\n' || c == '\r')
        {
            usb_line[usb_line_len] = '\0';
            if (usb_line_len > 0u) App_UsbCommand(usb_line);
            usb_line_len = 0u;
        }
        else if (usb_line_len < (uint8_t)(sizeof(usb_line) - 1u))
        {
            usb_line[usb_line_len++] = c;
        }
        else
        {
            usb_line_len = 0u;   /* 一行太长, 丢弃 */
        }
    }
}

static void App_UpdateEncoder(void)
{
    Encoder_Update(Encoder_Sin, Encoder_Cos);
    Encoder_Angle = Encoder_GetMechAngle();
    Encoder_Elec_Angle = Encoder_GetElecAngle();
    Encoder_Mech_Position = Encoder_GetMechanicalPosition();
    if (Encoder_IsValid() != 0u)
    {
        encoder_last_valid_ms = HAL_GetTick();
    }
}

static void App_UpdateSpeed(void)
{
    if (cur_mech_valid == 0u)
    {
        cur_mech_prev = Encoder_Mech_Position;
        cur_mech_valid = 1u;
        return;
    }

    cur_spd_accum += Encoder_Mech_Position - cur_mech_prev;
    cur_mech_prev = Encoder_Mech_Position;
    if (++cur_spd_count >= APP_SPEED_WINDOW_TICKS)
    {
        spd_meas_rpm += APP_SPEED_LPF_ALPHA *
                        (cur_spd_accum * APP_SPEED_RPM_SCALE - spd_meas_rpm);
        cur_spd_accum = 0.0f;
        cur_spd_count = 0u;
    }
}

static void App_UpdateProtection(float vdc)
{
    if (vdc_fault == 0u)
    {
        if (vdc < APP_VDC_MIN)
        {
            if (++vdc_low_count >= APP_VDC_FAULT_TICKS)
            {
                vdc_fault = 1u;
                vdc_low_count = 0u;
                vdc_ok_count = 0u;
                Control_ResetCurrentPID();
                Control_ResetSpeedPID();
                Control_ResetPositionPID();
                cur_iq_ref = 0.0f;
                speed_ref_cmd_rpm = 0.0f;
                spd_err_rpm = 0.0f;
                spd_meas_rpm = 0.0f;
                cur_spd_accum = 0.0f;
                cur_spd_count = 0u;
                cur_id_err = 0.0f;
                cur_iq_err = 0.0f;
                __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0u);
                __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0u);
                __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0u);
                HAL_GPIO_WritePin(EN_Gate_GPIO_Port, EN_Gate_Pin, GPIO_PIN_RESET);
            }
        }
        else
        {
            vdc_low_count = 0u;
        }
    }
    else
    {
        if (vdc > APP_VDC_RECOVER)
        {
            if (++vdc_ok_count >= APP_VDC_RECOVER_TICKS)
            {
                vdc_fault = 0u;
                vdc_ok_count = 0u;
                Control_ResetCurrentPID();
                Control_ResetSpeedPID();
                Control_ResetPositionPID();
                HAL_GPIO_WritePin(EN_Gate_GPIO_Port, EN_Gate_Pin, GPIO_PIN_SET);
            }
        }
        else
        {
            vdc_ok_count = 0u;
        }
    }
}

static uint8_t App_ProtectionActive(void)
{
    if (vdc_fault == 0u) return 0u;
    Control_ResetSpeedPID();
    Control_ResetPositionPID();
    cur_iq_ref = 0.0f;
    speed_ref_cmd_rpm = 0.0f;
    spd_err_rpm = 0.0f;
    spd_meas_rpm = 0.0f;
    cur_spd_accum = 0.0f;
    cur_spd_count = 0u;
    cur_id_err = 0.0f;
    cur_iq_err = 0.0f;
    pos_err_rad = 0.0f;
    cur_vd = 0.0f;
    cur_vq = 0.0f;
    return 1u;
}

static void App_CurrentLoopTick(float vdc, float theta_elec)
{
    Control_Mode_t mode;

    App_UpdateSpeed();
    App_UpdateProtection(vdc);
    if (App_ProtectionActive() != 0u)
    {
        return;
    }

    mode = Control_GetMode();
    if (mode != previous_mode)
    {
        Control_ResetSpeedPID();
        Control_ResetPositionPID();
        speed_loop_count = 0u;
        cur_iq_ref = 0.0f;
        cur_id_ref = 0.0f;
        speed_ref_cmd_rpm = 0.0f;
        pos_err_rad = 0.0f;
        previous_mode = mode;
    }

    if (mode == CONTROL_MODE_CURRENT)
    {
        Control_GetCurrentTarget(&cur_id_ref, &cur_iq_ref);
        speed_ref_cmd_rpm = 0.0f;
        spd_err_rpm = 0.0f;
        pos_err_rad = 0.0f;
    }
    else if (++speed_loop_count >= Control_GetSpeedLoop())
    {
        speed_loop_count = 0u;
        if (mode == CONTROL_MODE_SPEED)
        {
            speed_ref_cmd_rpm = Control_GetSpeedTarget();
            spd_err_rpm = speed_ref_cmd_rpm - spd_meas_rpm;
            cur_iq_ref = Control_SpeedStep(spd_err_rpm);
            cur_id_ref = 0.0f;
        }
        else if (mode == CONTROL_MODE_POSITION)
        {
            float position = App_WrapAngle(Encoder_Mech_Position);
            pos_err_rad = Control_GetPositionError(position);
            speed_ref_cmd_rpm = Control_PositionStep(position);
            spd_err_rpm = speed_ref_cmd_rpm - spd_meas_rpm;
            cur_iq_ref = Control_SpeedStep(spd_err_rpm);
            cur_id_ref = 0.0f;
        }
    }

    cur_id_err = cur_id_ref - id_filt;
    cur_iq_err = cur_iq_ref - iq_filt;
    cur_vd = Control_CurrentDStep(cur_id_err);
    cur_vq = Control_CurrentQStep(cur_iq_err);
    {
        float valpha;
        float vbeta;
        uint16_t ccr1;
        uint16_t ccr2;
        uint16_t ccr3;
        DeParkTransformer(&valpha, &vbeta, theta_elec, cur_vq, cur_vd);
        SMO.Input.Valpha = valpha;
        SMO.Input.Vbeta = vbeta;
        SVPWM(valpha, vbeta, vdc, &ccr1, &ccr2, &ccr3);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, ccr1);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, ccr2);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, ccr3);
    }
}

/* 三相电流同源后执行的统一处理 */
static void App_AdcStep(void)
{
    uint32_t cyc_isr0 = PROF_CYC();     /* 本拍起点, 用于统计超预算次数 */
    uint32_t now_us = App_NowUs();

    /* 原始值 -> 工程量(中断只搬原始值, 换算是应用层的事) */
    ADC_Voltage[0] = BSP_AdcRaw_PhaseV[0];
    ADC_Voltage[1] = BSP_AdcRaw_PhaseV[1];
    ADC_Voltage[2] = BSP_AdcRaw_PhaseV[2];
    Encoder_Sin = (int)BSP_AdcRaw_EncSin;
    Encoder_Cos = (int)BSP_AdcRaw_EncCos;
    ADC_Current[0] = BSP_AdcRaw_Current[0];
    ADC_Current[1] = BSP_AdcRaw_Current[1];
    ADC_Current[2] = BSP_AdcRaw_Current[2];
    ADC_Power = BSP_AdcRaw_Power;
    ADC_Temp  = BSP_AdcRaw_Temp;
    prof_ticks_total++;
    {   /* 每拍量一次中断本体耗时: 超过预算就是丢拍的直接证据 */
        uint32_t cyc_now = PROF_CYC() - cyc_isr0;
        if (cyc_now > (SystemCoreClock / APP_CTRL_HZ)) isr_over_cnt++;
    }
    Voltage[0] = (float)ADC_Voltage[0] * APP_ADC_VOLT_SCALE * APP_PHASE_VOLT_DIVIDER;
    Voltage[1] = (float)ADC_Voltage[1] * APP_ADC_VOLT_SCALE * APP_PHASE_VOLT_DIVIDER;
    Voltage[2] = (float)ADC_Voltage[2] * APP_ADC_VOLT_SCALE * APP_PHASE_VOLT_DIVIDER;
    Temp = (float)ADC_Temp * APP_ADC_VOLT_SCALE;
    if (adc_cal_done == 0u)
    {
        cal_sum[0] += (uint32_t)ADC_Current[0]; cal_count[0]++;
        cal_sum[1] += (uint32_t)ADC_Current[1]; cal_count[1]++;
        cal_sum[2] += (uint32_t)ADC_Current[2]; cal_count[2]++;
    }

    if (adc_cal_done == 0u &&
        cal_count[0] >= APP_ADC_CAL_WINDOW &&
        cal_count[1] >= APP_ADC_CAL_WINDOW &&
        cal_count[2] >= APP_ADC_CAL_WINDOW)
    {
        csa_vmid[0] = (float)cal_sum[0] / (float)cal_count[0];
        csa_vmid[1] = (float)cal_sum[1] / (float)cal_count[1];
        csa_vmid[2] = (float)cal_sum[2] / (float)cal_count[2];
        adc_cal_done = 1u;
    }

    Current[0] = ((float)ADC_Current[0] - csa_vmid[0]) * APP_CURRENT_SCALE;
    Current[1] = ((float)ADC_Current[1] - csa_vmid[1]) * APP_CURRENT_SCALE;
    Current[2] = ((float)ADC_Current[2] - csa_vmid[2]) * APP_CURRENT_SCALE;
    ClarkTransformer(Current[0], Current[1], Current[2], &ialpha, &ibeta);
    ia_filt = App_FilterCurrent(&ia_filter, ialpha);
    ib_filt = App_FilterCurrent(&ib_filter, ibeta);
    cur_mag = sqrtf(ia_filt * ia_filt + ib_filt * ib_filt);
    cur_mag_filt += APP_CURRENT_MAG_ALPHA * (cur_mag - cur_mag_filt);
    Power = (float)ADC_Power * APP_ADC_VOLT_SCALE * APP_VDC_DIVIDER;
    App_UpdateEncoder();
    ParkTransformer(ialpha, ibeta, Encoder_Elec_Angle, &iq, &id);
    id_filt = App_FilterCurrent(&id_filter, id);
    iq_filt = App_FilterCurrent(&iq_filter, iq);

    /* ---- 观测器: 抽取运行(见 App_SmoStep) ----
     *   ★APP_SMO_ENABLE=0 时整段跳过: 开环驱动角是积分角, 本来就不需要观测器,
     *     先去掉它把出问题的面缩到最小(失步保护也随之失效, 注意别长时间堵转)。 */
#if (APP_SMO_ENABLE != 0)
    {
        uint32_t c0 = PROF_CYC();
        App_SmoStep();
        if (prof_on != 0u)
        {
            uint32_t d = PROF_CYC() - c0;
            if (d > prof_smo) prof_smo = d;
        }
    }
#endif

    /* ---- 控制通路: 开环 V/F(自己积分驱动角, 不需要编码器) 或闭环电流环 ---- */
    if (VF_IsEnabled() != 0u)
    {
        uint32_t c0 = PROF_CYC();

#if (APP_SMO_ENABLE != 0)
        VF_SetObsFeedback(SMO.Output.Emf, SMO.Output.Rpm);
#endif
        VF_Tick(Power, Encoder_Elec_Angle);
        /* 开环自检: 拿编码器实测角反推实际电频率(判断 vf_dt 标定对不对) */
        VF_DebugFreq(Encoder_Elec_Angle, VF_GetDt());
        if (prof_on != 0u)
        {
            uint32_t d = PROF_CYC() - c0;
            if (d > prof_vf) prof_vf = d;
        }
#if (APP_SMO_ENABLE != 0)
        /* 失步/堵转: 电压已加但迟迟没有反电动势 -> 封驱动, 免堵转烧电机。
         *   ★只认"本拍 VF 刚置上的"失步标志, 并且封完就把它清掉 ——
         *     否则标志一直挂着, 后面每次 vf 都会在下一拍被立刻封死(表现为"完全不转")。 */
        if (vf_stall != 0u)
        {
            if (vdc_fault == 0u)
            {
                __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0u);
                __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0u);
                __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0u);
                HAL_GPIO_WritePin(EN_Gate_GPIO_Port, EN_Gate_Pin, GPIO_PIN_RESET);
            }
            VF_SetEnable(0u);
            vf_stall = 0u;
            vdc_fault = 0u;
        }
#endif
    }
    else
    {
        App_CurrentLoopTick(Power, Encoder_Elec_Angle);
    }

    /* 下一拍给观测器用的电压(本拍真正施加的电压) */
    SMO.Input.Valpha = vf_valpha;
    SMO.Input.Vbeta  = vf_vbeta;

    /* 方向自检: 只有在编码器有效时才有意义(无编码器时编码器角是噪声, 会误判) */
    if (SMO.Output.Valid != 0u && Encoder_IsValid() != 0u)
    {
        float d_obs = App_AngleDiff(SMO.Output.Theta, smo_theta_prev_isr);
        float d_enc = App_AngleDiff(Encoder_Elec_Angle, SMO.Output.ThetaEnc);

        if (fabsf(d_enc) > 0.05f)
        {
            if ((d_obs >= 0.0f) == (d_enc >= 0.0f)) smo_dir_ok++;
            else                                     smo_dir_bad++;
        }
    }
    smo_theta_prev_isr = SMO.Output.Theta;

    /* 打点: 1kHz 降采样 */
    App_SmoStreamTick(HAL_GetTick(), now_us);

    /* ★★ 从"中断侧"打印的状态行: 主循环/USB命令通道即使卡死, 这行也会出来。
     *   用来一刀切开三种情况:
     *     rx_total 一直不变  -> 上位机的命令根本没进 USB(线/驱动/接收方向)
     *     rx_total 在涨、set=0 -> 命令进来了但没解析成 vf(命令拼写/结尾符)
     *     set>=1 但 vf=0      -> 命令生效过又被关掉了(看 log 里的开关序列)
     *     vf=1                -> 开环其实已经使能, 问题在 V/F 内部(看 FREQ 行) */
    {
        static uint32_t isr_dbg_ms;
        uint32_t tnow = HAL_GetTick();

        if (tnow - isr_dbg_ms >= 2000u)
        {
            const uint8_t *lg = VF_GetEnLog();
            uint8_t n = VF_GetEnLogN();

            isr_dbg_ms = tnow;
            (void)CDC_Printf("# ISRDBG t=%u vf=%u set=%u clr=%u rx_total=%u rx_drop=%u "
                             "en_log=%u%u%u%u%u%u\r\n",
                             (unsigned)tnow, (unsigned)VF_IsEnabled(),
                             (unsigned)VF_GetEnSetCnt(), (unsigned)VF_GetEnClrCnt(),
                             (unsigned)CDC_RxTotal(), (unsigned)CDC_RxDropped(),
                             (unsigned)(n > 0u ? lg[0] : 9u), (unsigned)(n > 1u ? lg[1] : 9u),
                             (unsigned)(n > 2u ? lg[2] : 9u), (unsigned)(n > 3u ? lg[3] : 9u),
                             (unsigned)(n > 4u ? lg[4] : 9u), (unsigned)(n > 5u ? lg[5] : 9u));
        }
    }


    HAL_GPIO_WritePin(LED_Red_GPIO_Port, LED_Red_Pin,
                      (HAL_GPIO_ReadPin(FAULT_GPIO_Port, FAULT_Pin) == GPIO_PIN_RESET) ?
                      GPIO_PIN_SET : GPIO_PIN_RESET);
    if (HAL_GetTick() - heartbeat_ms >= 500u)
    {
        HAL_GPIO_TogglePin(LED_Green_GPIO_Port, LED_Green_Pin);
        heartbeat_ms = HAL_GetTick();
    }
}

static void App_PrintStatus(void)
{
    (void)CDC_Printf("# mode=%s rpm_cmd=%.0f rpm_meas=%.0f |E|=%.2f th_obs=%+.3f th_enc=%+.3f "
                     "err=%+.3f rpm_obs=%.0f vdc=%.1f iq=%.2f I=%.2f "
                     "slope=%.3f vmax=%.2f vboost=%.2f fstart=%.2f valign=%.2f ramp=%.0f "
                     "k=%.0f delta=%.1f lagw=%.0f off=%+.3f lag=%+.3f stream=%u\r\n",
                     (VF_IsEnabled() == 0u) ? "FOC-CLOSED"
                        : ((VF_GetDriveMode() == VF_DRV_ENCODER) ? "VF-enc" : "VF-open"),
                     (double)VF_GetTargetRPM(), (double)vf_speed_meas_rpm,
                     (double)SMO.Output.Emf, (double)SMO.Output.Theta,
                     (double)SMO.Output.ThetaEnc,
                     (double)App_AngleDiff(App_ObsCompensated(),
                                           SMO.Output.ThetaEnc),
                     (double)SMO.Output.Rpm, (double)Power, (double)iq_filt,
                     (double)cur_mag_filt,
                     (double)VF_GetConfig()->v_slope, (double)VF_GetConfig()->v_max,
                     (double)VF_GetConfig()->v_boost,
                     (double)VF_GetConfig()->f_start,
                     (double)VF_GetConfig()->v_align,
                     (double)VF_GetConfig()->rpm_ramp,
                     (double)SMO_GetGain(), (double)SMO_GetBoundary(),
                     (double)SMO_GetLagW(),
                     (double)obs_angle_off, (double)SMO.Output.Lag,
                     (unsigned)smo_stream_on);
    /* TIM1 自检: CEN 必须为 1, CC1E/2E/3E/4E 必须都置位, 否则 PWM/ADC 触发是死的 */
    (void)CDC_Printf("# tim1 CR1=0x%04X CCER=0x%04X ARR=%u CCR4=%u drv=%u vdc_fault=%u usb_drop=%u\r\n",
                     (unsigned)TIM1->CR1, (unsigned)TIM1->CCER,
                     (unsigned)TIM1->ARR, (unsigned)TIM1->CCR4,
                     (unsigned)((HAL_GPIO_ReadPin(FAULT_GPIO_Port, FAULT_Pin) == GPIO_PIN_RESET) ? 1u : 0u),
                     (unsigned)vdc_fault, (unsigned)CDC_RxDropped());
    /* 方向自检: dir_bad 明显多于 dir_ok 说明观测角是镜像形式(见 SMO.c 第 6 步注释);
     *   开环(无编码器)模式下没有编码器真值, 这里恒为"数据不足" */
    (void)CDC_Printf("# smo_dir ok=%u bad=%u -> %s\r\n",
                     (unsigned)smo_dir_ok, (unsigned)smo_dir_bad,
                     (Encoder_IsValid() == 0u)
                         ? "无编码器, 不判定(开环模式属正常)"
                         : ((smo_dir_bad > smo_dir_ok + 8u)
                                ? "★符号口径反了(dθ̂/dθ=-1): 把 SMO.c 的角度提取改成 atan2(+e_alpha, e_beta)"
                                : ((smo_dir_ok > 8u) ? "OK(观测角与编码器同向)" : "数据不足, 让电机转起来再看")));
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM2)
    {
        HAL_IncTick();
    }
}

void App_FOC_Init(void)
{
    Control_Init();
    Control_SetCurrentPID(APP_CURRENT_PID_KP, APP_CURRENT_PID_KI,
                          APP_CURRENT_PID_DT, APP_CURRENT_PID_LIMIT,
                          APP_CURRENT_PID_ILIMIT);
    Control_SetSpeedPID(APP_SPEED_PID_KP, APP_SPEED_PID_KI,
                        APP_SPEED_PID_DT, APP_SPEED_PID_LIMIT,
                        APP_SPEED_PID_ILIMIT);
    Control_SetPositionPID(APP_POSITION_PID_KP, APP_POSITION_PID_KI,
                            APP_POSITION_PID_DT, APP_POSITION_PID_LIMIT,
                            APP_POSITION_PID_ILIMIT);
    Control_SetSpeedLoop(1u);
    Encoder_Init();
    /* DWT 周期计数器: 打点时间戳 + 中断耗时统计(40kHz 下必须远小于 25us) */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0u;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    /* ★观测器抽取运行: 必须先告诉它抽取后的周期(Ts), 再 Init 计算 F/G */
    SMO_SetTs((float)SMO_DECIM / (float)APP_CTRL_HZ);
    SMO_Init();
    VF_Init();
    if (BSP_CAN_Init() != HAL_OK)
    {
        Error_Handler();
    }
    MX_USB_DEVICE_Init();
    BSP_Timer_Init();
    /* 把应用层处理函数挂到 ADC 中断上(底层只拿一个函数指针, 不认识它是什么) */
    BSP_AdcIrq_SetGroupHook(App_AdcStep);
    HAL_ADCEx_InjectedStart_IT(&hadc1);
    HAL_ADCEx_InjectedStart_IT(&hadc2);
    HAL_ADCEx_InjectedStart_IT(&hadc3);
    BSP_Timer_PWM_Start();

    {
        uint32_t calibration_start = HAL_GetTick();
        while (adc_cal_done == 0u)
        {
            if (HAL_GetTick() - calibration_start >= APP_ADC_CAL_TIMEOUT_MS)
            {
                adc_cal_done = 1u;
            }
        }
    }
    HAL_GPIO_WritePin(EN_Gate_GPIO_Port, EN_Gate_Pin, GPIO_PIN_SET);

    (void)CDC_Printf("# boot: SMO k=%.1f delta=%.1f alpha=%.3f lagw=%.0f | VF default "
                     "drv=%s rpm=%.0f ramp=%.0f slope=%.3f vmax=%.2f vboost=%.2f fstart=%.2f valign=%.2f\r\n",
                     (double)SMO_GetGain(), (double)SMO_GetBoundary(),
                     (double)SMO_EMF_LPF_ALPHA, (double)SMO_GetLagW(),
                     (VF_GetDriveMode() == VF_DRV_ENCODER) ? "encoder" : "openloop",
                     (double)OL_RPM_DEF, (double)OL_RAMP_DEF,
                     (double)OL_VSLOPE_DEF, (double)OL_VMAX_DEF,
                     (double)OL_VBOOST_DEF, (double)OL_FSTART_DEF, (double)OL_VALIGN_DEF);
    (void)CDC_Printf("# commands: vf <rpm> | stop | cur <A> | spd <rpm> | stream 1 | stat | help\r\n");
}

void App_FOC_MainLoop(void)
{
    if (HAL_GetTick() - can_feedback_ms >= APP_CAN_FEEDBACK_PERIOD_MS)
    {
        can_feedback_ms = HAL_GetTick();
        App_SendCanFeedback();
    }
    App_UsbPoll();
    App_SmoStreamDrain();
    App_PrintSmoStat();
    App_PrintOpenLoopLive();
    if (isr_cyc_reported == 0u && HAL_GetTick() >= 3000u)
    {
        isr_cyc_reported = 1u;
        (void)CDC_Printf("# ctrl_step_max=%u cyc = %.2f us (budget %.2f us @ %u Hz)\r\n",
                         (unsigned)BSP_AdcIrq_GetMaxCycle(),
                         (float)BSP_AdcIrq_GetMaxCycle() / 168.0f,
                         1.0e6f / (float)APP_CTRL_HZ,
                         (unsigned)APP_CTRL_HZ);
    }
    /* 中断分段耗时(开机 2 秒后打一次): 谁在吃预算一目了然 */
    if (prof_on != 0u && HAL_GetTick() >= 2000u)
    {
        prof_on = 0u;
        {
            uint32_t total = BSP_AdcIrq_GetMaxCycle();
            uint32_t cyc100 = SystemCoreClock / 1000000u;

            (void)CDC_Printf("# prof(2s内峰值): total=%.2fus | smo=%.2fus vf=%.2fus | "
                             "其余(采样/滤波/Park/打印)=%.2fus | 预算=%.2fus\r\n",
                             (float)total / (float)cyc100,
                             (float)prof_smo / (float)cyc100,
                             (float)prof_vf / (float)cyc100,
                             (float)(total > prof_smo + prof_vf
                                     ? total - prof_smo - prof_vf : 0u) / (float)cyc100,
                             1.0e6f / (float)APP_CTRL_HZ);
            (void)CDC_Printf("# prof 占比: smo=%.0f%% vf=%.0f%% 其余=%.0f%% (总 %u 拍)\r\n",
                             100.0f * (float)prof_smo / (float)total,
                             100.0f * (float)prof_vf / (float)total,
                             100.0f * (float)(total - prof_smo - prof_vf) / (float)total,
                             (unsigned)prof_ticks);
        }
    }
    /* 母线欠压故障事件: 一触发就封驱动 -> 电机不转, 这里明确打出来 */
    {
        static uint8_t vdc_fault_prev;
        if (vdc_fault != vdc_fault_prev)
        {
            vdc_fault_prev = vdc_fault;
            (void)CDC_Printf("# vdc_fault=%u vdc=%.2f V (阈值 %.1f/%.1f) drv=%u\r\n",
                             (unsigned)vdc_fault, (double)Power,
                             (double)APP_VDC_MIN, (double)APP_VDC_RECOVER,
                             (unsigned)((HAL_GPIO_ReadPin(FAULT_GPIO_Port, FAULT_Pin) == GPIO_PIN_RESET) ? 1u : 0u));
        }
    }
    HAL_Delay(1u);
}
