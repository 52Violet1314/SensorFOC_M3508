#include "VF.h"
#include "SVPWM.h"
#include "main.h"
#include "App_Config.h"
#include "FOC_Param.h"
#include "tim.h"
#include <math.h>

/* ============================================================================
 * 开环(V/F)运行 —— 驱动角自己积分, 不需要编码器
 *
 *   VF_Tick() 在 40kHz ADC 注入中断里调用:
 *     ① 对齐阶段: 固定角 + v_align, 停 OL_ALIGN_MS, 把转子贴到已知位置
 *     ② 指令频率按 ramp 斜坡上升(从 f_start 起), 驱动角 = 2π∫fe dt 积分得到
 *     ③ 电压幅值 V = v_boost + v_slope·fe, 限幅 v_max
 *     ④ 反馈: 观测器给的 |E| / ω̂。|E| 长期不达门槛 -> 判定没转, 封驱动
 *
 *   ★为什么一定要"封驱动": 开环下电机若没转起来, 施加的电压全变成堵转电流,
 *     没有编码器就只剩"反电动势"这一个证据能发现它。
 * ========================================================================== */

static VF_Config_t vf_cfg = {
    OL_RPM_DEF,
    OL_RAMP_DEF,
    OL_VBOOST_DEF,
    OL_VSLOPE_DEF,
    OL_VMAX_DEF,
    OL_TORQUE_ANGLE_DEF,
    OL_FSTART_DEF,
    OL_VALIGN_DEF,
    OL_VBOOST_LO_DEF,
    OL_FRES_HZ
};

static uint8_t vf_enabled    = 0u;
static uint8_t vf_drive_mode = VF_DRV_OPENLOOP;   /* 默认: 开环积分角 */
static uint8_t vf_stall_enable = 1u;              /* 失步保护开关 */
static float   vf_target_rpm = 0.0f;

/* 开环积分角(电角度 rad, 累加在 [0,2π)) */
static float   vf_theta_int  = 0.0f;
static float   vf_align_cnt  = 0.0f;      /* 对齐阶段剩余拍数 */
static uint8_t vf_align_done = 0u;

/* 回调率自校准: 统计 200ms 内调用次数, dt = 1/rate */
static uint8_t  vf_calib    = 1u;
static uint32_t vf_calib_t0 = 0u;
static uint32_t vf_calib_n  = 0u;

/* 编码器测速(只在编码器模式下用) */
static float    vf_theta_prev    = 0.0f;
static uint8_t  vf_theta_valid   = 0u;
static float    vf_spd_accum     = 0.0f;
static uint16_t vf_spd_cnt       = 0u;
static uint32_t vf_cmd_active_t0 = 0u;

/* 观测器反馈(由 App_FOC.c 每拍写入) */
static float    vf_obs_emf   = 0.0f;
static float    vf_obs_rpm   = 0.0f;
static uint32_t vf_emf_low_t0 = 0u;
static uint32_t vf_emf_low_ms = 0u;

float   vf_dt             = 1.0f / (float)APP_CTRL_HZ;   /* 上电后自校准覆盖 */
float   vf_speed_rpm      = 0.0f;
float   vf_speed_meas_rpm = 0.0f;
float   vf_valpha         = 0.0f;
float   vf_vbeta          = 0.0f;
float   vf_theta_cmd      = 0.0f;
float   vf_emf_est        = 0.0f;
float   vf_rpm_obs        = 0.0f;
float   vf_v_out          = 0.0f;
uint8_t vf_running        = 0u;
uint8_t vf_stall          = 0u;
uint8_t vf_aligned        = 0u;
uint8_t vf_boost_lo_on    = 0u;

/* -------------------------------------------------------------------------- */
void VF_Init(void)
{
    vf_dt             = 1.0f / (float)APP_CTRL_HZ;
    vf_speed_rpm      = 0.0f;
    vf_speed_meas_rpm = 0.0f;
    vf_theta_int      = 0.0f;
    vf_theta_prev     = 0.0f;
    vf_theta_valid    = 0u;
    vf_spd_accum      = 0.0f;
    vf_spd_cnt        = 0u;
    vf_running        = 0u;
    vf_stall          = 0u;
    vf_aligned        = 0u;
    vf_align_done     = 0u;
    vf_boost_lo_on    = 0u;
    vf_v_out          = 0.0f;
    vf_cmd_active_t0  = 0u;
    vf_emf_low_t0     = 0u;
    vf_emf_low_ms     = 0u;
    vf_valpha         = 0.0f;
    vf_vbeta          = 0.0f;
    vf_theta_cmd      = 0.0f;
    vf_emf_est        = 0.0f;
    vf_rpm_obs        = 0.0f;
    vf_obs_emf        = 0.0f;
    vf_obs_rpm        = 0.0f;
    vf_enabled        = 0u;
    vf_target_rpm     = 0.0f;
    vf_calib          = 1u;
    vf_calib_n        = 0u;
}

void VF_Reset(void)
{
    VF_Init();
}

/* 使能/关断的审计记录: 每次状态变化都记下来, 谁把开环关掉了要能查出来。
 *   ★这几行由 App_FOC.c 的中断侧每 2 秒打印一次(不依赖主循环),
 *     所以"主循环卡死 / USB 命令没送达"这两种情况能区分开。 */
static uint8_t  vf_en_log[8];        /* 最近的 8 次状态变化: 1=开 0=关 */
static uint8_t  vf_en_log_n;
static uint32_t vf_en_set_cnt;       /* 累计"开"次数 */
static uint32_t vf_en_clr_cnt;       /* 累计"关"次数 */

static void vf_log_enable(uint8_t on)
{
    if (vf_en_log_n < 8u) vf_en_log[vf_en_log_n++] = on;
    else
    {
        for (uint8_t i = 1u; i < 8u; i++) vf_en_log[i - 1u] = vf_en_log[i];
        vf_en_log[7] = on;
    }
    if (on != 0u) vf_en_set_cnt++;
    else          vf_en_clr_cnt++;
}

uint32_t VF_GetEnSetCnt(void) { return vf_en_set_cnt; }
uint32_t VF_GetEnClrCnt(void) { return vf_en_clr_cnt; }
const uint8_t *VF_GetEnLog(void) { return vf_en_log; }
uint8_t VF_GetEnLogN(void) { return vf_en_log_n; }

void VF_SetEnable(uint8_t on)
{
    if (on != 0u)
    {
        vf_target_rpm = 0.0f;
        vf_speed_rpm  = 0.0f;
        vf_running    = 0u;
        vf_stall      = 0u;         /* ★重新使能必须清失步锁, 否则再也起不来 */
        vf_cmd_active_t0 = 0u;
        vf_emf_low_t0 = 0u;
        vf_emf_low_ms = 0u;
        vf_theta_int  = 0.0f;       /* 积分角从 0 起(对齐阶段就停在 0) */
        vf_align_cnt  = (float)(OL_ALIGN_MS) * (float)APP_CTRL_HZ / 1000.0f;
        vf_align_done = (vf_cfg.v_align <= 0.0f) ? 1u : 0u;
        vf_aligned    = vf_align_done;
    }
    else
    {
        vf_target_rpm = 0.0f;
        vf_speed_rpm  = 0.0f;
        vf_stall      = 0u;         /* ★关断也清, 保证下一次是干净状态 */
    }
    if (((on != 0u) ? 1u : 0u) != vf_enabled)   /* 只在真正变化时记一笔 */
    {
        vf_log_enable((on != 0u) ? 1u : 0u);
    }
    vf_enabled = (on != 0u) ? 1u : 0u;
}

uint8_t VF_IsEnabled(void)
{
    return vf_enabled;
}

const VF_Config_t *VF_GetConfig(void)
{
    return &vf_cfg;
}

void VF_SetTargetRPM(float rpm)
{
    if (rpm >  OL_RPM_MAX) rpm =  OL_RPM_MAX;
    if (rpm < -OL_RPM_MAX) rpm = -OL_RPM_MAX;
    vf_target_rpm = rpm;
}

float VF_GetTargetRPM(void)
{
    return vf_target_rpm;
}

void VF_SetRamp(float rpm_per_s)
{
    if (rpm_per_s < 0.0f) rpm_per_s = -rpm_per_s;
    if (rpm_per_s < VF_RAMP_MIN) rpm_per_s = VF_RAMP_MIN;
    if (rpm_per_s > VF_RAMP_MAX) rpm_per_s = VF_RAMP_MAX;
    vf_cfg.rpm_ramp = rpm_per_s;
}

void VF_SetVBoost(float v)
{
    if (v < VF_VBOOST_MIN) v = VF_VBOOST_MIN;
    if (v > VF_VBOOST_MAX) v = VF_VBOOST_MAX;
    vf_cfg.v_boost = v;
}

void VF_SetVSlope(float v_per_hz)
{
    if (v_per_hz < VF_VSLOPE_MIN) v_per_hz = VF_VSLOPE_MIN;
    if (v_per_hz > VF_VSLOPE_MAX) v_per_hz = VF_VSLOPE_MAX;
    vf_cfg.v_slope = v_per_hz;
}

void VF_SetVMax(float v)
{
    if (v < VF_VMAX_MIN) v = VF_VMAX_MIN;
    if (v > VF_VMAX_MAX) v = VF_VMAX_MAX;
    vf_cfg.v_max = v;
}

void VF_SetTorqueAngle(float rad)
{
    while (rad >  APP_PI) rad -= APP_TWO_PI;
    while (rad < -APP_PI) rad += APP_TWO_PI;
    vf_cfg.torque_angle = rad;
}

void VF_SetDriveMode(uint8_t mode)
{
    vf_drive_mode = (mode == VF_DRV_ENCODER) ? VF_DRV_ENCODER : VF_DRV_OPENLOOP;
}

uint8_t VF_GetDriveMode(void)
{
    return vf_drive_mode;
}

void VF_SetFStart(float hz)
{
    if (hz < VF_FSTART_MIN) hz = VF_FSTART_MIN;
    if (hz > VF_FSTART_MAX) hz = VF_FSTART_MAX;
    vf_cfg.f_start = hz;
}

void VF_SetVAlign(float v)
{
    if (v < VF_VALIGN_MIN) v = VF_VALIGN_MIN;
    if (v > VF_VALIGN_MAX) v = VF_VALIGN_MAX;
    vf_cfg.v_align = v;
}

void VF_SetVBoostLo(float v)
{
    if (v < VF_VBLO_MIN) v = VF_VBLO_MIN;
    if (v > VF_VBLO_MAX) v = VF_VBLO_MAX;
    vf_cfg.v_boost_lo = v;
}

void VF_SetFRes(float hz)
{
    if (hz < VF_FRES_MIN) hz = VF_FRES_MIN;
    if (hz > VF_FRES_MAX) hz = VF_FRES_MAX;
    vf_cfg.f_res = hz;
}

void VF_SetStallEnable(uint8_t on)
{
    vf_stall_enable = (on != 0u) ? 1u : 0u;
}

uint8_t VF_GetStallEnable(void)
{
    return vf_stall_enable;
}

/* 台架点动/定位: 固定电角度 + 恒定电压, 不做积分、不查保护 */
static float vf_park_deg = 0.0f;
static float vf_park_v   = 0.0f;

void VF_SetPark(float degrees, float volts)
{
    if (volts < 0.0f)  volts = 0.0f;
    if (volts > 12.0f) volts = 12.0f;
    while (degrees >= 360.0f) degrees -= 360.0f;
    while (degrees < 0.0f)    degrees += 360.0f;
    vf_park_deg = degrees;
    vf_park_v   = volts;
}

float VF_GetParkDeg(void) { return vf_park_deg; }
float VF_GetParkV(void)   { return vf_park_v; }

/* --------------------------------------------------------------------------
 * 开环自检: 用"编码器实测角度"反推实际电频率, 和指令频率对比。
 *
 *   为什么要这个: 开环驱动角是 θ += 2π·fe·vf_dt 积出来的, 完全依赖 vf_dt 标定。
 *   编码器闭环用的是硬件实测角, 不依赖 vf_dt —— 所以"闭环能转、开环不转"时,
 *   第一个要怀疑的就是 vf_dt / 频率换算。这里把三者摆在一起, 一眼就能看出错在哪:
 *     fe_cmd : 代码按 vf_speed_rpm 算出来的指令电频率
 *     fe_act : 编码器角度差分反推出来的实际电频率
 *     ratio  : fe_act / fe_cmd —— 正常≈1; 差 2 倍/10 倍就是 vf_dt 或换算错了
 *   注意: 必须在电机真的跟着转的时候看才有意义(没转的话 fe_act 自然是 0)。
 * ------------------------------------------------------------------------ */
static float    vf_dbg_theta_prev;
static uint8_t  vf_dbg_valid;
static float    vf_dbg_fe_act;
static float    vf_dbg_fe_cmd;
static float    vf_dbg_dt_used;

void VF_DebugFreq(float theta_elec, float dt)
{
    float d;

    vf_dbg_dt_used = dt;
    vf_dbg_fe_cmd  = vf_speed_rpm / 60.0f * (float)VF_POLE_PAIRS;

    if (dt <= 0.0f) return;

    if (vf_dbg_valid == 0u)
    {
        vf_dbg_theta_prev = theta_elec;
        vf_dbg_valid = 1u;
        return;
    }
    d = theta_elec - vf_dbg_theta_prev;
    if (d >  APP_PI) d -= APP_TWO_PI;
    if (d < -APP_PI) d += APP_TWO_PI;
    vf_dbg_theta_prev = theta_elec;

    /* 再低通一次, 免得被单点噪声带偏 */
    vf_dbg_fe_act += 0.002f * (d / APP_TWO_PI / dt - vf_dbg_fe_act);
}

float VF_GetDbgFeAct(void) { return vf_dbg_fe_act; }
float VF_GetDbgFeCmd(void) { return vf_dbg_fe_cmd; }
float VF_GetDbgDt(void)    { return vf_dbg_dt_used; }
float VF_GetDt(void)       { return vf_dt; }

void VF_SetObsFeedback(float emf, float rpm_obs)
{
    vf_obs_emf = emf;
    vf_obs_rpm = rpm_obs;
}

/* --------------------------------------------------------------------------
 * 控制环, 在 ADC 注入转换完成中断中调用
 * ------------------------------------------------------------------------ */
void VF_Tick(float vdc, float theta_elec)
{
    float v;
    float theta;
    uint16_t ccr1;
    uint16_t ccr2;
    uint16_t ccr3;

    /* 0. 台架点动/定位模式最高优先级: 固定角 + 恒定电压直接输出,
     *    不走斜坡、不走积分、不查失步 —— 专门用来验证"驱动器有没有把电流送进电机" */
    if (vf_enabled != 0u && vf_park_v > 0.05f)
    {
        theta = vf_park_deg * 0.01745329252f;
        v     = vf_park_v;
        vf_theta_cmd = theta;
        vf_v_out     = v;
        vf_aligned   = 1u;
        vf_boost_lo_on = 1u;
        vf_valpha = v * cosf(theta);
        vf_vbeta  = v * sinf(theta);
        SVPWM(vf_valpha, vf_vbeta, vdc, &ccr1, &ccr2, &ccr3);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, ccr1);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, ccr2);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, ccr3);
        return;                     /* 点动模式下不更新失步计时/不触发保护 */
    }

    /* 0. 控制步长自校准(上电 200ms 内完成, 之后斜坡/积分时间常数准确) */
    if (vf_calib)
    {
        if (vf_calib_n == 0u) vf_calib_t0 = HAL_GetTick();
        vf_calib_n++;
        if (HAL_GetTick() - vf_calib_t0 >= 200u)
        {
            vf_dt = 0.001f * 200.0f / (float)vf_calib_n;
            vf_calib = 0u;
        }
    }

    /* 1. 指令转速斜坡 */
    {
        float diff = vf_target_rpm - vf_speed_rpm;
        float step = vf_cfg.rpm_ramp * vf_dt;
        if      (diff >  step) vf_speed_rpm += step;
        else if (diff < -step) vf_speed_rpm -= step;
        else                   vf_speed_rpm  = vf_target_rpm;
    }

    /* 2. 反馈转速: 编码器模式用编码器差分; 开环模式用观测器的 ω̂ */
    if (vf_drive_mode == VF_DRV_ENCODER)
    {
        if (vf_theta_valid)
        {
            float d = theta_elec - vf_theta_prev;
            if (d >  APP_PI) d -= APP_TWO_PI;
            if (d < -APP_PI) d += APP_TWO_PI;
            vf_spd_accum += d;
            vf_theta_prev = theta_elec;
            if (++vf_spd_cnt >= VF_SPD_WIN_TICKS)
            {
                float rpm = vf_spd_accum * 9.54929658551f /
                            ((float)VF_POLE_PAIRS * vf_dt * (float)VF_SPD_WIN_TICKS);
                vf_speed_meas_rpm += VF_SPD_LPF_ALPHA * (rpm - vf_speed_meas_rpm);
                vf_spd_accum = 0.0f;
                vf_spd_cnt   = 0u;
            }
        }
        else
        {
            vf_theta_prev  = theta_elec;
            vf_theta_valid = 1u;
        }
    }
    else
    {
        vf_speed_meas_rpm = vf_obs_rpm;     /* 观测器转速, 每拍由 App_FOC.c 写入 */
    }
    vf_emf_est = vf_obs_emf;

    /* 3. 起步对齐: 固定角 + v_align, 停 OL_ALIGN_MS 让转子贴到已知位置 */
    if (vf_align_done == 0u)
    {
        if (vf_align_cnt > 0.0f) vf_align_cnt -= 1.0f;
        else                     vf_align_done = 1u;
        vf_aligned = vf_align_done;

        if (vf_align_done == 0u)
        {
            v = vf_cfg.v_align;
            vf_v_out = v;
            vf_boost_lo_on = 1u;
            theta = 0.0f;                  /* 固定角, 积分角保持 0 */
            vf_theta_int = 0.0f;
            goto OUTPUT;
        }
    }
    vf_aligned = 1u;

    /* 4. 开环积分角: θ += 2π·fe·dt,  fe 是指令电频率(最低 f_start) */
    {
        float fe = vf_speed_rpm / 60.0f * (float)VF_POLE_PAIRS;
        float fe_abs = fabsf(fe);

        if (fe_abs < vf_cfg.f_start)
        {
            fe = (fe >= 0.0f) ? vf_cfg.f_start : -vf_cfg.f_start;
        }

        vf_theta_int += APP_TWO_PI * fe * vf_dt;
        while (vf_theta_int >= APP_TWO_PI) vf_theta_int -= APP_TWO_PI;
        while (vf_theta_int <  0.0f)       vf_theta_int += APP_TWO_PI;

        /* 5. 电压:
         *    ① 牵入成功(|E| 建立)前, 或电频率还低于 f_res 时, 用低频强制电压 v_boost_lo,
         *       否则 M3508 带 19:1 减速箱在几 rpm 上根本起不来(表现就是堵转/一冲就停);
         *    ② 牵入后回到 V/F 曲线 V = v_boost + slope·fe;
         *    ③ 限幅 v_max(母线线性区)。 */
        v = vf_cfg.v_boost + vf_cfg.v_slope * fabsf(fe);
        vf_boost_lo_on = 0u;
        if (vf_running == 0u || fabsf(fe) < vf_cfg.f_res)
        {
            if (vf_cfg.v_boost_lo > v) v = vf_cfg.v_boost_lo;
            vf_boost_lo_on = 1u;
        }
        if (v > vf_cfg.v_max) v = vf_cfg.v_max;
        vf_v_out = v;

        /* 6. 驱动角 */
        theta = (vf_drive_mode == VF_DRV_ENCODER)
                    ? (theta_elec + vf_cfg.torque_angle)
                    : vf_theta_int;
    }

OUTPUT:
    vf_theta_cmd = theta;
    vf_valpha    = v * cosf(theta);
    vf_vbeta     = v * sinf(theta);

    /* 7. 失步/堵转: 对齐窗口之外, 电压已经加上去(|v| 有值), |E| 却长期不达门槛。
     *    ★两个前提, 否则会误判:
     *      ① 对齐阶段(300ms)转子本来静止、没有反电动势 -> 不计时;
     *      ② 指令电频率要先超过"该转速下反电动势才可能达到门槛"的值
     *         (fe_min = VF_RUN_EMF_V/(2π·ψf)), 否则低转速段本来就没有 |E|。
     *    用 HAL_GetTick 计时(整数毫秒), 不用 ISR 拍数, 避免自校准前 vf_dt 不准。 */
    if (vf_enabled != 0u && vf_align_done != 0u && vf_stall_enable != 0u)
    {
        uint32_t now = HAL_GetTick();
        float fe_now = vf_speed_rpm / 60.0f * (float)VF_POLE_PAIRS;
        float fe_min = VF_RUN_EMF_V / (APP_TWO_PI * FOC_FLUX);

        if (fabsf(fe_now) < fe_min)
        {
            vf_emf_low_t0 = 0u;         /* 频率还太低, 不该有反电动势, 不计时 */
            vf_emf_low_ms = 0u;
        }
        else if (vf_obs_emf >= VF_RUN_EMF_V)
        {
            vf_running    = 1u;
            vf_emf_low_t0 = 0u;
            vf_emf_low_ms = 0u;
        }
        else if (fabsf(v) > 0.5f)
        {
            if (vf_emf_low_t0 == 0u) vf_emf_low_t0 = now;
            vf_emf_low_ms = now - vf_emf_low_t0;
            if (vf_emf_low_ms >= OL_EMF_TIMEOUT_MS) vf_stall = 1u;
        }
        else
        {
            vf_emf_low_t0 = 0u;
        }
    }

    /* 8. αβ 电压 → SVPWM → 更新 CCR */
    SVPWM(vf_valpha, vf_vbeta, vdc, &ccr1, &ccr2, &ccr3);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, ccr1);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, ccr2);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, ccr3);
}
