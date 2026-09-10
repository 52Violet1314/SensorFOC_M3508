#include "VF.h"
#include "SVPWM.h"
#include "main.h"
#include "App_Config.h"
#include "tim.h"
#include <math.h>

static float    vf_speed_rpm  = 0.0f;   /* 当前斜坡指令转速 rpm */
static float    vf_target_rpm = 0.0f;   /* 目标转速 rpm */
static float    vf_dt         = VF_DT;  /* 控制步长 s, 上电后自校准 */

/* 回调率自校准: 统计 200ms 内调用次数, dt = 1/rate */
static uint8_t  vf_calib    = 1u;
static uint32_t vf_calib_t0 = 0u;
static uint32_t vf_calib_n  = 0u;

/* 实测转速: 电角度窗口差分 + 一阶低通 */
static float    vf_theta_prev    = 0.0f;
static uint8_t  vf_theta_valid   = 0u;
static float    vf_spd_accum     = 0.0f;
static uint16_t vf_spd_cnt       = 0u;
static uint32_t vf_cmd_active_t0 = 0u;

float   vf_speed_meas_rpm = 0.0f;
uint8_t vf_running = 0u;
uint8_t vf_stall   = 0u;

void VF_SetTargetRPM(float rpm)
{
    if (rpm >  VF_MAX_RPM) rpm =  VF_MAX_RPM;
    if (rpm < -VF_MAX_RPM) rpm = -VF_MAX_RPM;
    vf_target_rpm = rpm;
}

/* 复位: 清斜坡与判断状态(每次重新使能前调用) */
void VF_Init(void)
{
    vf_speed_rpm      = 0.0f;
    vf_speed_meas_rpm = 0.0f;
    vf_theta_prev     = 0.0f;
    vf_theta_valid    = 0u;
    vf_spd_accum      = 0.0f;
    vf_spd_cnt        = 0u;
    vf_running        = 0u;
    vf_stall          = 0u;
    vf_cmd_active_t0  = 0u;
}

/* 控制环, 在 ADC 注入转换完成中断中调用
   驱动角度全程用编码器电角度(绝对角, 自同步), 不再软件积分模拟;
   电压按指令转速 V/F 前馈, 实测转速只用于启动/堵转判断 */
void VF_Tick(float vdc, float theta_elec)
{
    /* 0. 控制步长自校准(上电 200ms 内完成, 之后斜坡/测速时间常数准确) */
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

    /* 1. 指令转速斜坡: 限制电压上升率, 避免起步电流冲击 */
    float diff = vf_target_rpm - vf_speed_rpm;
    float step = VF_ACC_RPM_PER_S * vf_dt;
    if      (diff >  step) vf_speed_rpm += step;
    else if (diff < -step) vf_speed_rpm -= step;
    else                   vf_speed_rpm  = vf_target_rpm;

    /* 2. 实测转速: 电角度窗口差分 + 一阶低通 */
    if (vf_theta_valid)
    {
        float d = theta_elec - vf_theta_prev;
        if (d >  APP_PI) d -= APP_TWO_PI;
        if (d < -APP_PI) d += APP_TWO_PI;
        vf_spd_accum += d;
        vf_theta_prev = theta_elec;
        if (++vf_spd_cnt >= VF_SPD_WIN_TICKS)
        {
            float rpm = vf_spd_accum * 60.0f * 0.15915494309f
                        * (1.0f / vf_dt) *
                        (1.0f / (float)VF_SPD_WIN_TICKS) *
                        (1.0f / (float)VF_POLE_PAIRS);
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

    /* 3. 启动/堵转判断: 实测转速超阈值 => 已转起来(解除堵转限压);
          指令有速但实测长时间不转 => 堵转, 电压压到 V_BOOST 限流 */
    if (fabsf(vf_speed_meas_rpm) > VF_START_DETECT_RPM)
    {
        vf_running = 1u;
        vf_stall   = 0u;
    }
    else if (vf_stall == 0u && fabsf(vf_speed_rpm) > VF_START_DETECT_RPM)
    {
        if (vf_cmd_active_t0 == 0u) vf_cmd_active_t0 = HAL_GetTick();
        if (HAL_GetTick() - vf_cmd_active_t0 >= VF_STALL_MS) vf_stall = 1u;
    }
    else
    {
        vf_cmd_active_t0 = 0u;
    }

    /* 4. V/F 电压: 由指令电频率前馈, 限幅;
          起步阶段(未确认转动)叠加辅助电压破齿槽转矩, 转起来后自动撤掉;
          堵转时限压到基础电压限流 */
    float fe = vf_speed_rpm / 60.0f * VF_POLE_PAIRS;
    float v  = VF_V_BOOST + VF_V_SLOPE * fabsf(fe);
    if (vf_running == 0u) v += VF_START_EXTRA_V;
    if (v > VF_V_MAX) v = VF_V_MAX;
    if (vf_stall) v = VF_V_BOOST;

    /* 5. 驱动角度: 始终跟随编码器电角度+扭矩角(自同步, 停转也能直接给转矩) */
    float theta = theta_elec + VF_TORQUE_ANGLE_RAD;

    /* 6. αβ 电压 → SVPWM → 更新 CCR */
    uint16_t ccr1, ccr2, ccr3;
    SVPWM(v * cosf(theta), v * sinf(theta), vdc, &ccr1, &ccr2, &ccr3);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, ccr1);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, ccr2);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, ccr3);
}
