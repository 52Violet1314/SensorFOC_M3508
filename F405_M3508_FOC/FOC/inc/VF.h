#ifndef __VF_H__
#define __VF_H__

#include <stdint.h>

/* ============================================================================
 * VF.h —— 真正的开环运行通路(不需要编码器)
 *
 *   驱动角: θ_drv = 2π·∫fe dt  ← 固件自己积分, 这就是"开环"
 *   电压幅值: V = V_BOOST + V_SLOPE·fe(按指令频率前馈)
 *   反馈: 没有编码器时, 唯一能反映"电机转没转"的是滑模观测器的 |E| 与 ω̂:
 *           |E| = ωe·ψf 足够大 -> 转子确实在转;
 *           |E| 长期不达门槛   -> 没转/失步 -> 封驱动(OL_EMF_TIMEOUT_MS)
 *
 *   ★若台架上有编码器, 可把驱动角切回"编码器电角度+转矩角"(drv 1), 更稳。
 * ========================================================================== */

/* 极对数: 与 FOC_POLE_PAIRS 保持一致(7 对极 = 14 极) */
#define VF_POLE_PAIRS     7u

typedef struct
{
    float rpm_target;     /* 目标转速(电机侧 rpm) */
    float rpm_ramp;       /* 指令转速斜坡 rpm/s */
    float v_boost;        /* 低速补偿电压 V(峰值相电压) */
    float v_slope;        /* 电压/频率斜率 V/Hz */
    float v_max;          /* 电压幅值上限 V */
    float torque_angle;   /* 编码器模式下磁场超前电角度 rad */
    float f_start;        /* 开环积分角的起始电频率 Hz */
    float v_align;        /* 起步对齐电压 V */
    float v_boost_lo;     /* 低频强制启动电压 V(牵入成功前用, 解决起步堵转) */
    float f_res;          /* 低频强制段的上界电频率 Hz */
} VF_Config_t;

/* 控制步长自校准后的实际值 s(ADC 注入中断周期) */
extern float   vf_dt;
/* 指令转速(斜坡后)与"实测"转速, 都是电机侧 rpm。
 *   ★传感: 有编码器时用编码器差分, 无编码器时用观测器的 ω̂(见 VF_Tick 注释) */
extern float   vf_speed_rpm;        /* 指令 */
extern float   vf_speed_meas_rpm;   /* 反馈 */
extern float   vf_valpha;           /* 本拍实际施加的 αβ 指令电压(给 SMO) */
extern float   vf_vbeta;
extern float   vf_theta_cmd;        /* 本拍驱动电角度(开环积分角或编码器角) */
extern float   vf_emf_est;          /* 观测到的 |E| V(无编码器时的唯一反馈) */
extern float   vf_rpm_obs;          /* 观测器给的电机侧转速 rpm */
extern uint8_t vf_running;          /* 1 = |E| 已确认电机在转 */
extern uint8_t vf_stall;            /* 1 = 判定失步/堵转(电压已加但没反电动势) */
extern uint8_t vf_aligned;          /* 1 = 起步对齐阶段已结束 */
extern float   vf_v_out;            /* 本拍输出电压幅值 V(诊断: 起步段是否给了足够电压) */
extern uint8_t vf_boost_lo_on;      /* 1 = 正在低频强制段 */

/* 驱动角来源 */
#define VF_DRV_OPENLOOP   0u        /* 开环积分角(默认, 不需要编码器) */
#define VF_DRV_ENCODER    1u        /* 编码器电角度 + 转矩角 */

/* 失步判定: |E| 要超过这个值才算"转起来了"。
 *   ★必须 ≥ OBS_EMF_MIN_V(0.3V), 否则观测器自己都认为角度不可信, 这里却当成"已牵入",
 *     结果是永远 running=0、失步保护必然触发(实测踩过: 原来写 0.25V)。 */
#define VF_RUN_EMF_V          0.35f
#define VF_SPD_WIN_TICKS      50u      /* 编码器测速窗口拍数: 40kHz x 50 = 1.25ms */
#define VF_SPD_LPF_ALPHA      0.2f     /* 编码器测速一阶低通 */
#define VF_START_DETECT_RPM   30.0f    /* 实测转速超过此值判定"已转起来" */
#define VF_STALL_MS           1500u    /* 旧口径(编码器模式)堵转判定 ms */

/* --- 配置默认值(定义在 FOC_Param.h, 这里只兜底: 若那边没定义才用这里的值) --- */
#ifndef OL_RPM_DEF
#define OL_RPM_DEF       1500.0f
#define OL_RAMP_DEF      600.0f
#define OL_VBOOST_DEF    1.0f
#define OL_VSLOPE_DEF    0.05f
#define OL_VMAX_DEF      6.5f
#define OL_TORQUE_ANGLE_DEF 1.57079632679f
#define OL_RPM_MAX       6000.0f
#define OL_FSTART_DEF    2.0f
#define OL_VALIGN_DEF    1.5f
#define OL_ALIGN_MS      300u
#define OL_EMF_TIMEOUT_MS 2000u
#define OL_VBOOST_LO_DEF 2.8f
#define OL_FRES_HZ       6.0f
#endif
#define VF_VBOOST_MIN   0.0f
#define VF_VBOOST_MAX   10.0f
#define VF_VSLOPE_MIN   0.0f
#define VF_VSLOPE_MAX   1.0f
#define VF_VMAX_MIN     0.5f
#define VF_VMAX_MAX     13.0f
#define VF_RAMP_MIN     10.0f
#define VF_RAMP_MAX     20000.0f
#define VF_FSTART_MIN   0.1f
#define VF_FSTART_MAX   20.0f
#define VF_VALIGN_MIN   0.0f
#define VF_VALIGN_MAX   10.0f
#define VF_VBLO_MIN     0.0f
#define VF_VBLO_MAX     12.0f
#define VF_FRES_MIN     0.5f
#define VF_FRES_MAX     50.0f

void  VF_Init(void);                        /* 复位斜坡/积分角/判定状态 */
void  VF_Reset(void);
void  VF_SetEnable(uint8_t on);             /* 使能开环通路(内部先走对齐阶段) */
uint8_t VF_IsEnabled(void);
/* 使能审计: 记录每次"开/关"的变化, 用来查"谁把开环关掉了" */
uint32_t VF_GetEnSetCnt(void);
uint32_t VF_GetEnClrCnt(void);
const uint8_t *VF_GetEnLog(void);
uint8_t VF_GetEnLogN(void);
void  VF_Tick(float vdc, float theta_elec); /* 在 40kHz ADC 中断里调用 */

const VF_Config_t *VF_GetConfig(void);
void  VF_SetTargetRPM(float rpm);
float VF_GetTargetRPM(void);
void  VF_SetRamp(float rpm_per_s);
void  VF_SetVBoost(float v);
void  VF_SetVSlope(float v_per_hz);
void  VF_SetVMax(float v);
void  VF_SetTorqueAngle(float rad);
void  VF_SetDriveMode(uint8_t mode);        /* VF_DRV_OPENLOOP / VF_DRV_ENCODER */
uint8_t VF_GetDriveMode(void);
void  VF_SetFStart(float hz);
void  VF_SetVAlign(float v);
void  VF_SetVBoostLo(float v);
void  VF_SetFRes(float hz);
void  VF_SetStallEnable(uint8_t on);   /* 0 = 关掉失步保护(台架调试用, 会失去堵转保护) */
uint8_t VF_GetStallEnable(void);
/* 台架点动/定位模式(排查驱动用): 在固定电角度上恒定加电压, 不做积分、不做保护。
 *   volts = 0 退出。转子会被拉到该电角度并"吸"在那里(用手拧能感觉到定位力)。
 *   ★这是判断"驱动到底有没有把电流送进电机"最直接的手段:
 *     有力/会吸住 -> 功率级 + 接线正常, 问题在控制逻辑或参数;
 *     完全没感觉   -> 驱动器/接线/供电有问题, 跟观测器、V/F 都无关。 */
void  VF_SetPark(float degrees, float volts);
float VF_GetParkDeg(void);
float VF_GetParkV(void);
/* 开环自检: 用编码器实测角度反推实际电频率, 判断 vf_dt/频率换算对不对 */
void  VF_DebugFreq(float theta_elec, float dt);
float VF_GetDbgFeAct(void);
float VF_GetDbgFeCmd(void);
float VF_GetDbgDt(void);
float VF_GetDt(void);
/* 观测器每拍的结果由 App_FOC.c 写进来(供失步判定/打点用) */
void  VF_SetObsFeedback(float emf, float rpm_obs);

#endif
