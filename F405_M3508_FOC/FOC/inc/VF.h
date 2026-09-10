#ifndef __VF_H__
#define __VF_H__

#include <stdint.h>

/* 电机参数 */
#define VF_POLE_PAIRS     7u          /* 极对数(磁对数) */
#define VF_DT             0.00005f    /* 控制周期 s 初始值(ADC 注入中断), 上电后自校准 */

/* V/F 曲线: V = V_BOOST + V_START_EXTRA(未确认转动前) + V_SLOPE * fe, fe 为斜坡指令电频率 Hz
   起步阶段叠加辅助电压, 保证转矩突破齿槽转矩/静摩擦; 实测确认转动后自动撤掉 */
#define VF_V_BOOST        0.6f        /* 基础低速补偿电压 V(确认转动后) */
#define VF_START_EXTRA_V  1.0f        /* 起步辅助电压 V: 未确认转动前叠加, 破齿槽转矩 */
#define VF_V_SLOPE        0.06f       /* 电压/频率斜率 V/Hz, 覆盖反电动势+IR(按实测电流调) */
#define VF_V_MAX          6.0f        /* 输出电压峰值上限 V */

/* 加减速与限幅 */
#define VF_ACC_RPM_PER_S  30.0f       /* 指令转速斜坡斜率 rpm/s, 限制电压上升率 */
#define VF_MAX_RPM        100.0f      /* 目标转速限幅 rpm */

/* 磁场超前转子电角度, 90° = 最大转矩/安培; 若转向反了改成负值 */
#define VF_TORQUE_ANGLE_RAD  1.57079632679f

/* 实测转速判断(编码器电角度差分) */
#define VF_SPD_WIN_TICKS  50u         /* 测速窗口拍数: 20kHz x 50 = 2.5ms */
#define VF_SPD_LPF_ALPHA  0.2f        /* 转速一阶低通系数 */
#define VF_START_DETECT_RPM   10.0f   /* 实测转速超过此值判定"已转起来" */
#define VF_STALL_MS           1500u   /* 指令有速但实测长时间不转, 判定堵转(ms) */

extern float   vf_speed_meas_rpm;     /* 实测机械转速 rpm(编码器差分+低通), 供诊断打印 */
extern uint8_t vf_running;            /* 1 = 实测转速已超阈值, 启动成功 */
extern uint8_t vf_stall;              /* 1 = 判定堵转, 电压被限制在 V_BOOST 限流 */

void VF_Init(void);
void VF_SetTargetRPM(float rpm);
void VF_Tick(float vdc, float theta_elec);

#endif
