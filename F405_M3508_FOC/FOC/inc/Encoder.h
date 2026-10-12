#ifndef __ENCODER_H__
#define __ENCODER_H__

#include <stdint.h>

/* SIN/COS 模拟编码器: 直接输出电角度(每机械圈 7 个电周期, 与极对数一致)。
 * 角度解算使用一圈标定得到的中心/幅值，不能用上电静止点估计中心。
 * 注意: 编码器已是电角度, 不要再乘极对数。机械位置接口会将电角度解包后除以7。
 *
 * ============================================================================
 * 2000 点电角度校正表(用户要求 2000 点)
 *
 *   链路: sin/cos 原始值 -> 归一化 -> atan2 -> 粗角度 θ_raw
 *                                            |
 *                          θ = θ_raw + lut[idx(θ_raw)]   ← 本文件新增的这一步
 *
 *   表里每项 = 该角度格子上的"真实电角度 − 粗角度"(rad)。全 0 时与不查表完全
 *   等价, 所以加表本身不会让精度变差, 精度只取决于表里填了什么。
 *
 *   标定原理(为什么这样能测出来):
 *     把三相电压合成一个幅值固定、频率固定的旋转矢量去"拖"转子 —— 转子会被
 *     磁场锁住跟着转, 于是**驱动矢量的角度 φ 就是真实的转子电角度**。
 *     φ 是固件自己按 Ts 积分出来的, 与晶振同源、绝对均匀, 这就是唯一的真值基准。
 *     每一拍把 (φ − θ_raw) 累加到 θ_raw 所在的格子里, 转够 N 个电周期后取平均
 *     就是该格的校正量 —— 不需要假设转速恒定, 也不需要任何外部仪器。
 *
 *   ★不能拿闭环来标定: 速度环的反馈就是这个编码器, 它会把要测的误差直接
 *     抵消掉(实测出来永远是 0)。所以标定必须开环拖转。
 *   ★采样率约束: 10kHz 采样要分辨 2000 格, 电频率必须 ≤5Hz(每电周期 ≥2000 拍),
 *     默认用 APP_CAL_FREQ_HZ = 2Hz。转快了信息量不够, 表就是噪声。
 * ========================================================================== */

#define ENCODER_LUT_SIZE    2000u   /* 校正表点数 */

/* 标定合格判据(只挡"转子打滑"这种彻底失败): 平均 |φ − θ_raw| 超过这个值就认为
 *   转子没被拖住, 生成的表作废。★编码器本身误差大(十几度)是**正常**的, 那正是要
 *   用表修掉的东西, 所以这个门限不能设小。表到底修对了没有, 看第二遍标定打印的
 *   "查表后残差" —— 第一遍时表是全 0, 残差必然等于原始误差。 */
#define ENC_CAL_MAX_MEAN_ERR_RAD  0.60f

#define ENCODER_MECH_OFFSET 0.0f   /* 原始角度偏置 rad, 一般保持 0 */
#define ENCODER_ELEC_OFFSET 0.5045f /* 电角度对齐偏移 rad, 空载实测标定(+0.5045)。
                                     * ★标定过(表有效)之后这个偏移不用再加 ——
                                     *   表里已经包含了对齐量, 再加就重复了。 */

void    Encoder_Init(void);
void    Encoder_Update(int sin_raw, int cos_raw); /* 每个采样周期调用, 输入 SIN/COS ADC 原始值 */
uint8_t Encoder_IsValid(void);                    /* 最近一次 SIN/COS 解算是否有效 */
int     Encoder_GetSinMin(void);
int     Encoder_GetSinMax(void);
int     Encoder_GetCosMin(void);
int     Encoder_GetCosMax(void);
float   Encoder_GetMechAngle(void);               /* 原始电角度 rad, [-pi, pi] + MECH_OFFSET */
float   Encoder_GetElecAngle(void);               /* 电角度 rad, [0, 2pi) */
float   Encoder_GetRawElecAngle(void);            /* ★不查表的粗角度(标定/诊断用) */
float   Encoder_GetMechanicalPosition(void);      /* 上电后连续机械位置 rad, 1机械圈=2pi */
void    Encoder_ResetMechanicalPosition(void);    /* 将当前机械位置设为0, 仅建立运行时相对零点 */

/* ---- 2000 点校正表 ---- */
uint16_t Encoder_LutIndex(float theta);           /* 角度 -> 表项下标 */
float    Encoder_LutGet(uint16_t idx);            /* 读一项(导出用) */
void     Encoder_LutSet(uint16_t idx, float rad); /* 写一项(导入用) */
void     Encoder_LutClear(void);                  /* 清成全 0 = 不校正 */
uint8_t  Encoder_LutIsValid(void);                /* 表是否已标定过 */
void     Encoder_LutStats(float *dc_rad, float *pp_rad); /* 直流(对齐) / 峰峰(非线性) */
void     Encoder_LutLoadRom(void);                /* 上电把固化的表(Encoder_Lut.c)搬进 RAM */

/* ---- 标定(由 App_FOC.c 的标定状态机每拍调用) ---- */
void     Encoder_CalReset(void);                  /* 清累积, 开始一轮标定 */
void     Encoder_CalSample(float theta_raw, float theta_ref); /* 喂: 粗角度 + 参考角φ */
uint32_t Encoder_CalCount(void);                  /* 已采样本数(进度用) */
float    Encoder_CalSweepRev(void);               /* 粗角度累计扫过几圈电角度 */
/* 生成表。返回 0=成功 / 1=样本不足 / 2=覆盖不足 / 3=平均误差过大(打滑) */
uint8_t  Encoder_CalFinish(float *mean_abs_err_rad, float *max_abs_err_rad,
                           float *mean_abs_resid_rad,
                           uint16_t *used_bins);

#endif
