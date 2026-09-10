#ifndef __ENCODER_H__
#define __ENCODER_H__

#include <stdint.h>

/* SIN/COS 模拟编码器: 直接输出电角度(每机械圈 7 个电周期, 与极对数一致)。
 * 角度解算使用一圈标定得到的中心/幅值，不能用上电静止点估计中心。
 * 注意: 编码器已是电角度, 不要再乘极对数。机械位置接口会将电角度解包后除以7。
 */

#define ENCODER_MECH_OFFSET 0.0f   /* 原始角度偏置 rad, 一般保持 0 */
#define ENCODER_ELEC_OFFSET 0.5045f /* 电角度对齐偏移 rad, 空载实测标定(+0.5045) */

void    Encoder_Init(void);
void    Encoder_Update(int sin_raw, int cos_raw); /* 每个采样周期调用, 输入 SIN/COS ADC 原始值 */
uint8_t Encoder_IsValid(void);                    /* 最近一次 SIN/COS 解算是否有效 */
int     Encoder_GetSinMin(void);
int     Encoder_GetSinMax(void);
int     Encoder_GetCosMin(void);
int     Encoder_GetCosMax(void);
float   Encoder_GetMechAngle(void);               /* 原始电角度 rad, [-pi, pi] + MECH_OFFSET */
float   Encoder_GetElecAngle(void);               /* 电角度 rad, [0, 2pi) */
float   Encoder_GetMechanicalPosition(void);      /* 上电后连续机械位置 rad, 1机械圈=2pi */
void    Encoder_ResetMechanicalPosition(void);    /* 将当前机械位置设为0, 仅建立运行时相对零点 */

#endif
