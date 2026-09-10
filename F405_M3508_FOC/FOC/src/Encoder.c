#include "Encoder.h"
#include "VF.h"
#include "App_Config.h"
#include <math.h>


/* 传感器一圈标定得到的极值。静止时不能用当前点估计中心，否则
 * atan2f(0, 0) 会把 ADC 噪声放大成角度跳变。根据实测范围设置，
 * 更换传感器时只需重新测一圈并修改这四个值。 */
#define ENC_SIN_MIN_COUNTS 442.0f
#define ENC_SIN_MAX_COUNTS 2074.0f
#define ENC_COS_MIN_COUNTS 453.0f
#define ENC_COS_MAX_COUNTS 2068.0f
#define ENC_MIN_VECTOR     0.05f

/* 历史问题说明:
 * 1) 旧代码用硬编码范围(SIN 320/238, COS 2241/2306), 与实际信号(SIN ~467-2064,
 *    COS ~477-2062)严重不符, 归一化值达到 -40~-4, 使 atan2 输出被卡死在 -pi 附近,
 *    表现为编码器角永远在一个假的 ~1.18 rad 窗口里振荡。
 * 2) 旧代码还把该角度乘了极对数 7, 而此编码器直接输出电角度(每机械圈 7 个电周期),
 *    因此必须去掉乘 7。 */

static float enc_angle   = 0.0f;   /* 电角度 rad, [-pi, pi] */
static float enc_prev_angle = 0.0f;
static float enc_mech_position = 0.0f;
static uint8_t enc_position_valid = 0u;
static uint8_t enc_sample_valid = 0u;
/* 动态观测范围，仅用于标定诊断，不参与角度计算。 */
static int enc_obs_sin_min = 4095;
static int enc_obs_sin_max = 0;
static int enc_obs_cos_min = 4095;
static int enc_obs_cos_max = 0;

static void Encoder_UpdateMechanicalPosition(void);

void Encoder_Init(void)
{
    enc_angle   = 0.0f;
    enc_prev_angle = 0.0f;
    enc_mech_position = 0.0f;
    enc_position_valid = 0u;
    enc_sample_valid = 0u;
    enc_obs_sin_min = 4095;
    enc_obs_sin_max = 0;
    enc_obs_cos_min = 4095;
    enc_obs_cos_max = 0;
}

void Encoder_Update(int sin_raw, int cos_raw)
{
    if (sin_raw == 0 || cos_raw == 0)
    {
        enc_sample_valid = 0u;
        return;   /* ADC 未就绪(首拍 COS 可能尚未刷新) */
    }

    if (sin_raw < enc_obs_sin_min) enc_obs_sin_min = sin_raw;
    if (sin_raw > enc_obs_sin_max) enc_obs_sin_max = sin_raw;
    if (cos_raw < enc_obs_cos_min) enc_obs_cos_min = cos_raw;
    if (cos_raw > enc_obs_cos_max) enc_obs_cos_max = cos_raw;

    const float sin_center = (ENC_SIN_MIN_COUNTS + ENC_SIN_MAX_COUNTS) * 0.5f;
    const float cos_center = (ENC_COS_MIN_COUNTS + ENC_COS_MAX_COUNTS) * 0.5f;
    const float sin_amp = (ENC_SIN_MAX_COUNTS - ENC_SIN_MIN_COUNTS) * 0.5f;
    const float cos_amp = (ENC_COS_MAX_COUNTS - ENC_COS_MIN_COUNTS) * 0.5f;
    float sin_norm = ((float)sin_raw - sin_center) / sin_amp;
    float cos_norm = ((float)cos_raw - cos_center) / cos_amp;
    float vector_sq = sin_norm * sin_norm + cos_norm * cos_norm;
    if (vector_sq < ENC_MIN_VECTOR * ENC_MIN_VECTOR)
    {
        enc_sample_valid = 0u;
        return;
    }
    if (sin_norm > 1.2f) sin_norm = 1.2f;
    if (sin_norm < -1.2f) sin_norm = -1.2f;
    if (cos_norm > 1.2f) cos_norm = 1.2f;
    if (cos_norm < -1.2f) cos_norm = -1.2f;
    enc_angle = atan2f(sin_norm, cos_norm);   /* 电角度, [-pi, pi] */
    enc_sample_valid = 1u;
    Encoder_UpdateMechanicalPosition();
}

uint8_t Encoder_IsValid(void)
{
    return enc_sample_valid;
}

int Encoder_GetSinMin(void) { return enc_obs_sin_min; }
int Encoder_GetSinMax(void) { return enc_obs_sin_max; }
int Encoder_GetCosMin(void) { return enc_obs_cos_min; }
int Encoder_GetCosMax(void) { return enc_obs_cos_max; }

static void Encoder_UpdateMechanicalPosition(void)
{
    float delta;

    if (!enc_position_valid)
    {
        enc_prev_angle = enc_angle;
        enc_position_valid = 1u;
        return;
    }

    delta = enc_angle - enc_prev_angle;
    if (delta > APP_ENCODER_HALF_PI) delta -= APP_ENCODER_TWO_PI;
    if (delta < -APP_ENCODER_HALF_PI) delta += APP_ENCODER_TWO_PI;
    enc_mech_position += delta * APP_ENCODER_POLE_PAIRS_INV;
    enc_prev_angle = enc_angle;
}

float Encoder_GetMechAngle(void)
{
    return enc_angle + ENCODER_MECH_OFFSET;
}

float Encoder_GetElecAngle(void)
{
    float a = enc_angle + ENCODER_MECH_OFFSET + ENCODER_ELEC_OFFSET;
    a = fmodf(a, APP_ENCODER_TWO_PI);
    if (a < 0.0f) a += APP_ENCODER_TWO_PI;
    return a;
}

float Encoder_GetMechanicalPosition(void)
{
    return enc_mech_position;
}

void Encoder_ResetMechanicalPosition(void)
{
    enc_mech_position = 0.0f;
    enc_prev_angle = enc_angle;
    enc_position_valid = 1u;
}
