#include "Encoder.h"
#include "Encoder_Lut.h"
#include "App_Config.h"
#include <math.h>
#include <string.h>

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

static float enc_angle    = 0.0f;  /* 查表后的电角度 rad(下游都用它) */
static float enc_raw_angle = 0.0f; /* 未查表的粗角度 rad, [-pi, pi] */
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

/* ==========================================================================
 * 2000 点电角度校正表(推导见 Encoder.h 顶部)
 * ========================================================================== */
static float   enc_lut[ENCODER_LUT_SIZE];
static uint8_t enc_lut_valid;

/* 标定累积(标定模式下才写) */
static float    enc_cal_acc[ENCODER_LUT_SIZE];
static uint16_t enc_cal_cnt[ENCODER_LUT_SIZE];
static uint8_t  enc_cal_have[ENCODER_LUT_SIZE];
static uint32_t enc_cal_n;
static float    enc_cal_abs_sum;
static float    enc_cal_abs_max;
/* 查表后的残差(用**当前**的表算): 第一遍时表是全 0, 残差==原始误差;
 *   第二遍时表已经是第一遍的结果, 残差就是"这张表到底修对了没有"的唯一判据。 */
static float    enc_res_sum;
static uint32_t enc_res_n;
/* 粗角度累计扫过的角度: 标定失败的万能判据 —— 转子根本没转(卡住/打滑)时它接近 0,
 *   而样本数照样在涨, 只看样本数是看不出来的。 */
static float    enc_cal_sweep;
static float    enc_cal_raw_prev;
static uint8_t  enc_cal_raw_valid;

uint16_t Encoder_LutIndex(float theta)
{
    float a = theta;

    /* 归一到 [0, 2π) */
    a = fmodf(a, APP_TWO_PI);
    if (a < 0.0f) a += APP_TWO_PI;

    {
        uint16_t idx = (uint16_t)((a / APP_TWO_PI) * (float)ENCODER_LUT_SIZE);
        if (idx >= ENCODER_LUT_SIZE) idx = ENCODER_LUT_SIZE - 1u;
        return idx;
    }
}

float Encoder_LutGet(uint16_t idx)
{
    return (idx < ENCODER_LUT_SIZE) ? enc_lut[idx] : 0.0f;
}

void Encoder_LutSet(uint16_t idx, float rad)
{
    if (idx >= ENCODER_LUT_SIZE) return;
    enc_lut[idx] = rad;
    enc_lut_valid = 1u;
}

void Encoder_LutClear(void)
{
    memset(enc_lut, 0, sizeof(enc_lut));
    enc_lut_valid = 0u;
}

uint8_t Encoder_LutIsValid(void)
{
    return enc_lut_valid;
}

/* 表的统计量: dc = 直流分量(=电角度对齐偏差), pp = 峰峰(编码器一个电周期内的非线性)
 *   人看这两个数就能判断"偏的是对齐还是线性度" */
void Encoder_LutStats(float *dc_rad, float *pp_rad)
{
    uint16_t i;
    float mn = enc_lut[0];
    float mx = enc_lut[0];
    float sum = 0.0f;

    for (i = 0u; i < ENCODER_LUT_SIZE; i++)
    {
        sum += enc_lut[i];
        if (enc_lut[i] < mn) mn = enc_lut[i];
        if (enc_lut[i] > mx) mx = enc_lut[i];
    }
    if (dc_rad != NULL) *dc_rad = sum / (float)ENCODER_LUT_SIZE;
    if (pp_rad != NULL) *pp_rad = mx - mn;
}

void Encoder_LutLoadRom(void)
{
    uint16_t i;
    uint8_t any = 0u;

    memcpy(enc_lut, EncLut_Rom, sizeof(enc_lut));
    for (i = 0u; i < ENCODER_LUT_SIZE; i++)
    {
        if (enc_lut[i] != 0.0f) { any = 1u; break; }
    }
    enc_lut_valid = any;   /* 固化表全 0 = 还没标定过 */
}

/* 查表: 粗角度 + 表项, 并保持与上一拍连续(不跳 ±2π)
 *   ★索引口径必须和标定(Encoder_CalSample)完全一致, 两边都用**粗角度**:
 *     表就是"粗角度 → 需要补多少"的一张查找表, 简单、可预测、覆盖一定是满的。
 *     (曾经试过按"补偿后的角度"分格来精细化, 结果实战里出现 400 个空格子:
 *      分格口径一变, 覆盖就不再有保证。收益也没兑现, 已回退。) */
static float Encoder_LutApply(float theta_raw, float theta_prev)
{
    float a = theta_raw + enc_lut[Encoder_LutIndex(theta_raw)];

    if ((a - theta_prev) >  APP_PI) a -= APP_TWO_PI;
    else if ((a - theta_prev) < -APP_PI) a += APP_TWO_PI;

    return a;
}

/* --------------------------------------------------------------------------
 * 标定
 * ------------------------------------------------------------------------ */
void Encoder_CalReset(void)
{
    memset(enc_cal_acc, 0, sizeof(enc_cal_acc));
    memset(enc_cal_cnt, 0, sizeof(enc_cal_cnt));
    memset(enc_cal_have, 0, sizeof(enc_cal_have));
    enc_cal_n = 0u;
    enc_cal_abs_sum = 0.0f;
    enc_cal_abs_max = 0.0f;
    enc_res_sum = 0.0f;
    enc_res_n = 0u;
    enc_cal_sweep = 0.0f;
    enc_cal_raw_valid = 0u;
}

void Encoder_CalSample(float theta_raw, float theta_ref)
{
    float e = theta_ref - theta_raw;
    uint16_t idx;

    /* 累计粗角度扫过的角度(判"转子到底转没转"): 每拍增量都很小, 直接求和 */
    if (enc_cal_raw_valid != 0u)
    {
        float d = theta_raw - enc_cal_raw_prev;

        while (d >=  APP_PI) d -= APP_TWO_PI;
        while (d <  -APP_PI) d += APP_TWO_PI;
        enc_cal_sweep += fabsf(d);
    }
    enc_cal_raw_prev = theta_raw;
    enc_cal_raw_valid = 1u;

    /* 误差归一化到 [-π, π) */
    while (e >=  APP_PI) e -= APP_TWO_PI;
    while (e <  -APP_PI) e += APP_TWO_PI;

    /* ★按**粗角度**分格(和 Encoder_LutApply 口径一致):
     *   同一个粗角度一定对应同一个真实角度, 所以每格的均值就是该格要补的量, 不会抹平;
     *   覆盖也一定有保证(粗角度扫满一圈 = 2000 格全中)。 */
    idx = Encoder_LutIndex(theta_raw);
    enc_cal_acc[idx] += e;
    if (enc_cal_cnt[idx] < 60000u) enc_cal_cnt[idx]++;
    enc_cal_have[idx] = 1u;

    enc_cal_n++;
    enc_cal_abs_sum += fabsf(e);
    if (fabsf(e) > enc_cal_abs_max) enc_cal_abs_max = fabsf(e);

    /* 同时统计"查当前表之后还剩多少误差" —— 第二遍标定时这个数才是表好坏的判据 */
    {
        float r = theta_ref - (theta_raw + enc_lut[idx]);

        while (r >=  APP_PI) r -= APP_TWO_PI;
        while (r <  -APP_PI) r += APP_TWO_PI;
        enc_res_sum += fabsf(r);
        enc_res_n++;
    }
}

uint32_t Encoder_CalCount(void)
{
    return enc_cal_n;
}

/* 粗角度累计扫过多少圈(电角度): 标定结束时应当 ≈ APP_CAL_CYCLES。
 *   ~0 = 转子压根没转(卡住/没拖动) —— 这时候样本数照样是 20 万, 看样本数发现不了。 */
float Encoder_CalSweepRev(void)
{
    return enc_cal_sweep / APP_TWO_PI;
}

/* 空洞补齐(表是循环的, 两端要接起来) */
static void Encoder_LutFillGaps(void)
{
    uint16_t i;

    for (i = 0u; i < ENCODER_LUT_SIZE; i++)
    {
        uint16_t a;
        uint16_t b;
        uint16_t da = 0u;
        uint16_t db = 0u;
        float va;
        float vb;
        float d;

        if (enc_cal_have[i] != 0u) continue;

        a = i;
        b = i;
        do { a = (uint16_t)((a + ENCODER_LUT_SIZE - 1u) % ENCODER_LUT_SIZE); da++; }
        while ((enc_cal_have[a] == 0u) && (da < ENCODER_LUT_SIZE));
        do { b = (uint16_t)((b + 1u) % ENCODER_LUT_SIZE); db++; }
        while ((enc_cal_have[b] == 0u) && (db < ENCODER_LUT_SIZE));

        if ((da >= ENCODER_LUT_SIZE) || (db >= ENCODER_LUT_SIZE))
        {
            enc_lut[i] = 0.0f;      /* 整圈都没数据(不该发生) */
            continue;
        }

        va = enc_lut[a];
        vb = enc_lut[b];
        d  = vb - va;
        while (d >  APP_PI) d -= APP_TWO_PI;
        while (d < -APP_PI) d += APP_TWO_PI;
        enc_lut[i] = va + d * ((float)da / (float)(da + db));
    }
}

/* 环形 5 点滑动平均: 真实误差是低频谐波(≤ 几十次), 格子噪声是白的, 一滤就干净 */
static void Encoder_LutSmooth(void)
{
    static float tmp[ENCODER_LUT_SIZE];
    uint16_t i;

    for (i = 0u; i < ENCODER_LUT_SIZE; i++)
    {
        float s = 0.0f;
        for (int8_t k = -2; k <= 2; k++)
        {
            int32_t j = (int32_t)i + (int32_t)k;
            if (j < 0) j += (int32_t)ENCODER_LUT_SIZE;
            if (j >= (int32_t)ENCODER_LUT_SIZE) j -= (int32_t)ENCODER_LUT_SIZE;
            s += enc_lut[j];
        }
        tmp[i] = s * 0.2f;
    }
    memcpy(enc_lut, tmp, sizeof(enc_lut));
}

/* 返回: 0=成功; 1=样本不足; 2=覆盖不足; 3=平均误差过大(大概率打滑)
 *   ★统计量一定先填好再返回 —— 失败时必须能看到 n/覆盖/误差, 否则没法定位。
 *     (上一版就是失败时全填 0, 结果"样本 20 万却说覆盖 0", 白查了一轮) */
uint8_t Encoder_CalFinish(float *mean_abs_err_rad, float *max_abs_err_rad,
                          float *mean_abs_resid_rad, uint16_t *used_bins)
{
    uint16_t i;
    uint16_t used = 0u;
    float mean_abs;

    for (i = 0u; i < ENCODER_LUT_SIZE; i++)
    {
        if (enc_cal_cnt[i] > 0u) used++;
    }

    mean_abs = (enc_cal_n > 0u) ? (enc_cal_abs_sum / (float)enc_cal_n) : 0.0f;
    if (used_bins != NULL) *used_bins = used;
    if (mean_abs_err_rad != NULL) *mean_abs_err_rad = mean_abs;
    if (max_abs_err_rad != NULL) *max_abs_err_rad = enc_cal_abs_max;
    if (mean_abs_resid_rad != NULL)
    {
        *mean_abs_resid_rad = (enc_res_n > 0u) ? (enc_res_sum / (float)enc_res_n) : 0.0f;
    }

    if (enc_cal_n < ENCODER_LUT_SIZE) return 1u;   /* 样本太少 */
    if (used < (uint16_t)((ENCODER_LUT_SIZE * 9u) / 10u)) return 2u;  /* 覆盖不足 */

    /* 平均原始误差大得离谱 = 转子根本没被拖住(滑了), 这种表不能要。
     *   ★这个检查必须在**写表之前**: 否则一次打滑的运行会把好表覆盖成坏表,
     *     而 enc_lut_valid 还是 1(上一次成功留下的), 于是坏表被静默用上。
     *   ★它只挡"打滑"; 表准不准看第二遍的残差(见 Encoder.h)。 */
    if (mean_abs > ENC_CAL_MAX_MEAN_ERR_RAD) return 3u;

    for (i = 0u; i < ENCODER_LUT_SIZE; i++)
    {
        enc_lut[i] = (enc_cal_cnt[i] > 0u) ? (enc_cal_acc[i] / (float)enc_cal_cnt[i])
                                           : 0.0f;
    }
    Encoder_LutFillGaps();
    Encoder_LutSmooth();

    enc_lut_valid = 1u;
    return 0u;
}

/* -------------------------------------------------------------------------- */

void Encoder_Init(void)
{
    enc_angle   = 0.0f;
    enc_raw_angle = 0.0f;
    enc_prev_angle = 0.0f;
    enc_mech_position = 0.0f;
    enc_position_valid = 0u;
    enc_sample_valid = 0u;
    enc_obs_sin_min = 4095;
    enc_obs_sin_max = 0;
    enc_obs_cos_min = 4095;
    enc_obs_cos_max = 0;
    Encoder_CalReset();
    Encoder_LutClear();
    Encoder_LutLoadRom();   /* 固化过的表直接用 */
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
    float theta_raw;

    if (vector_sq < ENC_MIN_VECTOR * ENC_MIN_VECTOR)
    {
        enc_sample_valid = 0u;
        return;
    }
    if (sin_norm > 1.2f) sin_norm = 1.2f;
    if (sin_norm < -1.2f) sin_norm = -1.2f;
    if (cos_norm > 1.2f) cos_norm = 1.2f;
    if (cos_norm < -1.2f) cos_norm = -1.2f;

    theta_raw = atan2f(sin_norm, cos_norm);   /* 电角度, [-pi, pi] */
    enc_raw_angle = theta_raw;

    /* ★2000 点校正表(默认全 0, 与不校正完全等价)
     *   ★这里**不能**再写 enc_prev_angle = enc_angle: 下面的
     *     Encoder_UpdateMechanicalPosition() 正是用
     *     (enc_angle - enc_prev_angle) 求这一拍转过的角度, 提前把 prev 更新掉
     *     会让 delta 恒等于 0 —— 机械角永不累加、转速反馈恒为 0,
     *     现象就是"速度环没反馈、电机一路冲到极限"。prev 的更新由那个函数负责。 */
    enc_angle = Encoder_LutApply(theta_raw, enc_prev_angle);

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

float Encoder_GetRawElecAngle(void)
{
    return enc_raw_angle + ENCODER_MECH_OFFSET;
}

float Encoder_GetElecAngle(void)
{
    /* ★表有效时不再加 ENCODER_ELEC_OFFSET: 标定出来的表里已经含了对齐量,
     *   再加一次就等于把电角度整体偏掉 0.5rad(≈29°), 电机照样能转但效率、噪声都变差。 */
    float a = enc_angle + ENCODER_MECH_OFFSET +
              ((enc_lut_valid != 0u) ? 0.0f : ENCODER_ELEC_OFFSET);

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
