#include "SVPWM.h"
#include "main.h"
#include <stdint.h>

void SVPWM(float valpha, float vbeta, float vdc, uint16_t *ccr1, uint16_t *ccr2, uint16_t *ccr3)
{
    /* 母线电压无效时输出 50% 零压, 防止除零 */
    if (vdc < 1.0f)
    {
        *ccr1 = *ccr2 = *ccr3 = (uint16_t)(PWM_PERIOD / 2u);
        return;
    }

    /* ---- 第1步: 逆 Clarke, 得到三相相电压 ---- */
    float va = valpha;
    float vb = -0.5f * valpha + 0.86602540378f * vbeta;
    float vc = -0.5f * valpha - 0.86602540378f * vbeta;

    /* ---- 第2步: 零序分量注入 (min/max 法) ---- */
    float vmax = va;
    float vmin = va;
    if(vb > vmax) vmax = vb;
    if(vb < vmin) vmin = vb;
    if(vc > vmax) vmax = vc;
    if(vc < vmin) vmin = vc;
    float voffset = (vmax + vmin) * 0.5f;

    /* ---- 第3步: 映射到中心对齐占空比 [0, 1] ---- */
    float inv_vdc = 1.0f / vdc;
    float ta = (va - voffset) * inv_vdc + 0.5f;
    float tb = (vb - voffset) * inv_vdc + 0.5f;
    float tc = (vc - voffset) * inv_vdc + 0.5f;

    /* ---- 第4步: 限幅 + 映射到 CCR ---- */
    if(ta < 0.0f) ta = 0.0f; else if(ta > 1.0f) ta = 1.0f;
    if(tb < 0.0f) tb = 0.0f; else if(tb > 1.0f) tb = 1.0f;
    if(tc < 0.0f) tc = 0.0f; else if(tc > 1.0f) tc = 1.0f;

    *ccr1 = (uint16_t)(ta * PWM_PERIOD + 0.5f);
    *ccr2 = (uint16_t)(tb * PWM_PERIOD + 0.5f);
    *ccr3 = (uint16_t)(tc * PWM_PERIOD + 0.5f);
}
