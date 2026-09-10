#include "SVPWM.h"

static float clamp01(float value)
{
    if (value < 0.0f) return 0.0f;
    if (value > 1.0f) return 1.0f;
    return value;
}

void SVPWM(float valpha, float vbeta, float vdc,
           uint16_t *ccr1, uint16_t *ccr2, uint16_t *ccr3)
{
    float va;
    float vb;
    float vc;
    float vmax;
    float vmin;
    float offset;
    float inv_vdc;

    if (ccr1 == 0 || ccr2 == 0 || ccr3 == 0)
    {
        return;
    }

    /* Invalid bus voltage means zero line voltage (50% duty). */
    if (vdc < 1.0f)
    {
        *ccr1 = *ccr2 = *ccr3 = (uint16_t)(SVPWM_PWM_PERIOD / 2u);
        return;
    }

    va = valpha;
    vb = -0.5f * valpha + 0.86602540378f * vbeta;
    vc = -0.5f * valpha - 0.86602540378f * vbeta;

    vmax = va;
    vmin = va;
    if (vb > vmax) vmax = vb;
    if (vc > vmax) vmax = vc;
    if (vb < vmin) vmin = vb;
    if (vc < vmin) vmin = vc;
    offset = 0.5f * (vmax + vmin);
    inv_vdc = 1.0f / vdc;

    *ccr1 = (uint16_t)(clamp01(0.5f + (va - offset) * inv_vdc) *
                       (float)SVPWM_PWM_PERIOD + 0.5f);
    *ccr2 = (uint16_t)(clamp01(0.5f + (vb - offset) * inv_vdc) *
                       (float)SVPWM_PWM_PERIOD + 0.5f);
    *ccr3 = (uint16_t)(clamp01(0.5f + (vc - offset) * inv_vdc) *
                       (float)SVPWM_PWM_PERIOD + 0.5f);
}
