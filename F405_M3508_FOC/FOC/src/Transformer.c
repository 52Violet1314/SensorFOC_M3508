#include "Transformer.h"
#include "main.h"
#include <math.h>
#include <stdint.h>
// ======== Clarke: ia → iα, iβ = (ia+2ib)/√3 ==========
void ClarkTransformer(float ia, float ib, float ic, float *ialpha, float *ibeta)
{
    (void)ic;  // 用不到 ic, 由 ia+ib+ic=0 消去
    *ialpha = ia;
    *ibeta = (ia + 2.0f * ib) * SQRT3_INV;
}

// ======== 逆 Clarke: Vα,Vβ → Va,Vb,Vc ==========
void DeClarkTransformer(float *va, float *vb, float *vc, float valpha, float vbeta)
{
    *va = valpha;
    *vb = -0.5f * valpha + 0.86602540378f * vbeta;  // √3/2
    *vc = -0.5f * valpha - 0.86602540378f * vbeta;
}

// ======== Park: iα,iβ → id,iq ==========
void ParkTransformer(float ialpha, float ibeta, float theta, float *iq, float *id)
{
    float c = cosf(theta);
    float s = sinf(theta);
    *id =  ialpha * c + ibeta * s;
    *iq = -ialpha * s + ibeta * c;
}

// ======== 逆 Park: Vd,Vq → Vα,Vβ ==========
void DeParkTransformer(float *valpha, float *vbeta, float theta, float vq, float vd)
{
    float c = cosf(theta);
    float s = sinf(theta);
    *valpha = vd * c - vq * s;
    *vbeta  = vd * s + vq * c;
}
