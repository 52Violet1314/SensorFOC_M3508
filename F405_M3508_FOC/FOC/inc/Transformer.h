#ifndef __TRANSFORMER_H__
#define __TRANSFORMER_H__

/* 1/sqrt(3), Clarke 变换系数 */
#define SQRT3_INV        0.57735026919f

void ClarkTransformer(float ia, float ib, float ic, float *ialpha, float *ibeta);
void DeClarkTransformer(float *va, float *vb, float *vc, float valpha, float vbeta);
void ParkTransformer(float ialpha, float ibeta, float theta, float *iq, float *id);
void DeParkTransformer(float *valpha, float *vbeta, float theta, float vq, float vd);

#endif
