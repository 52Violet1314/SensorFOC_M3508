#ifndef ENCODER_LUT_H
#define ENCODER_LUT_H

#include "Encoder.h"

/* ============================================================================
 * Encoder_Lut.c —— 编码器电角度 2000 点校正表的"固化"存放处
 *
 *   ★这个文件平时是全 0(等于不校正), 标定完由 tools/lut_capture.py 自动覆盖:
 *       python tools/lut_capture.py --port COM7 --gen
 *     生成的表写回本文件, 重新编译烧录后就永久生效 —— 不用每次上电重标。
 *
 *   表项含义: 第 i 格(θ_raw ∈ [i, i+1)·2π/2000)需要补偿的 rad。
 *   全 0 时行为与"完全不查表"一致。
 * ========================================================================== */

extern const float EncLut_Rom[ENCODER_LUT_SIZE];

#endif
