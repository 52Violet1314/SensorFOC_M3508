#ifndef FOC_ENCODER_MAP_H
#define FOC_ENCODER_MAP_H

#include <stdint.h>

/* Periodic nonlinearity correction generated from the latest synchronized
 * encoder/software-angle capture. */
#define FOC_ENCODER_LUT_ENABLE 1u
#define FOC_ENCODER_LUT_SIZE   128u

/* Convert an MT6701 mechanical count to a wrapped electrical angle in rad. */
float FOC_EncoderCountToElectricalAngleLinear(uint16_t mechanical_count);
float FOC_EncoderCountToElectricalAngle(uint16_t mechanical_count);

#endif
