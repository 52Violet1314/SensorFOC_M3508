#include "Encoder_Map.h"

#include "FOC_Config.h"
#include "mt6701.h"

#define FOC_ENCODER_TWO_PI 6.28318530718f

static uint32_t encoder_linear_electrical_count(uint16_t mechanical_count)
{
    int32_t signed_count;
    int32_t electrical_count;

    signed_count = (FOC_ENCODER_DIRECTION < 0) ?
                   -(int32_t)mechanical_count : (int32_t)mechanical_count;
    electrical_count = signed_count * (int32_t)FOC_MOTOR_POLE_PAIRS;
    electrical_count %= (int32_t)MT6701_COUNTS_PER_TURN;
    if (electrical_count < 0)
    {
        electrical_count += (int32_t)MT6701_COUNTS_PER_TURN;
    }
    return (uint32_t)electrical_count;
}

/* Signed correction in electrical counts for equally spaced electrical-angle
 * points. One count is 2*pi/16384 electrical radians. The table is indexed by
 * the linear electrical count, i.e. the same phase reported as raw_mrad. */
#if FOC_ENCODER_LUT_ENABLE
static const int16_t foc_encoder_correction_lut[FOC_ENCODER_LUT_SIZE] = {
    173, 36, 45, 56, 145, 20, 82, 157,
    -76, 59, 87, -13, 44, 13, -103, 40,
    -97, -72, -33, -13, -60, -48, 49, -135,
    -116, -69, -8, -65, -17, -100, 43, -281,
    -16, -104, -102, -110, -237, -19, -109, -75,
    -37, -171, -49, -20, -219, -60, -22, 128,
    -29, -1, 303, -80, 208, -3, 20, 129,
    -79, 104, 106, 99, 154, 58, 160, 27,
    118, 4, 82, 229, 34, 86, -51, 116,
    84, 38, 47, 4, -11, -204, 22, -91,
    -51, -12, -61, 37, 39, -122, 21, -26,
    -50, -33, -101, 137, -45, -25, -25, -128,
    -57, -36, -97, -197, -58, -177, -85, -2,
    38, -156, 56, -159, 2, -206, 39, -110,
    107, 24, 37, 77, -17, -22, 165, 25,
    62, 46, 15, 117, 130, 151, 95, 127
};
#endif

float FOC_EncoderCountToElectricalAngleLinear(uint16_t mechanical_count)
{
    return (float)encoder_linear_electrical_count(mechanical_count) *
           (FOC_ENCODER_TWO_PI / (float)MT6701_COUNTS_PER_TURN);
}

float FOC_EncoderCountToElectricalAngle(uint16_t mechanical_count)
{
    uint32_t electrical_count;

    electrical_count = encoder_linear_electrical_count(mechanical_count);

#if FOC_ENCODER_LUT_ENABLE
    {
        uint32_t lut_position;
        uint32_t lut_index;
        uint32_t lut_fraction;
        uint32_t next_index;
        int32_t correction;
        int32_t correction_next;
        int32_t corrected_count;

        lut_position = (electrical_count * FOC_ENCODER_LUT_SIZE);
        lut_index = lut_position / MT6701_COUNTS_PER_TURN;
        lut_fraction = lut_position % MT6701_COUNTS_PER_TURN;
        next_index = (lut_index + 1u) % FOC_ENCODER_LUT_SIZE;
        correction = foc_encoder_correction_lut[lut_index];
        correction_next = foc_encoder_correction_lut[next_index];
        correction += ((correction_next - correction) *
                       (int32_t)lut_fraction) /
                      (int32_t)MT6701_COUNTS_PER_TURN;

        corrected_count = (int32_t)electrical_count + correction;
        corrected_count %= (int32_t)MT6701_COUNTS_PER_TURN;
        if (corrected_count < 0)
        {
            corrected_count += (int32_t)MT6701_COUNTS_PER_TURN;
        }
        electrical_count = (uint32_t)corrected_count;
    }
#endif

    return (float)electrical_count *
           (FOC_ENCODER_TWO_PI / (float)MT6701_COUNTS_PER_TURN);
}
