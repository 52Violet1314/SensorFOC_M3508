#ifndef FOC_PARAM_H
#define FOC_PARAM_H

#include <stdint.h>

typedef struct
{
    struct
    {
        uint16_t ia;
        uint16_t ib;
        uint16_t ic;
        uint16_t va;
        uint16_t vb;
        uint16_t vc;
        uint16_t vbus;
    } raw;

    struct
    {
        float a;
        float b;
        float c;
    } current;

    struct
    {
        float a;
        float b;
        float c;
    } voltage;

    float bus_voltage;
} FOC_Param_t;

extern volatile FOC_Param_t foc_param;

void FOC_ParamUpdateAdc1(void);
void FOC_ParamUpdateAdc2(void);

#endif
