#pragma once

#include <AP_HAL/AP_HAL.h>

typedef union {
    float fValue;
    uint8_t u8Data[4];
} FloatUchar4;

typedef struct _EepromAttribute {
    FloatUchar4 pressureRange;
    FloatUchar4 pressureMinimum;
    
    FloatUchar4 offsetCoefficient0;
    FloatUchar4 offsetCoefficient1;
    FloatUchar4 offsetCoefficient2;
    FloatUchar4 offsetCoefficient3;

    FloatUchar4 spanC0;
    FloatUchar4 spanC1;
    FloatUchar4 spanC2;
    FloatUchar4 spanC3;

    FloatUchar4 shapeCoefficient0;
    FloatUchar4 shapeCoefficient1;
    FloatUchar4 shapeCoefficient2;
    FloatUchar4 shapeCoefficient3;

    uint8_t adcCfg[4];
    uint8_t pressureUnit[5];
} EepromAttribute;

