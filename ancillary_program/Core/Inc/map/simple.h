#pragma once

#include "main/config.h"
#include "main/fn_state.h"

typedef enum SimpleDirect
{
    SIMD_INVALED,
    SIMD_FOWARD,
    SIMD_BACKWARD,
    SIMD_LEFT,
    SIMD_RIGHT,
} SimpleDirect;

typedef struct SimplePoint
{
    uint32_t uid;
    SimpleDirect direct;
} SimplePoint;

Result simple_point_go(void);
Result simple_point_store(SimpleDirect direct);
Result simple_point_select(uint32_t point);
