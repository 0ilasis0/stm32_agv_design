#pragma once

#include <stdint.h>
#include "tim.h"
#include "gpio.h"

typedef uint16_t ArmPosition;
#define ARM_POS_000    500
#define ARM_POS_012    750
#define ARM_POS_025   1000
#define ARM_POS_038   1250
#define ARM_POS_050   1500
#define ARM_POS_062   1750
#define ARM_POS_075   2000
#define ARM_POS_088   2250
#define ARM_POS_100   2500
#define ARM_POS_STEP    50

// #define ARM_DEG_000  500
// #define ARM_DEG_015  666
// #define ARM_DEG_030  834
// #define ARM_DEG_045 1000
// #define ARM_DEG_060 1166
// #define ARM_DEG_075 1334
// #define ARM_DEG_090 1500
// #define ARM_DEG_105 1666
// #define ARM_DEG_120 1834
// #define ARM_DEG_135 2000
// #define ARM_DEG_150 2116
// #define ARM_DEG_165 2334
// #define ARM_DEG_180 2500

typedef struct ArmConst
{
    TIM_HandleTypeDef* TIMx;
    uint32_t TIM_CHANNEL_x;
} ArmConst;
typedef struct ArmParameter
{
    const ArmConst* arm_const;
    uint16_t pos_current;
    uint16_t pos_setpoint;
} ArmParameter;
extern ArmParameter arm_bottom;
extern ArmParameter arm_shoulder;
extern ArmParameter arm_elbow_btm;
extern ArmParameter arm_elbow_top;
extern ArmParameter arm_wrist;
extern ArmParameter arm_finger;
