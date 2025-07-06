#pragma once

#include "tim.h"
#include "gpio.h"
#include "main/config.h"

typedef uint8_t ArmTim;
#define ARM_TIM_MIN    50
#define ARM_TIM_MAX   250
#define ARM_TIM_STEP    1

typedef struct ArmConst
{
    TIM_HandleTypeDef* htimx;
    uint32_t TIM_CHANNEL_x;
    ArmTim tim_min;
    ArmTim tim_max;
} ArmConst;
typedef struct ArmParameter
{
    const ArmConst* const_h;
    ArmTim tim_current;
    ArmTim tim_setpoint;
} ArmParameter;
extern ArmParameter arm_bottom;
extern ArmParameter arm_shoulder;
extern ArmParameter arm_elbow_btm;
extern ArmParameter arm_elbow_top;
extern ArmParameter arm_wrist;
extern ArmParameter arm_finger;
void arm_set_tim(ArmParameter* arm, ArmTim tim);
void arm_set_pos(ArmParameter* arm, uint8_t pos);
