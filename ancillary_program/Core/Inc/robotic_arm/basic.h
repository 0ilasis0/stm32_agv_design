#pragma once

#include "tim.h"
#include "gpio.h"
#include "main/config.h"

#define ARM_MAX 6

typedef uint8_t ArmTim;
#define ARM_TIM_MIN    50
#define ARM_TIM_MAX   250
#define ARM_TIM_STEP    5

typedef struct ArmMotorConst
{
    uint8_t id;
    TIM_HandleTypeDef* htimx;
    uint32_t TIM_CHANNEL_x;
    ArmTim tim_min;
    ArmTim tim_max;
    ArmTim tim_step;
} ArmMotorConst;
typedef struct ArmMotorParameter
{
    const ArmMotorConst const_h;
    ArmTim tim_current;
    ArmTim tim_setpoint;
} ArmMotorParameter;
extern ArmMotorParameter arm_bottom;
extern ArmMotorParameter arm_shoulder;
extern ArmMotorParameter arm_elbow_btm;
extern ArmMotorParameter arm_elbow_top;
extern ArmMotorParameter arm_wrist;
extern ArmMotorParameter arm_finger;

void arm_set_tim(ArmMotorParameter* arm, ArmTim tim);
void arm_set_pos(ArmMotorParameter* arm, uint8_t pos);
