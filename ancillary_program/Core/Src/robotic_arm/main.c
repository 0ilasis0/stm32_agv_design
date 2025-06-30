#include "robotic_arm/main.h"

static const ArmConst arm_bottom_const = {
    // PA0(L28) 
    .TIMx = &htim2,
    .TIM_CHANNEL_x = TIM_CHANNEL_1,
    .tim_min = ARM_TIM_MIN,
    .tim_max = ARM_TIM_MAX,
};
ArmParameter arm_bottom = {
    .arm_const = &arm_bottom_const,
    .tim_current = 150,
    .tim_setpoint = 150,
};

static const ArmConst arm_shoulder_const = {
    // PA1(L30)
    .TIMx = &htim2,
    .TIM_CHANNEL_x = TIM_CHANNEL_2,
    .tim_min = ARM_TIM_MIN,
    .tim_max = ARM_TIM_MAX,
};
ArmParameter arm_shoulder = {
    .arm_const = &arm_shoulder_const,
    .tim_current = 150,
    .tim_setpoint = 150,
};

static const ArmConst arm_elbow_btm_const = {
    // PB10(R25)
    .TIMx = &htim2,
    .TIM_CHANNEL_x = TIM_CHANNEL_3,
    .tim_min = ARM_TIM_MIN,
    .tim_max = ARM_TIM_MAX,
};
ArmParameter arm_elbow_btm = {
    .arm_const = &arm_elbow_btm_const,
    .tim_current = 150,
    .tim_setpoint = 150,
};

static const ArmConst arm_elbow_top_const = {
    // PA6(R13)
    .TIMx = &htim3,
    .TIM_CHANNEL_x = TIM_CHANNEL_1,
    .tim_min = ARM_TIM_MIN,
    .tim_max = ARM_TIM_MAX,
};
ArmParameter arm_elbow_top = {
    .arm_const = &arm_elbow_top_const,
    .tim_current = 150,
    .tim_setpoint = 150,
};

static const ArmConst arm_wrist_const = {
    // PA4(L32)
    .TIMx = &htim3,
    .TIM_CHANNEL_x = TIM_CHANNEL_2,
    .tim_min = ARM_TIM_MIN,
    .tim_max = ARM_TIM_MAX,
};
ArmParameter arm_wrist = {
    .arm_const = &arm_wrist_const,
    .tim_current = 150,
    .tim_setpoint = 150,
};

static const ArmConst arm_finger_const = {
    // PB0(L34)
    .TIMx = &htim3,
    .TIM_CHANNEL_x = TIM_CHANNEL_3,
    .tim_min = ARM_TIM_MIN,
    .tim_max = ARM_TIM_MAX / 2,
};
ArmParameter arm_finger = {
    .arm_const = &arm_finger_const,
    .tim_current = 150,
    .tim_setpoint = 150,
};

void arm_set_tim(ArmParameter* arm, ArmTim tim)
{
    const ArmConst* arm_const = arm->arm_const;
    if      (tim < arm_const->tim_min) arm->tim_setpoint = arm_const->tim_min;
    else if (tim > arm_const->tim_max) arm->tim_setpoint = arm_const->tim_max;
    arm->tim_setpoint = tim;
}

inline void arm_set_pos(ArmParameter* arm, uint8_t pos)
{
    if (pos > 100) pos = 100;
    arm_set_tim(arm, pos*2 + 50);
}

static void arm_setup(ArmParameter* arm)
{
    const ArmConst* arm_const = arm->arm_const;
    HAL_TIM_PWM_Start(arm_const->TIMx, arm_const->TIM_CHANNEL_x);
}

static void arm_turn(ArmParameter* arm)
{
    if (arm->tim_current == arm->tim_setpoint) return;
    int16_t dtim = arm->tim_setpoint - arm->tim_current;
    if      (dtim >  ARM_TIM_STEP)  arm->tim_current += ARM_TIM_STEP;
    else if (dtim < -ARM_TIM_STEP)  arm->tim_current -= ARM_TIM_STEP;
    else                            arm->tim_current  = arm->tim_setpoint;
    // const ArmConst* arm_const = arm->arm_const;
    // if (arm->tim_current < arm_const->tim_min)
    //     arm->tim_current = arm_const->tim_min;
    // else if (arm->tim_current > arm_const->tim_max)
    //     arm->tim_current = arm_const->tim_max;
    __HAL_TIM_SET_COMPARE(arm->arm_const->TIMx, arm->arm_const->TIM_CHANNEL_x, arm->tim_current);
}

void StartArmTask(void *argument)
{
    arm_setup(&arm_bottom);
    arm_setup(&arm_shoulder);
    arm_setup(&arm_elbow_btm);
    arm_setup(&arm_elbow_top);
    arm_setup(&arm_wrist);
    arm_setup(&arm_finger);
    uint16_t tick = 0;
    for(;;)
    {
        arm_turn(&arm_bottom);
        arm_turn(&arm_shoulder);
        arm_turn(&arm_elbow_btm);
        arm_turn(&arm_elbow_top);
        arm_turn(&arm_wrist);
        arm_turn(&arm_finger);
        if (tick % 100 == 0) tick = 0;
        osDelay(10);
        tick++;
    }
}
