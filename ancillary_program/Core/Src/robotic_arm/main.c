#include "robotic_arm/main.h"

static const ArmConst arm_bottom_const = {
    // PB0(L34)
    .htimx = &htim3,
    .TIM_CHANNEL_x = TIM_CHANNEL_3,
    .tim_min = ARM_TIM_MIN,
    .tim_max = ARM_TIM_MAX / 2,
};
ArmParameter arm_bottom = {
    .const_h = &arm_bottom_const,
    .tim_current = 150,
    .tim_setpoint = 150,
};

static const ArmConst arm_shoulder_const = {
    // PA4(L32)
    .htimx = &htim3,
    .TIM_CHANNEL_x = TIM_CHANNEL_2,
    .tim_min = ARM_TIM_MIN,
    .tim_max = ARM_TIM_MAX,
};
ArmParameter arm_shoulder = {
    .const_h = &arm_shoulder_const,
    .tim_current = 150,
    .tim_setpoint = 150,
};

static const ArmConst arm_elbow_btm_const = {
    // PA1(L30)
    .htimx = &htim2,
    .TIM_CHANNEL_x = TIM_CHANNEL_2,
    .tim_min = ARM_TIM_MIN,
    .tim_max = ARM_TIM_MAX,
};
ArmParameter arm_elbow_btm = {
    .const_h = &arm_elbow_btm_const,
    .tim_current = 150,
    .tim_setpoint = 150,
};

static const ArmConst arm_elbow_top_const = {
    // PB10(R25)
    .htimx = &htim2,
    .TIM_CHANNEL_x = TIM_CHANNEL_3,
    .tim_min = ARM_TIM_MIN,
    .tim_max = ARM_TIM_MAX,
};
ArmParameter arm_elbow_top = {
    .const_h = &arm_elbow_top_const,
    .tim_current = 150,
    .tim_setpoint = 150,
};

static const ArmConst arm_wrist_const = {
    // PA6(R13)
    .htimx = &htim3,
    .TIM_CHANNEL_x = TIM_CHANNEL_1,
    .tim_min = ARM_TIM_MIN,
    .tim_max = ARM_TIM_MAX,
};
ArmParameter arm_wrist = {
    .const_h = &arm_wrist_const,
    .tim_current = 150,
    .tim_setpoint = 150,
};

static const ArmConst arm_finger_const = {
    // PA0(L28) 
    .htimx = &htim2,
    .TIM_CHANNEL_x = TIM_CHANNEL_1,
    .tim_min = ARM_TIM_MIN,
    .tim_max = ARM_TIM_MAX,
};
ArmParameter arm_finger = {
    .const_h = &arm_finger_const,
    .tim_current = 150,
    .tim_setpoint = 150,
};

void arm_set_tim(ArmParameter* arm, ArmTim tim)
{
    const ArmConst* const_h = arm->const_h;
    if      (tim < const_h->tim_min) arm->tim_setpoint = const_h->tim_min;
    else if (tim > const_h->tim_max) arm->tim_setpoint = const_h->tim_max;
    arm->tim_setpoint = tim;
}

inline void arm_set_pos(ArmParameter* arm, uint8_t pos)
{
    if (pos > 100) pos = 100;
    arm_set_tim(arm, pos*2 + 50);
}

static void arm_setup(ArmParameter* arm)
{
    const ArmConst* const_h = arm->const_h;
    HAL_TIM_PWM_Start(const_h->htimx, const_h->TIM_CHANNEL_x);
}

static void arm_turn(ArmParameter* arm)
{
    if (arm->tim_current == arm->tim_setpoint) return;
    int16_t dtim = arm->tim_setpoint - arm->tim_current;
    if      (dtim >  ARM_TIM_STEP)  arm->tim_current += ARM_TIM_STEP;
    else if (dtim < -ARM_TIM_STEP)  arm->tim_current -= ARM_TIM_STEP;
    else                            arm->tim_current  = arm->tim_setpoint;
    // const ArmConst* const_h = arm->const_h;
    // if (arm->tim_current < const_h->tim_min)
    //     arm->tim_current = const_h->tim_min;
    // else if (arm->tim_current > const_h->tim_max)
    //     arm->tim_current = const_h->tim_max;
    __HAL_TIM_SET_COMPARE(arm->const_h->htimx, arm->const_h->TIM_CHANNEL_x, arm->tim_current);
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
