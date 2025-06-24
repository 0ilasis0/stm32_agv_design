#include "robotic_arm/main.h"
#include "cmsis_os.h"

static const ArmConst arm_bottom_const = {
    .TIMx = &htim2,
    .TIM_CHANNEL_x = TIM_CHANNEL_1,
};
ArmParameter arm_bottom = {
    .arm_const = &arm_bottom_const,
};

static const ArmConst arm_shoulder_const = {
    .TIMx = &htim2,
    .TIM_CHANNEL_x = TIM_CHANNEL_2,
};
ArmParameter arm_shoulder = {
    .arm_const = &arm_shoulder_const,
};

static const ArmConst arm_elbow_btm_const = {
    .TIMx = &htim2,
    .TIM_CHANNEL_x = TIM_CHANNEL_3,
};
ArmParameter arm_elbow_btm = {
    .arm_const = &arm_elbow_btm_const,
};

static const ArmConst arm_elbow_top_const = {
    .TIMx = &htim3,
    .TIM_CHANNEL_x = TIM_CHANNEL_1,
};
ArmParameter arm_elbow_top = {
    .arm_const = &arm_elbow_top_const,
};

static const ArmConst arm_wrist_const = {
    .TIMx = &htim3,
    .TIM_CHANNEL_x = TIM_CHANNEL_2,
};
ArmParameter arm_wrist = {
    .arm_const = &arm_wrist_const,
};

static const ArmConst arm_finger_const = {
    .TIMx = &htim3,
    .TIM_CHANNEL_x = TIM_CHANNEL_3,
};
ArmParameter arm_finger = {
    .arm_const = &arm_finger_const,
};

void arm_set_pos(ArmParameter* arm, ArmPosition pos)
{
    if (pos < ARM_POS_000) pos = ARM_POS_000;
    else if (pos > ARM_POS_100) pos = ARM_POS_100;
    arm->pos_setpoint = pos;
}

static void arm_setup(ArmParameter* arm)
{
    const ArmConst* arm_const = arm->arm_const;
    HAL_TIM_PWM_Start(arm_const->TIMx, arm_const->TIM_CHANNEL_x);
}

static void arm_turn(ArmParameter* arm)
{
    if (arm->pos_current == arm->pos_setpoint) return;
    if (arm->pos_current < arm->pos_setpoint)
    {
        arm->pos_current += ARM_POS_STEP;
    }
    else
    {
        arm->pos_current -= ARM_POS_STEP;
    }
    __HAL_TIM_SET_COMPARE(arm->arm_const->TIMx, arm->arm_const->TIM_CHANNEL_x, arm->pos_current);
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
