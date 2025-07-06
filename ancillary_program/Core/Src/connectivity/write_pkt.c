#include "connectivity/write_pkt.h"
#include "connectivity/cmds.h"

FnState pkt_test(VecByte* vec_byte, uint32_t* value)
{
    vec_rm_all(vec_byte);
    ERROR_CHECK_FNS_RETURN(vec_byte_push(vec_byte, (uint8_t[]){CMD_DATA_B0, CMD_DATA_B1_LEFT_SPEED, 0x01, 0x00}, 4));
    ERROR_CHECK_FNS_RETURN(vec_byte_push_u32(vec_byte, *value));
    (*value)++;
    return FNS_OK;
}

#ifdef PRINCIPAL_PROGRAM
#include "motor/main.h"

FnState pkt_left_speed(VecByte* vec_byte)
{
    vec_rm_all(vec_byte);
    ERROR_CHECK_FNS_RETURN(vec_byte_push(vec_byte, (uint8_t[]){CMD_DATA_B0, CMD_DATA_B1_LEFT_SPEED, 0x01, 0x00}, 4));
    ERROR_CHECK_FNS_RETURN(vec_byte_push_f32(vec_byte, motor_left.rps_present));
    return FNS_OK;
}

FnState pkt_right_speed(VecByte* vec_byte)
{
    vec_rm_all(vec_byte);
    ERROR_CHECK_FNS_RETURN(vec_byte_push(vec_byte, (uint8_t[]){CMD_DATA_B0, CMD_DATA_B1_RIGHT_SPEED, 0x01, 0x00}, 4));
    ERROR_CHECK_FNS_RETURN(vec_byte_push_f32(vec_byte, motor_right.rps_present));
    return FNS_OK;
}

FnState pkt_left_duty(VecByte* vec_byte)
{
    vec_rm_all(vec_byte);
    ERROR_CHECK_FNS_RETURN(vec_byte_push(vec_byte, (uint8_t[]){CMD_DATA_B0, CMD_DATA_B1_LEFT_DUTY, 0x01, 0x00}, 4));
    ERROR_CHECK_FNS_RETURN(vec_byte_push_byte(vec_byte, motor_left.duty));
    return FNS_OK;
}

FnState pkt_right_duty(VecByte* vec_byte)
{
    vec_rm_all(vec_byte);
    ERROR_CHECK_FNS_RETURN(vec_byte_push(vec_byte, (uint8_t[]){CMD_DATA_B0, CMD_DATA_B1_RIGHT_DUTY, 0x01, 0x00}, 4));
    ERROR_CHECK_FNS_RETURN(vec_byte_push_byte(vec_byte, motor_right.duty));
    return FNS_OK;
}
#endif

#ifdef ANCILLARY_PROGRAM
#include "robotic_arm/main.h"

FnState pkt_arm_bottom(VecByte* vec_byte)
{
    vec_rm_all(vec_byte);
    ERROR_CHECK_FNS_RETURN(vec_byte_push(vec_byte, (uint8_t[]){CMD_DATA_B0, CMD_DATA_B1_ARM_BOTTOM, 0x01, 0x00}, 4));
    ERROR_CHECK_FNS_RETURN(vec_byte_push_byte(vec_byte, (arm_bottom.tim_current - ARM_TIM_MIN) / 2));
    return FNS_OK;
}

FnState pkt_arm_shoulder(VecByte* vec_byte)
{
    vec_rm_all(vec_byte);
    ERROR_CHECK_FNS_RETURN(vec_byte_push(vec_byte, (uint8_t[]){CMD_DATA_B0, CMD_DATA_B1_ARM_SHOULDER, 0x01, 0x00}, 4));
    ERROR_CHECK_FNS_RETURN(vec_byte_push_byte(vec_byte, (arm_shoulder.tim_current - ARM_TIM_MIN) / 2));
    return FNS_OK;
}

FnState pkt_arm_elbow_btm(VecByte* vec_byte)
{
    vec_rm_all(vec_byte);
    ERROR_CHECK_FNS_RETURN(vec_byte_push(vec_byte, (uint8_t[]){CMD_DATA_B0, CMD_DATA_B1_ARM_ELBOW_BTM, 0x01, 0x00}, 4));
    ERROR_CHECK_FNS_RETURN(vec_byte_push_byte(vec_byte, (arm_elbow_btm.tim_current - ARM_TIM_MIN) / 2));
    return FNS_OK;
}

FnState pkt_arm_elbow_top(VecByte* vec_byte)
{
    vec_rm_all(vec_byte);
    ERROR_CHECK_FNS_RETURN(vec_byte_push(vec_byte, (uint8_t[]){CMD_DATA_B0, CMD_DATA_B1_ARM_ELBOW_TOP, 0x01, 0x00}, 4));
    ERROR_CHECK_FNS_RETURN(vec_byte_push_byte(vec_byte, (arm_elbow_top.tim_current - ARM_TIM_MIN) / 2));
    return FNS_OK;
}

FnState pkt_arm_wrist(VecByte* vec_byte)
{
    vec_rm_all(vec_byte);
    ERROR_CHECK_FNS_RETURN(vec_byte_push(vec_byte, (uint8_t[]){CMD_DATA_B0, CMD_DATA_B1_ARM_WRIST, 0x01, 0x00}, 4));
    ERROR_CHECK_FNS_RETURN(vec_byte_push_byte(vec_byte, (arm_wrist.tim_current - ARM_TIM_MIN) / 2));
    return FNS_OK;
}

FnState pkt_arm_finger(VecByte* vec_byte)
{
    vec_rm_all(vec_byte);
    ERROR_CHECK_FNS_RETURN(vec_byte_push(vec_byte, (uint8_t[]){CMD_DATA_B0, CMD_DATA_B1_ARM_FINGER, 0x01, 0x00}, 4));
    ERROR_CHECK_FNS_RETURN(vec_byte_push_byte(vec_byte, (arm_finger.tim_current - ARM_TIM_MIN)));
    return FNS_OK;
}
#endif
