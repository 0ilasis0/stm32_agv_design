#include "connectivity/write_pkt.h"
#include "connectivity/cmds.h"

Result pkt_test(VecByte* vec_byte, uint32_t* value)
{
    vec_rm_all(vec_byte);
    RESULT_CHECK_RET_RES(vec_byte_push(vec_byte, (uint8_t[]){CMD_DATA_B0_CONTROL, CMD_DATA_B1_LEFT_SPEED, 0x01, 0x00}, 4));
    RESULT_CHECK_RET_RES(vec_byte_push_u32(vec_byte, *value));
    (*value)++;
    return RESULT_OK(NULL);
}

#ifdef PRINCIPAL_PROGRAM
#include "motor/main.h"

Result pkt_left_speed(VecByte* vec_byte)
{
    vec_rm_all(vec_byte);
    RESULT_CHECK_RET_RES(vec_byte_push(vec_byte, (uint8_t[]){CMD_DATA_B0_CONTROL, CMD_DATA_B1_LEFT_SPEED, 0x01, 0x00}, 4));
    RESULT_CHECK_RET_RES(vec_byte_push_f32(vec_byte, motor_left.rps_present));
    return RESULT_OK(NULL);
}

Result pkt_right_speed(VecByte* vec_byte)
{
    vec_rm_all(vec_byte);
    RESULT_CHECK_RET_RES(vec_byte_push(vec_byte, (uint8_t[]){CMD_DATA_B0_CONTROL, CMD_DATA_B1_RIGHT_SPEED, 0x01, 0x00}, 4));
    RESULT_CHECK_RET_RES(vec_byte_push_f32(vec_byte, motor_right.rps_present));
    return RESULT_OK(NULL);
}

Result pkt_left_duty(VecByte* vec_byte)
{
    vec_rm_all(vec_byte);
    RESULT_CHECK_RET_RES(vec_byte_push(vec_byte, (uint8_t[]){CMD_DATA_B0_CONTROL, CMD_DATA_B1_LEFT_DUTY, 0x01, 0x00}, 4));
    RESULT_CHECK_RET_RES(vec_byte_push_byte(vec_byte, motor_left.pwm_duty));
    return RESULT_OK(NULL);
}

Result pkt_right_duty(VecByte* vec_byte)
{
    vec_rm_all(vec_byte);
    RESULT_CHECK_RET_RES(vec_byte_push(vec_byte, (uint8_t[]){CMD_DATA_B0_CONTROL, CMD_DATA_B1_RIGHT_DUTY, 0x01, 0x00}, 4));
    RESULT_CHECK_RET_RES(vec_byte_push_byte(vec_byte, motor_right.pwm_duty));
    return RESULT_OK(NULL);
}
#endif

#ifdef ANCILLARY_PROGRAM
#include "robotic_arm/main.h"

Result pkt_arm_bottom(VecByte* vec_byte)
{
    vec_rm_all(vec_byte);
    RESULT_CHECK_RET_RES(vec_byte_push(vec_byte, (uint8_t[]){CMD_DATA_B0_CONTROL, CMD_DATA_B1_ARM_BOTTOM, 0x01, 0x00}, 4));
    RESULT_CHECK_RET_RES(vec_byte_push_byte(vec_byte, (arm_bottom.tim_current - ARM_TIM_MIN) / 2));
    return RESULT_OK(NULL);
}

Result pkt_arm_shoulder(VecByte* vec_byte)
{
    vec_rm_all(vec_byte);
    RESULT_CHECK_RET_RES(vec_byte_push(vec_byte, (uint8_t[]){CMD_DATA_B0_CONTROL, CMD_DATA_B1_ARM_SHOULDER, 0x01, 0x00}, 4));
    RESULT_CHECK_RET_RES(vec_byte_push_byte(vec_byte, (arm_shoulder.tim_current - ARM_TIM_MIN) / 2));
    return RESULT_OK(NULL);
}

Result pkt_arm_elbow_btm(VecByte* vec_byte)
{
    vec_rm_all(vec_byte);
    RESULT_CHECK_RET_RES(vec_byte_push(vec_byte, (uint8_t[]){CMD_DATA_B0_CONTROL, CMD_DATA_B1_ARM_ELBOW_BTM, 0x01, 0x00}, 4));
    RESULT_CHECK_RET_RES(vec_byte_push_byte(vec_byte, (arm_elbow_btm.tim_current - ARM_TIM_MIN) / 2));
    return RESULT_OK(NULL);
}

Result pkt_arm_elbow_top(VecByte* vec_byte)
{
    vec_rm_all(vec_byte);
    RESULT_CHECK_RET_RES(vec_byte_push(vec_byte, (uint8_t[]){CMD_DATA_B0_CONTROL, CMD_DATA_B1_ARM_ELBOW_TOP, 0x01, 0x00}, 4));
    RESULT_CHECK_RET_RES(vec_byte_push_byte(vec_byte, (arm_elbow_top.tim_current - ARM_TIM_MIN) / 2));
    return RESULT_OK(NULL);
}

Result pkt_arm_wrist(VecByte* vec_byte)
{
    vec_rm_all(vec_byte);
    RESULT_CHECK_RET_RES(vec_byte_push(vec_byte, (uint8_t[]){CMD_DATA_B0_CONTROL, CMD_DATA_B1_ARM_WRIST, 0x01, 0x00}, 4));
    RESULT_CHECK_RET_RES(vec_byte_push_byte(vec_byte, (arm_wrist.tim_current - ARM_TIM_MIN) / 2));
    return RESULT_OK(NULL);
}

Result pkt_arm_finger(VecByte* vec_byte)
{
    vec_rm_all(vec_byte);
    RESULT_CHECK_RET_RES(vec_byte_push(vec_byte, (uint8_t[]){CMD_DATA_B0_CONTROL, CMD_DATA_B1_ARM_FINGER, 0x01, 0x00}, 4));
    RESULT_CHECK_RET_RES(vec_byte_push_byte(vec_byte, (arm_finger.tim_current - ARM_TIM_MIN)));
    return RESULT_OK(NULL);
}

Result pkt_map_info(VecByte* vec_byte, uint32_t uid, uint8_t n_exist)
{
    vec_rm_all(vec_byte);
    RESULT_CHECK_RET_RES(vec_byte_push(vec_byte, (uint8_t[]){CMD_MAP_B0_CONTROL, CMD_MAP_B1_INFO}, 2));
    RESULT_CHECK_RET_RES(vec_byte_push_u32(vec_byte, uid));
    RESULT_CHECK_RET_RES(vec_byte_push_byte(vec_byte, n_exist));
    return RESULT_OK(NULL);
}

// set_mode必須最後
Result pkt_vehi_set_mode(VecByte* vec_byte, VehicleMode mode, uint8_t value)
{
    vec_rm_all(vec_byte);
    RESULT_CHECK_RET_RES(vec_byte_push(vec_byte, (uint8_t[]){CMD_VEHI_B0_CONTROL, CMD_VEHI_B1_MODE}, 2));
    switch (mode)
    {
        case VEHICLE_MODE_FREE:
        {
            RESULT_CHECK_RET_RES(vec_byte_push_byte(vec_byte, CMD_VEHI_B2_FREE));
            return RESULT_OK(NULL);
        }
        case VEHICLE_MODE_END:
        {
            RESULT_CHECK_RET_RES(vec_byte_push_byte(vec_byte, CMD_VEHI_B2_END));
            return RESULT_OK(NULL);
        }
        case VEHICLE_MODE_F_TRACK:
        {
            RESULT_CHECK_RET_RES(vec_byte_push_byte(vec_byte, CMD_VEHI_B2_F_TRACK));
            return RESULT_OK(NULL);
        }
        case VEHICLE_MODE_TRACK:
        {
            RESULT_CHECK_RET_RES(vec_byte_push_byte(vec_byte, CMD_VEHI_B2_TRACK));
            return RESULT_OK(NULL);
        }
        case VEHICLE_MODE_SEARCH:
        {
            RESULT_CHECK_RET_RES(vec_byte_push_byte(vec_byte, CMD_VEHI_B2_SEARCH));
            return RESULT_OK(NULL);
        }
        case VEHICLE_MODE_ROTATE:
        {
            RESULT_CHECK_RET_RES(vec_byte_push_byte(vec_byte, CMD_VEHI_B2_ROTATE));
            RESULT_CHECK_RET_RES(vec_byte_push_byte(vec_byte, value));
            return RESULT_OK(NULL);
        }
        default: break;
    }
    return RESULT_ERROR(RES_ERR_FAIL);
}

Result pkt_vehi_set_motion(VecByte* vec_byte, VehicleMotion motion)
{
    vec_rm_all(vec_byte);
    RESULT_CHECK_RET_RES(vec_byte_push(vec_byte, (uint8_t[]){CMD_VEHI_B0_CONTROL, CMD_VEHI_B1_MOTION}, 2));
    switch (motion)
    {
        case VEHICLE_MOTION_STOP:
        {
            RESULT_CHECK_RET_RES(vec_byte_push_byte(vec_byte, CMD_VEHI_B2_FREE));
            return RESULT_OK(NULL);
        }
        case VEHICLE_MOTION_FORWARD:
        {
            RESULT_CHECK_RET_RES(vec_byte_push_byte(vec_byte, CMD_VEHI_B2_END));
            return RESULT_OK(NULL);
        }
        case VEHICLE_MOTION_BACKWARD:
        {
            RESULT_CHECK_RET_RES(vec_byte_push_byte(vec_byte, CMD_VEHI_B2_F_TRACK));
            return RESULT_OK(NULL);
        }
        case VEHICLE_MOTION_CLOCKWISE:
        {
            RESULT_CHECK_RET_RES(vec_byte_push_byte(vec_byte, CMD_VEHI_B2_SEARCH));
            return RESULT_OK(NULL);
        }
        case VEHICLE_MOTION_C_CLOCKWISE:
        {
            RESULT_CHECK_RET_RES(vec_byte_push_byte(vec_byte, CMD_VEHI_B2_TRACK));
            return RESULT_OK(NULL);
        }
        default: break;
    }
    return RESULT_ERROR(RES_ERR_FAIL);
}

Result pkt_vehi_set_speed(VecByte* vec_byte, Percentage value)
{
    vec_rm_all(vec_byte);
    RESULT_CHECK_RET_RES(vec_byte_push(vec_byte, (uint8_t[]){CMD_VEHI_B0_CONTROL, CMD_VEHI_B1_SPEED}, 2));
    RESULT_CHECK_RET_RES(vec_byte_push_byte(vec_byte, value));
    return RESULT_OK(NULL);
}
#endif
