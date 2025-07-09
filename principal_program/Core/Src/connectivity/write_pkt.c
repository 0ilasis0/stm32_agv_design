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
    ERROR_CHECK_FNS_RETURN(vec_byte_push_byte(vec_byte, motor_left.pwm_duty));
    return FNS_OK;
}

FnState pkt_right_duty(VecByte* vec_byte)
{
    vec_rm_all(vec_byte);
    ERROR_CHECK_FNS_RETURN(vec_byte_push(vec_byte, (uint8_t[]){CMD_DATA_B0, CMD_DATA_B1_RIGHT_DUTY, 0x01, 0x00}, 4));
    ERROR_CHECK_FNS_RETURN(vec_byte_push_byte(vec_byte, motor_right.pwm_duty));
    return FNS_OK;
}
#endif

#ifdef ANCILLARY_PROGRAM
#include "connectivity/vehicle.h"
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

FnState pkt_vehi_set_mode(VecByte* vec_byte, VehicleMode mode)
{
    vec_rm_all(vec_byte);
    ERROR_CHECK_FNS_RETURN(vec_byte_push(vec_byte, (uint8_t[]){CMD_VEHI_B0_CONTROL, CMD_VEHI_B1_VEHICLE, CMD_VEHI_B2_MODE}, 3));
    switch (mode)
    {
        case VEHICLE_MODE_FREE:
        {
            ERROR_CHECK_FNS_RETURN(vec_byte_push_byte(vec_byte, CMD_VEHI_B3_FREE));
            return FNS_OK;
        }
        case VEHICLE_MODE_TRACK:
        {
            ERROR_CHECK_FNS_RETURN(vec_byte_push_byte(vec_byte, CMD_VEHI_B3_TRACK));
            return FNS_OK;
        }
        case VEHICLE_MODE_SEARCH:
        {
            ERROR_CHECK_FNS_RETURN(vec_byte_push_byte(vec_byte, CMD_VEHI_B3_SEARCH));
            return FNS_OK;
        }
        default: break;
    }
    return FNS_FAIL;
}

FnState pkt_vehi_set_direct(VecByte* vec_byte, VehicleDirect direction, Percentage value)
{
    vec_rm_all(vec_byte);
    ERROR_CHECK_FNS_RETURN(vec_byte_push(vec_byte, (uint8_t[]){CMD_VEHI_B0_CONTROL, CMD_VEHI_B1_VEHICLE}, 2));
    switch (direction)
    {
        case VEHICLE_DIRECT_STOP:
        {
            ERROR_CHECK_FNS_RETURN(vec_byte_push_byte(vec_byte, CMD_VEHI_B2_STOP));
            ERROR_CHECK_FNS_RETURN(vec_byte_push_byte(vec_byte, value));
            return FNS_OK;
        }
        case VEHICLE_DIRECT_FORWARD:
        {
            ERROR_CHECK_FNS_RETURN(vec_byte_push_byte(vec_byte, CMD_VEHI_B2_FOWARD));
            ERROR_CHECK_FNS_RETURN(vec_byte_push_byte(vec_byte, value));
            return FNS_OK;
        }
        case VEHICLE_DIRECT_BACKWARD:
        {
            ERROR_CHECK_FNS_RETURN(vec_byte_push_byte(vec_byte, CMD_VEHI_B2_BACKWARD));
            ERROR_CHECK_FNS_RETURN(vec_byte_push_byte(vec_byte, value));
            return FNS_OK;
        }
        case VEHICLE_DIRECT_CLOCKWISE:
        {
            ERROR_CHECK_FNS_RETURN(vec_byte_push_byte(vec_byte, CMD_VEHI_B2_CLOCK));
            ERROR_CHECK_FNS_RETURN(vec_byte_push_byte(vec_byte, value));
            return FNS_OK;
        }
        case VEHICLE_DIRECT_C_CLOCKWISE:
        {
            ERROR_CHECK_FNS_RETURN(vec_byte_push_byte(vec_byte, CMD_VEHI_B2_C_CLOCK));
            ERROR_CHECK_FNS_RETURN(vec_byte_push_byte(vec_byte, value));
            return FNS_OK;
        }
        default: break;
    }
    return FNS_FAIL;
}

// FnState pkt_vehi_set_speed(VecByte* vec_byte, Percentage value)
// {
//     if (value > 100) value = 100;
//     ERROR_CHECK_FNS_RETURN(vec_byte_push(vec_byte, (uint8_t[]){CMD_VEHI_B0_CONTROL, CMD_VEHI_B1_VEHICLE}, 2));
// }

#endif
