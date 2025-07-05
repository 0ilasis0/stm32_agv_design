#include "connectivity/write_pkt.h"
#include "connectivity/cmds.h"

#ifdef PRINCIPAL_PROGRAM
#include "motor/main.h"
#endif

FnState pkt_test(VecByte* vec_byte, uint32_t* value)
{
    vec_rm_all(vec_byte);
    ERROR_CHECK_FNS_RETURN(vec_byte_push(vec_byte, (uint8_t[]){CMD_B0_DATA, CMD_B1_LEFT_SPEED, 0x01, 0x00}, 4));
    ERROR_CHECK_FNS_RETURN(vec_byte_push_u32(vec_byte, *value));
    (*value)++;
    return FNS_OK;
}

#ifdef PRINCIPAL_PROGRAM
FnState pkt_left_speed(VecByte* vec_byte)
{
    vec_rm_all(vec_byte);
    ERROR_CHECK_FNS_RETURN(vec_byte_push(vec_byte, (uint8_t[]){CMD_B0_DATA, CMD_B1_LEFT_SPEED, 0x01, 0x00}, 4));
    ERROR_CHECK_FNS_RETURN(vec_byte_push_f32(vec_byte, motor_left.rps_present));
    return FNS_OK;
}

FnState pkt_right_speed(VecByte* vec_byte)
{
    vec_rm_all(vec_byte);
    ERROR_CHECK_FNS_RETURN(vec_byte_push(vec_byte, (uint8_t[]){CMD_B0_DATA, CMD_B1_RIGHT_SPEED, 0x01, 0x00}, 4));
    ERROR_CHECK_FNS_RETURN(vec_byte_push_f32(vec_byte, motor_right.rps_present));
    return FNS_OK;
}

FnState pkt_left_duty(VecByte* vec_byte)
{
    vec_rm_all(vec_byte);
    ERROR_CHECK_FNS_RETURN(vec_byte_push(vec_byte, (uint8_t[]){CMD_B0_DATA, CMD_B1_LEFT_DUTY, 0x01, 0x00}, 4));
    ERROR_CHECK_FNS_RETURN(vec_byte_push_byte(vec_byte, motor_left.duty));
    return FNS_OK;
}

FnState pkt_right_duty(VecByte* vec_byte)
{
    vec_rm_all(vec_byte);
    ERROR_CHECK_FNS_RETURN(vec_byte_push(vec_byte, (uint8_t[]){CMD_B0_DATA, CMD_B1_RIGHT_DUTY, 0x01, 0x00}, 4));
    ERROR_CHECK_FNS_RETURN(vec_byte_push_byte(vec_byte, motor_right.duty));
    return FNS_OK;
}
#endif
