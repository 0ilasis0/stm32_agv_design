#include "connectivity/write_pkt.h"
#include "connectivity/cmds.h"
#include "main/config.h"
#include "main/vec.h"
#include "motor/main.h"

uint32_t write_pkt_0 = 0;
FnState pkt_test(VecByte* vec_byte)
{
    vec_rm_all(vec_byte);
    vec_byte_push(vec_byte, (uint8_t[]){CMD_B0_DATA, CMD_B1_LEFT_SPEED, 0x01, 0x00}, 4);
    vec_byte_push_u32(vec_byte, write_pkt_0);
    write_pkt_0+=256;
    return FNS_OK;
}

FnState pkt_left_speed(VecByte* vec_byte)
{
    vec_rm_all(vec_byte);
    vec_byte_push(vec_byte, (uint8_t[]){CMD_B0_DATA, CMD_B1_LEFT_SPEED, 0x01, 0x00}, 4);
    vec_byte_push_f32(vec_byte, motor_left.speed_present);
    return FNS_OK;
}

FnState pkt_right_speed(VecByte* vec_byte)
{
    vec_rm_all(vec_byte);
    vec_byte_push(vec_byte, (uint8_t[]){CMD_B0_DATA, CMD_B1_RIGHT_SPEED, 0x01, 0x00}, 4);
    vec_byte_push_f32(vec_byte, motor_right.speed_present);
    return FNS_OK;
}

FnState pkt_left_duty(VecByte* vec_byte)
{
    vec_rm_all(vec_byte);
    vec_byte_push(vec_byte, (uint8_t[]){CMD_B0_DATA, CMD_B1_LEFT_DUTY, 0x01, 0x00}, 4);
    vec_byte_push(vec_byte, (uint8_t[]){0x00, 0x00, 0x00}, 3);
    vec_byte_push_byte(vec_byte, motor_left.duty_value);
    return FNS_OK;
}

FnState pkt_right_duty(VecByte* vec_byte)
{
    vec_rm_all(vec_byte);
    vec_byte_push(vec_byte, (uint8_t[]){CMD_B0_DATA, CMD_B1_RIGHT_DUTY, 0x01, 0x00}, 4);
    vec_byte_push(vec_byte, (uint8_t[]){0x00, 0x00, 0x00}, 3);
    vec_byte_push_byte(vec_byte, motor_right.duty_value);
    return FNS_OK;
}
