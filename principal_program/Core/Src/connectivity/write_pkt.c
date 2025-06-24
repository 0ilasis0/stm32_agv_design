#include "connectivity/write_pkt.h"
#include "connectivity/cmds.h"
#include "main/config.h"
#include "main/vec.h"
#include "motor/main.h"

float f32_test = 0;
FnState pkt_left_speed(ByteTrcvBuf* trcv_buf)
{
    VecByte vec_byte;
    ERROR_CHECK_FNS_RETURN(vec_byte_new(&vec_byte, 8));
    ERROR_CHECK_FNS_CLEAN(vec_byte_push_byte(&vec_byte, CMD_B0_DATA), vec_byte_free(&vec_byte));
    ERROR_CHECK_FNS_CLEAN(vec_byte_push_byte(&vec_byte, CMD_B1_LEFT_SPEED), vec_byte_free(&vec_byte));
    ERROR_CHECK_FNS_CLEAN(vec_byte_push_byte(&vec_byte, 0x01), vec_byte_free(&vec_byte));
    ERROR_CHECK_FNS_CLEAN(vec_byte_push_byte(&vec_byte, 0x00), vec_byte_free(&vec_byte));
    // ERROR_CHECK_FNS_CLEAN(vec_byte_push_f32(&vec_byte, motor_left.speed_present), vec_byte_free(&vec_byte));
    ERROR_CHECK_FNS_CLEAN(vec_byte_push_f32(&vec_byte, f32_test), vec_byte_free(&vec_byte));
    f32_test+=0.001;
    ERROR_CHECK_FNS_CLEAN(connect_trcv_buf_push(trcv_buf, &vec_byte), vec_byte_free(&vec_byte));
    vec_byte_free(&vec_byte);
    return FNS_OK;
}

FnState pkt_right_speed(ByteTrcvBuf* trcv_buf)
{
    VecByte vec_byte;
    ERROR_CHECK_FNS_RETURN(vec_byte_new(&vec_byte, 8));
    ERROR_CHECK_FNS_CLEAN(vec_byte_push_byte(&vec_byte, CMD_B0_DATA), vec_byte_free(&vec_byte));
    ERROR_CHECK_FNS_CLEAN(vec_byte_push_byte(&vec_byte, CMD_B1_RIGHT_SPEED), vec_byte_free(&vec_byte));
    ERROR_CHECK_FNS_CLEAN(vec_byte_push_byte(&vec_byte, 0x01), vec_byte_free(&vec_byte));
    ERROR_CHECK_FNS_CLEAN(vec_byte_push_byte(&vec_byte, 0x00), vec_byte_free(&vec_byte));
    ERROR_CHECK_FNS_CLEAN(vec_byte_push_f32(&vec_byte, motor_right.speed_present), vec_byte_free(&vec_byte));
    ERROR_CHECK_FNS_CLEAN(connect_trcv_buf_push(trcv_buf, &vec_byte), vec_byte_free(&vec_byte));
    vec_byte_free(&vec_byte);
    return FNS_OK;
}

FnState pkt_left_duty(ByteTrcvBuf* trcv_buf)
{
    VecByte vec_byte;
    ERROR_CHECK_FNS_RETURN(vec_byte_new(&vec_byte, 8));
    ERROR_CHECK_FNS_CLEAN(vec_byte_push_byte(&vec_byte, CMD_B0_DATA), vec_byte_free(&vec_byte));
    ERROR_CHECK_FNS_CLEAN(vec_byte_push_byte(&vec_byte, CMD_B1_LEFT_DUTY), vec_byte_free(&vec_byte));
    ERROR_CHECK_FNS_CLEAN(vec_byte_push_byte(&vec_byte, 0x01), vec_byte_free(&vec_byte));
    ERROR_CHECK_FNS_CLEAN(vec_byte_push_byte(&vec_byte, 0x00), vec_byte_free(&vec_byte));
    ERROR_CHECK_FNS_CLEAN(vec_byte_push_byte(&vec_byte, motor_left.duty_value), vec_byte_free(&vec_byte));
    ERROR_CHECK_FNS_CLEAN(connect_trcv_buf_push(trcv_buf, &vec_byte), vec_byte_free(&vec_byte));
    vec_byte_free(&vec_byte);
    return FNS_OK;
}

FnState pkt_right_duty(ByteTrcvBuf* trcv_buf)
{
    VecByte vec_byte;
    ERROR_CHECK_FNS_RETURN(vec_byte_new(&vec_byte, 8));
    ERROR_CHECK_FNS_CLEAN(vec_byte_push_byte(&vec_byte, CMD_B0_DATA), vec_byte_free(&vec_byte));
    ERROR_CHECK_FNS_CLEAN(vec_byte_push_byte(&vec_byte, CMD_B1_RIGHT_DUTY), vec_byte_free(&vec_byte));
    ERROR_CHECK_FNS_CLEAN(vec_byte_push_byte(&vec_byte, 0x01), vec_byte_free(&vec_byte));
    ERROR_CHECK_FNS_CLEAN(vec_byte_push_byte(&vec_byte, 0x00), vec_byte_free(&vec_byte));
    ERROR_CHECK_FNS_CLEAN(vec_byte_push_byte(&vec_byte, motor_right.duty_value), vec_byte_free(&vec_byte));
    ERROR_CHECK_FNS_CLEAN(connect_trcv_buf_push(trcv_buf, &vec_byte), vec_byte_free(&vec_byte));
    vec_byte_free(&vec_byte);
    return FNS_OK;
}
