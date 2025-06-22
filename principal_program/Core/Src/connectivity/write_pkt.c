#include "connectivity/write_pkt.h"
#include "main/mcu_const.h"
#include "main/config.h"
#include "main/vec.h"
#include "motor/main.h"

float f32_test = 0;
FnState pkt_left_speed(ByteTrcvBuf* trcv_buf)
{
    VecByte vec_u8;
    FNS_ERROR_CHECK(vec_byte_new(&vec_u8, 8));
    FNS_ERROR_CHECK_CLEAN(vec_byte_push_byte(&vec_u8, CMD_B0_DATA), vec_byte_free(&vec_u8));
    FNS_ERROR_CHECK_CLEAN(vec_byte_push_byte(&vec_u8, CMD_B1_LEFT_SPEED), vec_byte_free(&vec_u8));
    FNS_ERROR_CHECK_CLEAN(vec_byte_push_byte(&vec_u8, CMD_BX_EMPTY), vec_byte_free(&vec_u8));
    FNS_ERROR_CHECK_CLEAN(vec_byte_push_byte(&vec_u8, CMD_BX_EMPTY), vec_byte_free(&vec_u8));
    // FNS_ERROR_CHECK_CLEAN(vec_byte_push_f32(&vec_u8, motor_left.speed_present), vec_byte_free(&vec_u8));
    FNS_ERROR_CHECK_CLEAN(vec_byte_push_f32(&vec_u8, f32_test), vec_byte_free(&vec_u8));
    FNS_ERROR_CHECK_CLEAN(connect_trcv_buf_push(trcv_buf, &vec_u8), vec_byte_free(&vec_u8));
    vec_byte_free(&vec_u8);
    return FNS_OK;
}

FnState pkt_right_speed(ByteTrcvBuf* trcv_buf)
{
    f32_test++;
    VecByte vec_u8;
    FNS_ERROR_CHECK(vec_byte_new(&vec_u8, 8));
    FNS_ERROR_CHECK_CLEAN(vec_byte_push_byte(&vec_u8, CMD_B0_DATA), vec_byte_free(&vec_u8));
    FNS_ERROR_CHECK_CLEAN(vec_byte_push_byte(&vec_u8, CMD_B1_RIGHT_SPEED), vec_byte_free(&vec_u8));
    FNS_ERROR_CHECK_CLEAN(vec_byte_push_byte(&vec_u8, CMD_BX_EMPTY), vec_byte_free(&vec_u8));
    FNS_ERROR_CHECK_CLEAN(vec_byte_push_byte(&vec_u8, CMD_BX_EMPTY), vec_byte_free(&vec_u8));
    FNS_ERROR_CHECK_CLEAN(vec_byte_push_f32(&vec_u8, motor_right.speed_present), vec_byte_free(&vec_u8));
    FNS_ERROR_CHECK_CLEAN(connect_trcv_buf_push(trcv_buf, &vec_u8), vec_byte_free(&vec_u8));
    vec_byte_free(&vec_u8);
    return FNS_OK;
}

FnState pkt_left_duty(ByteTrcvBuf* trcv_buf)
{
    VecByte vec_u8;
    FNS_ERROR_CHECK(vec_byte_new(&vec_u8, 8));
    FNS_ERROR_CHECK_CLEAN(vec_byte_push_byte(&vec_u8, CMD_B0_DATA), vec_byte_free(&vec_u8));
    FNS_ERROR_CHECK_CLEAN(vec_byte_push_byte(&vec_u8, CMD_B1_LEFT_DUTY), vec_byte_free(&vec_u8));
    FNS_ERROR_CHECK_CLEAN(vec_byte_push_byte(&vec_u8, CMD_BX_EMPTY), vec_byte_free(&vec_u8));
    FNS_ERROR_CHECK_CLEAN(vec_byte_push_byte(&vec_u8, CMD_BX_EMPTY), vec_byte_free(&vec_u8));
    FNS_ERROR_CHECK_CLEAN(vec_byte_push_byte(&vec_u8, motor_left.duty_value), vec_byte_free(&vec_u8));
    FNS_ERROR_CHECK_CLEAN(connect_trcv_buf_push(trcv_buf, &vec_u8), vec_byte_free(&vec_u8));
    vec_byte_free(&vec_u8);
    return FNS_OK;
}

FnState pkt_right_duty(ByteTrcvBuf* trcv_buf)
{
    VecByte vec_u8;
    FNS_ERROR_CHECK(vec_byte_new(&vec_u8, 8));
    FNS_ERROR_CHECK_CLEAN(vec_byte_push_byte(&vec_u8, CMD_B0_DATA), vec_byte_free(&vec_u8));
    FNS_ERROR_CHECK_CLEAN(vec_byte_push_byte(&vec_u8, CMD_B1_RIGHT_DUTY), vec_byte_free(&vec_u8));
    FNS_ERROR_CHECK_CLEAN(vec_byte_push_byte(&vec_u8, CMD_BX_EMPTY), vec_byte_free(&vec_u8));
    FNS_ERROR_CHECK_CLEAN(vec_byte_push_byte(&vec_u8, CMD_BX_EMPTY), vec_byte_free(&vec_u8));
    FNS_ERROR_CHECK_CLEAN(vec_byte_push_byte(&vec_u8, motor_right.duty_value), vec_byte_free(&vec_u8));
    FNS_ERROR_CHECK_CLEAN(connect_trcv_buf_push(trcv_buf, &vec_u8), vec_byte_free(&vec_u8));
    vec_byte_free(&vec_u8);
    return FNS_OK;
}
