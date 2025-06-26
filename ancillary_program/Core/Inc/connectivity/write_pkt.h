#pragma once

#include "main/config.h"
#include "main/fn_state.h"
#include "main/vec.h"

FnState pkt_test(VecByte* vec_byte, uint32_t* value);
#ifdef PRINCIPAL_PROGRAM
FnState pkt_left_speed(VecByte* vec_byte);
FnState pkt_right_speed(VecByte* vec_byte);
FnState pkt_left_duty(VecByte* vec_byte);
FnState pkt_right_duty(VecByte* vec_byte);
#endif
