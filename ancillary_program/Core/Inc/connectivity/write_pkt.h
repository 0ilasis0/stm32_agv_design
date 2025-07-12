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

#ifdef ANCILLARY_PROGRAM
#include "main/vehicle.h"
FnState pkt_arm_bottom(VecByte* vec_byte);
FnState pkt_arm_shoulder(VecByte* vec_byte);
FnState pkt_arm_elbow_btm(VecByte* vec_byte);
FnState pkt_arm_elbow_top(VecByte* vec_byte);
FnState pkt_arm_wrist(VecByte* vec_byte);
FnState pkt_arm_finger(VecByte* vec_byte);
FnState pkt_vehi_set_mode(VecByte* vec_byte, VehicleMode mode, uint8_t value);
FnState pkt_vehi_set_direct(VecByte* vec_byte, VehicleMotion direction);
FnState pkt_vehi_set_speed(VecByte* vec_byte, Percentage value);
#endif
