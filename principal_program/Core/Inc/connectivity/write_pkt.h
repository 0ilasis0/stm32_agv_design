#pragma once

#include "main/config.h"
#include "main/fn_state.h"
#include "main/vec.h"

Result pkt_test(VecByte* vec_byte, uint32_t* value);
#ifdef PRINCIPAL_PROGRAM
Result pkt_left_speed(VecByte* vec_byte);
Result pkt_right_speed(VecByte* vec_byte);
Result pkt_left_duty(VecByte* vec_byte);
Result pkt_right_duty(VecByte* vec_byte);
#endif

#ifdef ANCILLARY_PROGRAM
#include "main/vehicle.h"
Result pkt_arm_bottom(VecByte* vec_byte);
Result pkt_arm_shoulder(VecByte* vec_byte);
Result pkt_arm_elbow_btm(VecByte* vec_byte);
Result pkt_arm_elbow_top(VecByte* vec_byte);
Result pkt_arm_wrist(VecByte* vec_byte);
Result pkt_arm_finger(VecByte* vec_byte);
Result pkt_map_info(VecByte* vec_byte, uint32_t uid, uint8_t n_exist);
Result pkt_vehi_set_mode(VecByte* vec_byte, VehicleMode mode, uint8_t value);
Result pkt_vehi_set_motion(VecByte* vec_byte, VehicleMotion motion);
Result pkt_vehi_set_speed(VecByte* vec_byte, Percentage value);
#endif
