#pragma once

#include <stdint.h>
#include "motor/main.h"
#include "adc.h"

#define HALL_MAGNITUTE_EDGE 1870  //1730

// ? add current
typedef enum {
    motion_unchange,
    motion_forward,
    motion_backward,
    motion_clockwise,
    motion_c_clockwise
} MotionCommand;

extern MotionCommand direction_mode;

void vehicle2_motion_and_speed_control(MotionCommand mode, uint8_t sepoint_value);
void vehicle2_ensure_motor_stop(void);
MotionCommand vehicle2_get_rotate_direction(int8_t start_dir, int8_t end_dir);
void vehicle2_renew_vehicle_rotation_status (uint8_t count_until_zero);
uint8_t vehicle2_pass_magnetic_stripe_calculate(
    MotionCommand rotate_direction_mode,
    uint16_t current_id_input,
    uint8_t from_dir,
    uint8_t to_dir
);
