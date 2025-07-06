#pragma once

#include "main/config.h"
#include "main/fn_state.h"
#include "motor/main.h"

typedef enum {
    motion_unchange,
    motion_forward,
    motion_backward,
    motion_clockwise,
    motion_c_clockwise,
    motion_stop,
} MotionCommand;

typedef struct VehicleState
{
    MotionCommand motion_present;
} VehicleState;

void vehicle_ensure_stop(void);
void vehicle_set_motion(MotionCommand mode);
void vehicle_set_speed(uint8_t value);
