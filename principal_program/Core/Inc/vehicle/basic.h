#pragma once

#include "main/config.h"
#include "main/fn_state.h"
#include "motor/main.h"

typedef enum {
    motion_forward,
    motion_backward,
    motion_clockwise,
    motion_c_clockwise,
    motion_stop,
} VehicleMotion;

typedef uint8_t VehicleMode;
#define VEHICLE_MODE_FREE       0
#define VEHICLE_MODE_TRACK      1
#define VEHICLE_MODE_SEARCH     2

typedef struct VehicleState
{
    Percentage speed;
    VehicleMode mode;
    VehicleMode mode_inner;
    VehicleMotion motion;
    VehicleMotion motion_inner;
} VehicleState;

extern VehicleState vehicle_state;

void vehicle_ensure_stop_inner(void);
void vehicle_ensure_stop(void);
void vehicle_set_mode(VehicleMode mode);
void vehicle_set_motion(VehicleMotion mode);
void vehicle_set_speed(Percentage value);
