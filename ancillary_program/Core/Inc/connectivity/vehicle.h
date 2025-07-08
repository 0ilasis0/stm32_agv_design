#pragma once

#include "main/config.h"

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
