#pragma once

#include "main/config.h"

typedef uint8_t VehicleDirect;
#define VEHICLE_DIRECT_STOP         0
#define VEHICLE_DIRECT_FORWARD      1
#define VEHICLE_DIRECT_BACKWARD     2
#define VEHICLE_DIRECT_CLOCKWISE    3
#define VEHICLE_DIRECT_C_CLOCKWISE  4

typedef uint8_t VehicleMode;
#define VEHICLE_MODE_FREE       0
#define VEHICLE_MODE_TRACK      1
#define VEHICLE_MODE_SEARCH     2
