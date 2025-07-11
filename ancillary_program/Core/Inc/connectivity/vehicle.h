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
#define VEHICLE_MODE_ROTATE     3
#define VEHICLE_MODE_END        4
#define VEHICLE_MODE_IDLE       5

// typedef enum
// {
//     VEHICLE_DIRECT_STOP,
//     VEHICLE_DIRECT_FORWARD,
//     VEHICLE_DIRECT_BACKWARD,
//     VEHICLE_DIRECT_CLOCKWISE,
//     VEHICLE_DIRECT_C_CLOCKWISE,
//     VEHICLE_DIRECT_UNKNOWN = -1,
// } VehicleDirect;

// typedef enum
// {
//     VEHICLE_MODE_FREE,
//     VEHICLE_MODE_TRACK,
//     VEHICLE_MODE_SEARCH,
//     VEHICLE_MODE_ROTATE,
//     VEHICLE_MODE_END,
//     VEHICLE_MODE_IDLE,
// } VehicleMode;

typedef uint8_t VehicleMode;
#define VEHICLE_MODE_FREE       0
#define VEHICLE_MODE_TRACK      1
#define VEHICLE_MODE_SEARCH     2
