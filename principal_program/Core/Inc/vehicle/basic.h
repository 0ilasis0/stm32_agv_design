#pragma once

#include "main/config.h"
#include "main/fn_state.h"
#include "motor/main.h"

// typedef uint8_t VehicleDirect;

typedef enum
{
    VEHICLE_DIRECT_STOP,
    VEHICLE_DIRECT_FORWARD,
    VEHICLE_DIRECT_BACKWARD,
    VEHICLE_DIRECT_CLOCKWISE,
    VEHICLE_DIRECT_C_CLOCKWISE,
    VEHICLE_DIRECT_UNKNOWN = -1,
} VehicleDirect;

// typedef uint8_t VehicleMode;
typedef enum
{
    VEHICLE_MODE_FREE,
    VEHICLE_MODE_TRACK,
    VEHICLE_MODE_SEARCH,
    VEHICLE_MODE_ROTATE,
    VEHICLE_MODE_END,
    VEHICLE_MODE_IDLE,
} VehicleMode;

typedef int8_t    MapDirF;
typedef struct VehicleParameter
{
    Percentage speed;
    VehicleMode mode;
    VehicleDirect direction;
    VehicleDirect direction_inner;
    uint32_t last_tick_on_mag;
    MapDirF need_rotate_count;
} VehicleParameter;

extern VehicleParameter vehicle_parameter;

void vehicle_ensure_stop_inner(void);
void agv_forward_leave_strong_magnet (void);
void vehicle_ensure_stop(void);
void vehicle_set_mode(VehicleMode mode);
void vehicle_set_direct(VehicleDirect direction);
void vehicle_set_speed(Percentage value);
