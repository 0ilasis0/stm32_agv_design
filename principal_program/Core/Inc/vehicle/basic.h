#pragma once

#include "main/config.h"
#include "main/fn_state.h"
#include "motor/main.h"

typedef uint8_t VehicleDirect;
#define VEHICLE_DIRECT_STOP         0
#define VEHICLE_DIRECT_FORWARD      1
#define VEHICLE_DIRECT_BACKWARD     2
#define VEHICLE_DIRECT_CLOCKWISE    3
#define VEHICLE_DIRECT_C_CLOCKWISE  4
#define VEHICLE_DIRECT_UNKNOWN      0xFF

typedef uint8_t VehicleMode;
#define VEHICLE_MODE_FREE       0
#define VEHICLE_MODE_TRACK      1
#define VEHICLE_MODE_SEARCH     2

typedef struct VehicleParameter
{
    Percentage speed;
    VehicleMode mode;
    VehicleMode mode_inner;
    VehicleDirect direction;
    VehicleDirect direction_inner;
    uint32_t last_tick_on_mag;
} VehicleParameter;

extern VehicleParameter vehicle_parameter;

void vehicle_ensure_stop_inner(void);
void agv_forward_leave_strong_magnet (void);
void vehicle_ensure_stop(void);
void vehicle_set_mode(VehicleMode mode);
void vehicle_set_direct(VehicleDirect direction);
void vehicle_set_speed(Percentage value);
