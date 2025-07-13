#pragma once

#include "main/fn_state.h"
#include "main/config.h"
#include "main/vehicle.h"

#define INF 99999
#define MAX_NODE 10
#define NO_DATA 0xFF

#define ERROR_STOP_MAP_RETURN(name, expr)   \
    do {                                    \
        FnState   _err = (expr);            \
        if (_err != FNS_OK)                 \
        {                                   \
            name = _err;                    \
            map_enable = false;             \
            agv_state = map_data_init;      \
            map_trans(&agv_state);          \
            return;                         \
        }                                   \
    } while (0)

typedef uint16_t   MapIdF;
typedef uint8_t    MapDirF;
typedef uint8_t    MapCountF;
typedef uint16_t   MapDisF;

typedef struct{
    FnState lose_navigation;
    FnState no_path;
    FnState map_data_trans_error[5];
} MapError;

typedef struct {
    MapIdF  id;
    MapDisF distance;
} Connection;

typedef struct{
    MapIdF     local_id;
    Connection connect[8];
} Location;

typedef struct {
    MapIdF        address_id;
    MapDirF       direction;
    VehicleMotion vehicle_motion;
    MapCountF     need_rotate_count;
    VehicleMode   mode;
    Percentage    speed_setpoint;
} MapData;

typedef struct{
    MapCountF  current_count;
    MapData    map_data[MAX_NODE];
} MapDataAll;
