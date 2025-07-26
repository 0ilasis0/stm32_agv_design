#pragma once

#include "main/fn_state.h"
#include "main/config.h"
#include "main/vehicle.h"

#define INF 99999
#define MAX_NODE 10
#define NO_DATA 0xFF

#define ERROR_STOP_MAP_RETURN(name, expr)   \
    do {                                    \
        Result _err = (expr);               \
        if (RESULT_CHECK_RAW(_err))         \
        {                                   \
            name = _err;                    \
            map_enable = false;             \
            agv_state = map_data_init;      \
            map_trans(&agv_state);          \
            return;                         \
        }                                   \
    } while (0)

typedef uint32_t   MapIdF;
typedef uint8_t    MapDirF;
typedef uint8_t    MapCountF;
typedef uint16_t   MapDisF;

typedef struct MapError{
    Result lose_navigation;
    Result no_path;
    Result input_start_id_err;
} MapError;

typedef struct Connection{
    MapIdF  id;
    MapDisF distance;
} Connection;

typedef struct Location{
    MapIdF     local_id;
    Connection connect[8];
} Location;

typedef struct MapData{
    MapIdF        address_id;
    MapDirF       direction;
    VehicleMotion vehicle_motion;
    MapCountF     need_rotate_count;
    VehicleMode   mode;
    Percentage    speed_setpoint;
} MapData;

typedef struct MapDataAll{
    MapCountF  current_count;
    MapData    map_data[MAX_NODE];
} MapDataAll;
