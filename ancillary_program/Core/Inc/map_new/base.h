#pragma once

#include "map_new/variable.h"



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



void map_trans (const MapData*);
void floyd_warshall (void);
VehicleMode decide_map_mode_and_speed(uint8_t, MapDirF);
VehicleMotion decide_map_vehicle_motion(MapDirF, MapDirF);
void map_data_renew_direction_and_address (
    MapData*,
    MapIdF,
    MapDirF
);
MapCountF decide_need_rotate_count(
    MapDirF,
    MapIdF,
    MapDirF,
    MapDirF
);
void enforce_stop (void);
MapIdF get_index_by_id (MapIdF);
MapDirF opposite_direction (MapDirF);
