#pragma once

#include "map_new/variable.h"



void map_trans (const MapDataNode*);
void floyd_warshall (void);
VehicleMode decide_map_mode_and_speed(uint8_t, MapDirF);
VehicleMotion decide_map_vehicle_motion(MapDirF, MapDirF);
void map_data_renew_direction_and_address (
    MapDataNode*,
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
