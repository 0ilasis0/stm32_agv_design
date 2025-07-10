#pragma once

#include "main/map.h"

#define UNFIND_MAG_TIME 5000

extern MapDataCurrent agv_state;

void agv_state_renew (
    MapIdF address_id,
    MapDirF direction,
    VehicleDirect currnet_mode,
    MapDirF real_rotate_count,
    VehicleMode status
);

void vehicle_track_mode(void);
void vehicle_navigation(void);
void set_agv_state_data(void);
