#pragma once

#include "main/map.h"

#define UNFIND_MAG_TIME 5000

typedef struct {
    MapIdF        address_id;
    MapDirF       direction;
    VehicleDirect currnet_mode;
    MapDirF       real_rotate_count;
    AgvStatus     status;
} AgvState;

extern AgvState agv_state;

void agv_state_renew (
    MapIdF address_id,
    MapDirF direction,
    VehicleDirect currnet_mode,
    MapDirF real_rotate_count,
    AgvStatus status
);
void vehicle_track_mode(void);
void vehicle_navigation(void);
void set_agv_state_data(void);
