#pragma once

#include "main/map.h"

#define UNFIND_MAG_TIME 5000

typedef struct {
    MapIdF        address_id;
    MapDirF       direction_8;
    VehicleDirect vehicle_currnet_mode;
} AgvState;

extern AgvState agv_state;

void vehicle_track_mode(void);
void vehicle_navigation(void);
void set_agv_state_data(void);
