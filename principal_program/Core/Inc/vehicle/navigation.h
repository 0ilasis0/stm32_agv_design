#pragma once

#include "main/map.h"


#define UNFIND_MAG_TIME 5000

extern MapData agv_state;
extern uint8_t current_count;

void vehicle_track_mode(void);
void vehicle_navigation(void);
void set_agv_state_data(void);
