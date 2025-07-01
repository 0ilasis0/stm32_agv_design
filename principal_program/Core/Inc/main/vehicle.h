#pragma once

#include "main/config.h"
#include "main/vehicle2.h"



void vehicle_main(void);
void decide_move_mode(void);
void vehicle_track_mode(void);
void vehicle_rotate_in_place(void);
void vehicle_over_hall_fall_back(void);
void vehicle_breakdown_all_hall_lost (void);
void vehicle_search_magnetic_path (MotionCommand search_direction, uint16_t time);
void vehicle_adjust_startup_heading (void);
void vehicle_test_no_load_speed(uint16_t mile_sec);
void protect_over_hall(void);
