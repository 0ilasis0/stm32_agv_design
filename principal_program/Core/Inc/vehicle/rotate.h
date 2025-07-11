#pragma once

#include "vehicle/basic.h"
#include "main/map.h"


void vehicle_rotate_in_place(MapDirF count_until_zero, VehicleDirect vehicle_direction, Percentage setpoint_speed);
