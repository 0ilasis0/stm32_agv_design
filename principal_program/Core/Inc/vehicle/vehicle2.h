#pragma once

#include "motor/main.h"
#include "main/config.h"
#include "adc.h"
#include "vehicle/main.h"

VehicleMotion vehicle2_get_rotate_direction(int8_t start_dir, int8_t end_dir);
void vehicle2_renew_vehicle_rotation_status (uint8_t count_until_zero);
uint8_t vehicle2_pass_magnetic_stripe_calculate(
    VehicleMotion rotate_direction_mode,
    uint16_t current_id_input,
    uint8_t from_dir,
    uint8_t to_dir
);
