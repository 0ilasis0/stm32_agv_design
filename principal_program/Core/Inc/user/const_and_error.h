#ifndef USER_CONST_H
#define USER_CONST_H

#include <stdbool.h>
#include <stdint.h>

#define error_timeout_time_limit 15 * 1000

extern bool PI_enable;
extern bool search_magnetic_path_enable;

extern bool debug_breakdown_all_hall_lost_enable;
extern bool debug_test_no_load_speed_enable;

typedef struct {
    bool ensure_motor_stop;
    bool test_no_load_speed;
    bool over_hall_fall_back_time_based;
    bool over_hall_fall_back;
    bool rotate_in_place;
    bool rotate_in_place_hall;
    bool renew_vehicle_rotation_status;
    bool search_magnetic_path;
    bool search_magnetic_path_in;
} ERROR_TIMEOUT;

typedef struct {
    bool rotate_in_place__map_data_current_count;
    bool breakdown_all_hall_lost__path_not_found;
} ERROR_DATA;

extern ERROR_TIMEOUT error_timeout;
extern ERROR_DATA error_data;

bool timeout_error (uint32_t error_start, bool *error_parameter);

#endif
