#pragma once

#include <stdbool.h>
#include <stdint.h>

#define error_timeout_time_limit 30 * 1000



typedef enum {
    state_ok,
    state_timeout,
    state_data_no_match,
    state_stop_move
} ERR_STATE_MODE;

typedef struct{
    ERR_STATE_MODE vehicle_test_no_load_speed;
    ERR_STATE_MODE vehicle_over_hall_fall_back;
    ERR_STATE_MODE vehicle_rotate_in_place_hall;
    ERR_STATE_MODE vehicle_search_magnetic_path;
    ERR_STATE_MODE vehicle2_ensure_motor_stop;
    ERR_STATE_MODE vehicle2_renew_vehicle_rotation_status;
    ERR_STATE_MODE rotate_in_place__map_data_current_count;
    ERR_STATE_MODE breakdown_all_hall_lost__path_not_found;
} ERR_STATE;

typedef struct{
    bool enable_PI;
    bool enable_adc;
    bool enable_search_magnetic_path;
    bool enable_timeout_error;

    bool enable_debug_breakdown_all_hall_lost;
    bool enable_debug_test_no_load_speed;
} SYSTEM_RUNTIME_SWITCH;



extern ERR_STATE error_state;
extern SYSTEM_RUNTIME_SWITCH sys_run_switch;

void const_and_error_set (void);
SYSTEM_RUNTIME_SWITCH sys_run_switch_init (void);
bool timeout_error (uint32_t error_start, ERR_STATE_MODE *error_parameter);
