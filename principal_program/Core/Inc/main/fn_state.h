/*
#include "main/fn_state.h"
*/
#pragma once

#include <stdint.h>
#include <stdbool.h>

#define error_timeout_time_limit 30 * 1000

typedef enum FnState
{
    FNS_OK,
    FNS_ERROR,
    FNS_TIMEOUT,
    FNS_BUF_EMPTY,
    FNS_BUF_OVERFLOW,
    FNS_NO_MATCH,

    FNS_NOT_MOVE,
} FnState;
#define FNS_ERROR_CHECK(expr)   \
    do {                        \
        FnState _err = (expr);  \
        if (_err != FNS_OK)     \
            return _err;        \
    } while (0)

typedef struct FnState_h
{
    FnState vehicle_test_no_load_speed;
    FnState vehicle_over_hall_fall_back;
    FnState vehicle_rotate_in_place_hall;
    FnState vehicle_search_magnetic_path;
    FnState vehicle2_ensure_motor_stop;
    FnState vehicle2_renew_vehicle_rotation_status;
    FnState rotate_in_place__map_data_current_count;
    FnState breakdown_all_hall_lost__path_not_found;
} FnState_h;
extern FnState_h error_state;

bool timeout_error (uint32_t error_start, FnState *error_parameter);
