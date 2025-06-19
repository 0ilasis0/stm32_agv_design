#include "main/config.h"

SYSTEM_RUNTIME_SWITCH sys_run_switch = {
    .enable_adc = 0,
    .enable_PI = 1,
    .enable_search_magnetic_path = 1,
    .enable_timeout_error = 0,

    .enable_debug_breakdown_all_hall_lost = 1,
    .enable_debug_test_no_load_speed = 0,
};
