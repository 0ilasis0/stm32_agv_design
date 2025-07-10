#include "main/config.h"

//FNC_DISABLE
//FNC_ENABLE

RuntimeSwitch runtime_switch = {
    .adc = FNC_ENABLE,
    .rps_control = FNC_ENABLE,
    .search_magnetic_path = FNC_ENABLE,
    .timeout = FNC_DISABLE,

    // .debug_breakdown_all_hall_lost = FNC_DISABLE,
    .debug_protect_over_hall = FNC_DISABLE,
    .debug_test_no_load_speed = FNC_DISABLE,
};

size_t ticking = 0;
