#include "main/const_and_error.h"
#include "stm32g4xx_hal.h"



ERR_STATE error_state = {state_ok};
SYSTEM_RUNTIME_SWITCH sys_run_switch = {0};



void const_and_error_set (void) {
    sys_run_switch = sys_run_switch_init();
}

SYSTEM_RUNTIME_SWITCH sys_run_switch_init (void) {
    SYSTEM_RUNTIME_SWITCH sys_run_switch;
    sys_run_switch.enable_adc = 0;
    sys_run_switch.enable_PI = 1;
    sys_run_switch.enable_search_magnetic_path = 1;
    sys_run_switch.enable_timeout_error = 0;

    sys_run_switch.enable_debug_breakdown_all_hall_lost = 1;
    sys_run_switch.enable_debug_test_no_load_speed = 0;

    return sys_run_switch;
}

bool timeout_error (uint32_t start_time, ERR_STATE_MODE *error_parameter) {
    if (!sys_run_switch.enable_timeout_error) return true;

    if (HAL_GetTick() - start_time > error_timeout_time_limit) {
        *error_parameter = state_timeout;
        return false;
    }
    return true;
}
