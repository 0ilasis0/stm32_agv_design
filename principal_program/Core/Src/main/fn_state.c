#include "main/fn_state.h"
#include "main/config.h"

FnState_h error_state = {
    .vehicle_test_no_load_speed = FNS_INVALID,
    .vehicle_over_hall_fall_back = FNS_INVALID,
    .vehicle_rotate_in_place_hall = FNS_INVALID,
    .vehicle_search_magnetic_path = FNS_INVALID,
    .vehicle2_ensure_motor_stop = FNS_INVALID,
    .vehicle2_renew_vehicle_rotation_status = FNS_INVALID,
    .rotate_in_place__map_data_current_count = FNS_INVALID,
    .breakdown_all_hall_lost__path_not_found = FNS_INVALID,
};

bool timeout_error (uint32_t start_time, FnState *error_parameter) {
    if (!sys_run_switch.enable_timeout_error) return true;

    if (HAL_GetTick() - start_time > error_timeout_time_limit) {
        *error_parameter = FNS_TIMEOUT;
        return false;
    }
    return true;
}
