#include "main/fn_state.h"
#include "vehicle/basic.h"

FnState last_error = FNS_INVALID;

#ifdef PRINCIPAL_PROGRAM

FnState_h error_state = {
    .vehicle_rotate_in_place                    = FNS_INVALID,
    .vehicle_test_no_load_rps                   = FNS_INVALID,
    .vehicle_over_hall_fall_back                = FNS_INVALID,
    .agv_forward_leave_strong_magnet            = FNS_INVALID,
    // .vehicle_search_magnetic_path               = FNS_INVALID,
    .rotate_in_place__map_data_current_count    = FNS_INVALID,
    // .breakdown_all_hall_lost__path_not_found    = FNS_INVALID,
};

static uint32_t look_timeout_dif = 0;
void timeout_error(uint32_t start_time, FnState *error_parameter) {
    osDelay(10);
    if (!runtime_switch.timeout) return;

    look_timeout_dif = HAL_GetTick() - start_time;

    if (HAL_GetTick() - start_time > ERROR_TIMEOUT_TIME_LIMIT) {
        *error_parameter = FNS_TIMEOUT;
        vehicle_ensure_stop();
        while (true) osDelay(10);
    }
}

#endif

#ifdef AGV_ESP32_DEVICE

void Error_Handler(void)
{
    while (1)
    {
    }
}

#endif
