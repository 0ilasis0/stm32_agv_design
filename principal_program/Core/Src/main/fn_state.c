#include "main/fn_state.h"
#include "main/config.h"

FnState_h error_state = {0};

bool timeout_error (uint32_t start_time, FnState *error_parameter) {
    if (!sys_run_switch.enable_timeout_error) return true;

    if (HAL_GetTick() - start_time > error_timeout_time_limit) {
        *error_parameter = FNS_TIMEOUT;
        return false;
    }
    return true;
}
