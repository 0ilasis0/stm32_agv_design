#include "main/const_and_error.h"
#include "stm32g4xx_hal.h"

bool PI_enable = 1;
bool search_magnetic_path_enable = 1;

bool debug_breakdown_all_hall_lost_enable = 1;
bool debug_test_no_load_speed_enable = 0;



ERROR_TIMEOUT error_timeout = {0};
ERROR_DATA error_data = {0};



bool timeout_error (uint32_t error_start, bool *error_parameter) {
    if (HAL_GetTick() - error_start > error_timeout_time_limit) {
        *error_parameter = 1;
        return false;
    }
    return true;
}
