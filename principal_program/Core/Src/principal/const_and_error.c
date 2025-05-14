#include "principal/const_and_error.h"
#include "stm32g4xx_hal.h"

bool ADC_DISABLE = 1;
bool PI_CONTROL_DISABLE = 1;

ERROR_TIMEOUT error_timeout = {
    0,
    0,
    0,
    0,
    0,
    0,
};

void timeout_error (uint32_t error_start, bool *error_parameter) {
    if (HAL_GetTick() - error_start > error_timeout_time_limit) {
        *error_parameter = 1;
    }
}
