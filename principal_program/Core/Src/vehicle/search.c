#include "vehicle/search.h"
#include "vehicle/basic.h"
#include "adc/main.h"

static FnState search_magnetic_direc(Percentage speed, uint32_t ms)
{
    if (!runtime_switch.search_magnetic_path) return FNS_INVALID;

    vehicle_set_motion(VEHICLE_DIRECT_CLOCKWISE);
    vehicle_set_speed(speed);
    uint32_t past_time = HAL_GetTick();
    for(;;)
    {
        if (HAL_GetTick() - past_time >= ms)
        {
            vehicle_set_motion(VEHICLE_DIRECT_STOP);
            vehicle_ensure_stop();
            return FNS_NOT_FOUND;
        }
        if (adchall_direction.value < adchall_direction.const_h.magnetic_value) break;
        osDelay(10);
    }
    vehicle_set_speed(0);
    vehicle_ensure_stop();
    return FNS_OK;
}

static FnState walk_until_on_path(Percentage speed, uint32_t ms)
{
    vehicle_set_motion(VEHICLE_DIRECT_FORWARD);
    vehicle_set_speed(speed);
    uint32_t past_time = HAL_GetTick();
    for(;;)
    {
        if (HAL_GetTick() - past_time >= ms)
        {
            vehicle_set_motion(VEHICLE_DIRECT_STOP);
            vehicle_ensure_stop();
            return FNS_NOT_FOUND;
        }
        if (
               adchall_track_left.value   < adchall_track_left.const_h.magnetic_value
            || adchall_track_right.value  < adchall_track_right.const_h.magnetic_value
        ) break;
        osDelay(10);
    }
    vehicle_set_speed(0);
    vehicle_ensure_stop();
    return FNS_OK;
}

FnState vehicle_search_mode(void)
{
    ERROR_CHECK_FNS_RETURN(search_magnetic_direc(10, 5000));
    ERROR_CHECK_FNS_RETURN(walk_until_on_path(10, 3000));
    ERROR_CHECK_FNS_RETURN(search_magnetic_direc(10, 5000));
    return FNS_OK;
}
