#include "vehicle/search.h"
#include "vehicle/basic.h"
#include "main/adc.h"



static FnState search_magnetic_direc(Percentage speed, uint32_t ms)
{
    if (!runtime_switch.search_magnetic_path) return FNS_INVALID;

    vehicle_set_motion(motion_clockwise);
    vehicle_set_speed(speed);
    uint32_t past_time = HAL_GetTick();
    for(;;)
    {
        if (HAL_GetTick() - past_time >= ms)
        {
            vehicle_set_speed(0);
            vehicle_ensure_stop();
            return FNS_NOT_FOUND;
        }
        if (adc_hall.sensor_direction < adc_hall.magnetic_stripe_value) break;
        osDelay(10);
    }
    vehicle_set_speed(0);
    vehicle_ensure_stop();
    return FNS_OK;
}

static FnState walk_until_on_path(Percentage speed, uint32_t ms)
{
    vehicle_set_motion(motion_forward);
    vehicle_set_speed(speed);
    uint32_t past_time = HAL_GetTick();
    for(;;)
    {
        if (HAL_GetTick() - past_time >= ms)
        {
            vehicle_set_speed(0);
            vehicle_ensure_stop();
            return FNS_NOT_FOUND;
        }
        if (
               adc_hall.sensor_track_left   < adc_hall.magnetic_stripe_value
            || adc_hall.sensor_track_right  < adc_hall.magnetic_stripe_value
        ) break;
        osDelay(10);
    }
    vehicle_set_speed(0);
    vehicle_ensure_stop();
    return FNS_OK;
}

FnState search_mode(void)
{
    ERROR_CHECK_FNS_RETURN(search_magnetic_direc(10, 10000));
    ERROR_CHECK_FNS_RETURN(walk_until_on_path(10, 3000));
    ERROR_CHECK_FNS_RETURN(search_magnetic_direc(10, 10000));
    return FNS_OK;
}
