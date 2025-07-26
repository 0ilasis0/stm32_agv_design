#include "vehicle/search.h"
#include "vehicle/basic.h"
#include "adc/main.h"

static Result search_magnetic(VehicleMotion motion, uint32_t ms)
{
    Result result;
    vehicle_set_motion(motion);
    vehicle_ensure_stop();
    uint32_t timeout = HAL_GetTick() + ms;
    for(;;)
    {
        if (HAL_GetTick() >= timeout)
        {
            result = RESULT_ERROR(RES_ERR_NOT_FOUND);
            break;
        }
        if (adchall_direction.state != ADC_HALL_STATE_NONE)
        {
            result = RESULT_OK(NULL);
            break;
        }
        osDelay(50);
    }
    vehicle_set_motion(VEHICLE_MOTION_STOP);
    return result;
}

static Result walk_to_mag(VehicleMotion motion, uint32_t ms)
{
    vehicle_set_motion(motion);
    vehicle_ensure_stop();
    uint32_t timeout = HAL_GetTick() + ms;
    for(;;)
    {
        if (HAL_GetTick() >= timeout)
        {
            vehicle_set_motion(VEHICLE_MOTION_STOP);
            return RESULT_ERROR(RES_ERR_NOT_FOUND);
        }
        if (
               adchall_track_left.state != ADC_HALL_STATE_NONE
            || adchall_track_right.state != ADC_HALL_STATE_NONE
        ) break;
        osDelay(50);
    }
    return RESULT_OK(NULL);
}

Result vehicle_search_mode(Percentage speed, uint32_t ms)
{
    if (!runtime_switch.search_magnetic_path) return RESULT_ERROR(RES_ERR_INVALID);
    Percentage ori_speed = vehicle_h.speed;
    VehicleMotion ori_motion = vehicle_h.motion;
    VehicleMotion motion = VEHICLE_MOTION_CLOCKWISE;
    vehicle_set_speed(speed);
    if (RESULT_CHECK_RAW(search_magnetic(motion, ms)))
    {
        motion = VEHICLE_MOTION_C_CLOCKWISE;
        RESULT_CHECK_RET_RES(search_magnetic(motion, 2*ms));
        motion = VEHICLE_MOTION_CLOCKWISE;
    }
    else
    {
        motion = VEHICLE_MOTION_C_CLOCKWISE;
    }
    RESULT_CHECK_RET_RES(walk_to_mag(ori_motion, 3000));
    RESULT_CHECK_RET_RES(search_magnetic(motion, ms));
    vehicle_set_motion(ori_motion);
    vehicle_set_speed(ori_speed);
    vehicle_set_mode(VEHICLE_MODE_TRACK);
    return RESULT_OK(NULL);
}
