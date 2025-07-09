#include "vehicle/main.h"
#include "vehicle/navigation.h"
#include "vehicle/search.h"
#include "main/adc.h"

static void direction_set_inner(VehicleMotion motion)
{
    switch(motion)
    {
        case motion_stop:
        {
            motor_set_direction(&motor_left,  MOTOR_DIRECTION_STOP);
            motor_set_direction(&motor_right, MOTOR_DIRECTION_STOP);
            break;
        }
        case motion_forward:
        {
            motor_set_direction(&motor_left,  MOTOR_DIRECTION_CCLW);
            motor_set_direction(&motor_right, MOTOR_DIRECTION_CLW);
            break;
        }
        case motion_backward:
        {
            motor_set_direction(&motor_left,  MOTOR_DIRECTION_CLW);
            motor_set_direction(&motor_right, MOTOR_DIRECTION_CCLW);
            break;
        }
        case motion_clockwise:
        {
            motor_set_direction(&motor_left,  MOTOR_DIRECTION_CCLW);
            motor_set_direction(&motor_right, MOTOR_DIRECTION_CCLW);
            break;
        }
        case motion_c_clockwise:
        {
            motor_set_direction(&motor_left,  MOTOR_DIRECTION_CLW);
            motor_set_direction(&motor_right, MOTOR_DIRECTION_CLW);
            break;
        }
        default: break;
    }
    vehicle_state.motion = motion;
}

static void direction_update(void)
{
    if (vehicle_state.motion_inner == vehicle_state.motion) return;
    motor_set_state(&motor_left, MOTOR_STATE_SLOW);
    motor_set_state(&motor_right, MOTOR_STATE_SLOW);
    if (
            (motor_left.rps_present != 0)
        || (motor_right.rps_present != 0)
    ) return;
    vehicle_state.motion_inner = vehicle_state.motion;
    direction_set_inner(vehicle_state.motion_inner);
    motor_set_state(&motor_left, MOTOR_STATE_CONTROL);
    motor_set_state(&motor_right, MOTOR_STATE_CONTROL);
}

// static FnState search_magnetic_direc(Percentage speed, uint32_t ms)
// {
//     if (!runtime_switch.search_magnetic_path) return FNS_INVALID;

//     vehicle_set_motion(motion_clockwise);
//     vehicle_set_speed(speed);
//     uint32_t past_time = HAL_GetTick();
//     for(;;)
//     {
//         if (HAL_GetTick() - past_time >= ms)
//         {
//             vehicle_set_speed(0);
//             vehicle_ensure_stop();
//             return FNS_NOT_FOUND;
//         }
//         if (adchall_direction.value < adchall_direction.magnetic_value) break;
//         osDelay(10);
//     }
//     vehicle_set_speed(0);
//     vehicle_ensure_stop();
//     return FNS_OK;
// }

// static FnState walk_until_on_path(Percentage speed, uint32_t ms)
// {
//     vehicle_set_motion(motion_forward);
//     vehicle_set_speed(speed);
//     uint32_t past_time = HAL_GetTick();
//     for(;;)
//     {
//         if (HAL_GetTick() - past_time >= ms)
//         {
//             vehicle_set_speed(0);
//             vehicle_ensure_stop();
//             return FNS_NOT_FOUND;
//         }
//         if (
//                adchall_track_left.value   < adchall_track_left.magnetic_value
//             || adchall_track_right.value  < adchall_track_right.magnetic_value
//         ) break;
//         osDelay(10);
//     }
//     vehicle_set_speed(0);
//     vehicle_ensure_stop();
//     return FNS_OK;
// }

// static FnState vehicle_search_mode(void)
// {
//     ERROR_CHECK_FNS_RETURN(search_magnetic_direc(10, 10000));
//     ERROR_CHECK_FNS_RETURN(walk_until_on_path(10, 3000));
//     ERROR_CHECK_FNS_RETURN(search_magnetic_direc(10, 10000));
//     return FNS_OK;
// }

uint32_t time_diff;
void vehicle_test_no_load_rps(uint32_t ms)
{
    // if (!runtime_switch.debug_test_no_load_speed) return;
    // runtime_switch.rps_control = 0;

    motor_set_max_rps(&motor_left, 0);
    motor_set_max_rps(&motor_right, 0);
    motor_set_state(&motor_left, MOTOR_STATE_FREE);
    motor_set_state(&motor_right, MOTOR_STATE_FREE);

    direction_set_inner(motion_forward);
    uint32_t past_time = HAL_GetTick();
    time_diff = past_time;
    motor_set_duty(&motor_left, 100);
    motor_set_duty(&motor_right, 100);
    for(;;)
    {
        if (HAL_GetTick() - past_time > ms) break;
        if (motor_left.rps_present > motor_left.rps_max)
        {
            motor_set_max_rps(&motor_left, motor_left.rps_present);
            past_time = HAL_GetTick();
        }
        if (motor_right.rps_present > motor_right.rps_max)
        {
            motor_set_max_rps(&motor_right, motor_right.rps_present);
            past_time = HAL_GetTick();
        }
        osDelay(10);
        // timeout_error(previous_time_diff, &error_state.vehicle_test_no_load_rps);
    }
    motor_set_duty(&motor_left,  0);
    motor_set_duty(&motor_right, 0);
    if (motor_left.rps_max > motor_right.rps_max) motor_set_max_rps(&motor_left, motor_right.rps_max);
    else motor_set_max_rps(&motor_right, motor_left.rps_max);
    time_diff = HAL_GetTick() - time_diff;
    vehicle_ensure_stop_inner();
    osDelay(1000);

    direction_set_inner(motion_backward);
    past_time = HAL_GetTick();
    motor_set_duty(&motor_left,  100);
    motor_set_duty(&motor_right, 100);
    for(;;)
    {
        if (HAL_GetTick() - past_time >= time_diff) break;
        osDelay(10);
        // timeout_error(past_time, &error_state.vehicle_test_no_load_rps);
    }
    motor_set_duty(&motor_left, 0);
    motor_set_duty(&motor_right, 0);
    vehicle_ensure_stop_inner();

    motor_set_state(&motor_left, MOTOR_STATE_CONTROL);
    motor_set_state(&motor_right, MOTOR_STATE_CONTROL);
    // runtime_switch.rps_control = 1;
}

void StartVehicleTask(void *argument)
{
    for(;;)
    {
        adc_renew();
        direction_update();
        osDelay(50);
    }
}

void vehicle_main(void)
{
    // vehicle_navigation();
    // vehicle_set_motion(motion_forward);
    // vehicle_set_speed(20);

    switch (vehicle_state.mode)
    {
        case VEHICLE_MODE_TRACK:
        {
            vehicle_track_mode();
            return;
        }
        case VEHICLE_MODE_SEARCH:
        {
            if (ERROR_CHECK_FNS_RAW(vehicle_search_mode()))
            {
                vehicle_set_mode(VEHICLE_MODE_FREE);
                return;
            }
            vehicle_set_mode(VEHICLE_MODE_TRACK);
            return;
        }
        default: break;
    }
}
