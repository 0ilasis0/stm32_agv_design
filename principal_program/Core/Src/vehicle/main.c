#include "vehicle/main.h"
#include "vehicle/navigation.h"
#include "vehicle/search.h"
#include "vehicle/basic.h"
#include "adc/main.h"
#include "main/fn_state.h"

static void motion_update_inner(VehicleMotion direction)
{
    vehicle_parameter.motion_inner = direction;
    switch(direction)
    {
        case VEHICLE_MOTION_STOP:
        {
            motor_set_direct(&motor_left,  MOTOR_DIRECTION_STOP);
            motor_set_direct(&motor_right, MOTOR_DIRECTION_STOP);
            return;
        }
        case VEHICLE_MOTION_FORWARD:
        {
            motor_set_direct(&motor_left,  MOTOR_DIRECTION_CCLW);
            motor_set_direct(&motor_right, MOTOR_DIRECTION_CLW);
            return;
        }
        case VEHICLE_MOTION_BACKWARD:
        {
            motor_set_direct(&motor_left,  MOTOR_DIRECTION_CLW);
            motor_set_direct(&motor_right, MOTOR_DIRECTION_CCLW);
            return;
        }
        case VEHICLE_MOTION_CLOCKWISE:
        {
            motor_set_direct(&motor_left,  MOTOR_DIRECTION_CCLW);
            motor_set_direct(&motor_right, MOTOR_DIRECTION_CCLW);
            return;
        }
        case VEHICLE_MOTION_C_CLOCKWISE:
        {
            motor_set_direct(&motor_left,  MOTOR_DIRECTION_CLW);
            motor_set_direct(&motor_right, MOTOR_DIRECTION_CLW);
            return;
        }
        default: break;
    }
}

static void motion_update(void)
{
    if (vehicle_parameter.motion_inner == vehicle_parameter.motion) return;
    else if (vehicle_parameter.motion == VEHICLE_MOTION_UNKNOWN)
    {
        vehicle_parameter.motion_inner = VEHICLE_MOTION_UNKNOWN;
        return;
    }
    motor_set_state(&motor_left, MOTOR_STATE_SLOW);
    motor_set_state(&motor_right, MOTOR_STATE_SLOW);
    if (
           (motor_left.rps_present > MOTOR_STOP_GATE)
        || (motor_right.rps_present > MOTOR_STOP_GATE)
    ) return;
    motion_update_inner(vehicle_parameter.motion);
    motor_set_state(&motor_left, MOTOR_STATE_CONTROL);
    motor_set_state(&motor_right, MOTOR_STATE_CONTROL);
}

void StartVehicleTask(void *argument)
{
    for(;;)
    {
        motion_update();
        osDelay(50);
    }
}

uint32_t time_diff;
void vehicle_test_no_load_rps(uint32_t ms)
{
    motor_set_max_rps(&motor_left, 0);
    motor_set_max_rps(&motor_right, 0);
    motor_set_state(&motor_left, MOTOR_STATE_FREE);
    motor_set_state(&motor_right, MOTOR_STATE_FREE);

    motion_update_inner(VEHICLE_MOTION_FORWARD);
    uint32_t loop_start = HAL_GetTick();
    time_diff = loop_start;
    motor_set_rps_pcn(&motor_left, 100);
    motor_set_rps_pcn(&motor_right, 100);
    for(;;)
    {
        if (HAL_GetTick() - loop_start > ms) break;
        if (motor_left.rps_present > motor_left.rps_max)
        {
            motor_set_max_rps(&motor_left, motor_left.rps_present);
            loop_start = HAL_GetTick();
        }
        if (motor_right.rps_present > motor_right.rps_max)
        {
            motor_set_max_rps(&motor_right, motor_right.rps_present);
            loop_start = HAL_GetTick();
        }
        osDelay(10);
    }
    motor_set_rps_pcn(&motor_left,  0);
    motor_set_rps_pcn(&motor_right, 0);
    if (motor_left.rps_max > motor_right.rps_max) motor_set_max_rps(&motor_left, motor_right.rps_max);
    else motor_set_max_rps(&motor_right, motor_left.rps_max);
    time_diff = HAL_GetTick() - time_diff;
    vehicle_ensure_stop_inner();
    osDelay(1000);
    // Todo FIX
    motion_update_inner(VEHICLE_MOTION_BACKWARD);
    loop_start = HAL_GetTick();
    motor_set_rps_pcn(&motor_left,  100);
    motor_set_rps_pcn(&motor_right, 100);
    for(;;)
    {
        if (HAL_GetTick() - loop_start >= time_diff) break;
        osDelay(10);
    }
    motor_set_rps_pcn(&motor_left, 0);
    motor_set_rps_pcn(&motor_right, 0);
    vehicle_ensure_stop_inner();

    motor_set_state(&motor_left, MOTOR_STATE_CONTROL);
    motor_set_state(&motor_right, MOTOR_STATE_CONTROL);
}

static bool text_end = 0;
void vehicle_main (void)
{
    switch (vehicle_parameter.mode)
    {
        case VEHICLE_MODE_END:
        {
            text_end = 1;
            vehicle_set_motion(VEHICLE_MOTION_STOP);
            vehicle_set_mode(VEHICLE_MODE_FREE);
            return;
        }
        case VEHICLE_MODE_TRACK:
        {
            vehicle_track_mode();
            return;
        }
        case VEHICLE_MODE_SEARCH:
        {
            if (ERROR_CHECK_FNS_RAW(vehicle_search_mode()))
            {
                vehicle_set_motion(VEHICLE_MOTION_STOP);
                vehicle_set_mode(VEHICLE_MODE_FREE);
                return;
            }
            vehicle_set_mode(VEHICLE_MODE_TRACK);
            return;
        }
        case VEHICLE_MODE_ROTATE:
        {
            vehicle_rotate_in_place();
            return;
        }
        default: break;
    }
}
