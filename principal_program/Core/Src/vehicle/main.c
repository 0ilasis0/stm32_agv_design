#include "vehicle/main.h"
#include "main/fn_state.h"
#include "adc/main.h"
#include "us_sensor/main.h"

static void motion_update_inner(VehicleMotion direction)
{
    vehicle_h.motion_inner = direction;
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
    if (
           us_sensor_head.status == USS_STATUS_DANGER
        && vehicle_h.motion == VEHICLE_MOTION_FORWARD
    ) vehicle_set_motion(VEHICLE_MOTION_STOP);
    if (vehicle_h.motion_inner == vehicle_h.motion) return;
    else if (vehicle_h.motion == VEHICLE_MOTION_UNKNOWN)
    {
        vehicle_h.motion_inner = VEHICLE_MOTION_UNKNOWN;
        return;
    }
    motor_set_state(&motor_left, MOTOR_STATE_SLOW_0);
    motor_set_state(&motor_right, MOTOR_STATE_SLOW_0);
    if (
           (motor_left.rps_present > MOTOR_STOP_GATE)
        || (motor_right.rps_present > MOTOR_STOP_GATE)
    ) return;
    motion_update_inner(vehicle_h.motion);
    motor_set_state(&motor_left, MOTOR_STATE_CONTROL);
    motor_set_state(&motor_right, MOTOR_STATE_CONTROL);
}

bool vehicle_ready = false;
void StartVehicleUpdateTask(void *argument)
{
    for(;;)
    {
        us_sensor_enable(&us_sensor_head);
        motion_update();
        osDelay(50);
        vehicle_ready = true;
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

void StartVehicleTask(void *argument)
{
    while (
           !motor_ready
        || !vehicle_ready
    ) osDelay(50);

    // vehicle_test_no_load_rps(1000);
    // vehicle_set_motion(VEHICLE_MOTION_CLOCKWISE);
    // vehicle_set_speed(10);

    for(;;)
    {
        switch (vehicle_h.mode)
        {
            case VEHICLE_MODE_END:
            {
                vehicle_set_motion(VEHICLE_MOTION_STOP);
                vehicle_set_mode(VEHICLE_MODE_FREE);
                break;
            }
            case VEHICLE_MODE_TRACK:
            {
                vehicle_track_mode(1000);
                break;
            }
            case VEHICLE_MODE_SEARCH:
            {
                if (ERROR_CHECK_FNS_RAW(vehicle_search_mode(20, 2000)))
                {
                    vehicle_stop();
                    vehicle_set_mode(VEHICLE_MODE_FREE);
                    break;
                }
                vehicle_ensure_stop();
                vehicle_set_mode(VEHICLE_MODE_TRACK);
                break;
            }
            case VEHICLE_MODE_ROTATE:
            {
                vehicle_rotate_in_place(10000);
                break;
            }
            default: break;
        }
        osDelay(50);
    }
}
