#include "vehicle/main.h"
#include "vehicle/navigation.h"
#include "vehicle/search.h"
#include "vehicle/rotate.h"
#include "vehicle/basic.h"
#include "adc/main.h"
#include "main/fn_state.h"

static void direct_update_inner(VehicleDirect direction)
{
    vehicle_parameter.direction_inner = direction;
    switch(direction)
    {
        case VEHICLE_DIRECT_STOP:
        {
            motor_set_direct(&motor_left,  MOTOR_DIRECTION_STOP);
            motor_set_direct(&motor_right, MOTOR_DIRECTION_STOP);
            return;
        }
        case VEHICLE_DIRECT_FORWARD:
        {
            motor_set_direct(&motor_left,  MOTOR_DIRECTION_CCLW);
            motor_set_direct(&motor_right, MOTOR_DIRECTION_CLW);
            return;
        }
        case VEHICLE_DIRECT_BACKWARD:
        {
            motor_set_direct(&motor_left,  MOTOR_DIRECTION_CLW);
            motor_set_direct(&motor_right, MOTOR_DIRECTION_CCLW);
            return;
        }
        case VEHICLE_DIRECT_CLOCKWISE:
        {
            motor_set_direct(&motor_left,  MOTOR_DIRECTION_CCLW);
            motor_set_direct(&motor_right, MOTOR_DIRECTION_CCLW);
            return;
        }
        case VEHICLE_DIRECT_C_CLOCKWISE:
        {
            motor_set_direct(&motor_left,  MOTOR_DIRECTION_CLW);
            motor_set_direct(&motor_right, MOTOR_DIRECTION_CLW);
            return;
        }
        default: break;
    }
}

static void direct_update(void)
{
    if (vehicle_parameter.direction_inner == vehicle_parameter.direction) return;
    else if (vehicle_parameter.direction == VEHICLE_DIRECT_UNKNOWN)
    {
        vehicle_parameter.direction_inner = VEHICLE_DIRECT_UNKNOWN;
        return;
    }
    motor_set_state(&motor_left, MOTOR_STATE_SLOW);
    motor_set_state(&motor_right, MOTOR_STATE_SLOW);
    if (
           (motor_left.rps_present > MOTOR_STOP_GATE)
        || (motor_right.rps_present > MOTOR_STOP_GATE)
    ) return;
    direct_update_inner(vehicle_parameter.direction);
    motor_set_state(&motor_left, MOTOR_STATE_CONTROL);
    motor_set_state(&motor_right, MOTOR_STATE_CONTROL);
}

uint32_t time_diff;
void vehicle_test_no_load_rps(uint32_t ms)
{
    motor_set_max_rps(&motor_left, 0);
    motor_set_max_rps(&motor_right, 0);
    motor_set_state(&motor_left, MOTOR_STATE_FREE);
    motor_set_state(&motor_right, MOTOR_STATE_FREE);

    direct_update_inner(VEHICLE_DIRECT_FORWARD);
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
    direct_update_inner(VEHICLE_DIRECT_BACKWARD);
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
    for(;;)
    {
        direct_update();
        osDelay(50);
    }
}

static bool text_end = 0;
void vehicle_main (void)
{
    vehicle_navigation();

    switch (vehicle_parameter.mode)
    {
        case VEHICLE_MODE_SEARCH:
        {
            if (ERROR_CHECK_FNS_RAW(vehicle_search_mode()))
            {
                vehicle_set_direct(VEHICLE_DIRECT_STOP);
                vehicle_set_mode(VEHICLE_MODE_FREE);
                return;
            }
            vehicle_set_mode(VEHICLE_MODE_TRACK);
            return;
        }
        case VEHICLE_MODE_TRACK:
        {
            vehicle_track_mode();
            return;
        }
        case VEHICLE_MODE_ROTATE:
        {
            vehicle_rotate_in_place();
            return;
        }
        case VEHICLE_MODE_END:
        {
            current_count = 0;
            
            //已經由另一stm32 map修改，目前為text
            map_data_renew_direction_and_address(
                &map_data_start,
                map_data_all.map_data[map_data_all.current_count - 1].address_id,
                map_data_all.map_data[map_data_all.current_count - 1].direction
                );
            //已經由另一stm32 map修改，目前為text

            // 終止目前沒有要做甚麼所以先停止動作
            while (1)
            {
                vehicle_set_direct(VEHICLE_DIRECT_STOP);
                text_end = 1;
                osDelay(100);
            }
            return;
        }
        default: break;
    }
}
