#include "vehicle/navigation.h"
#include "tim.h"
#include "vehicle/rotate.h"
#include "vehicle/search.h"
#include "main/fn_state.h"
#include "main/map.h"
#include "adc/main.h"
#include "main/config.h"

AgvState agv_state;

/**
  * @brief AGV 倒退直到離開強力磁鐵感應
  */
static void vehicle_over_hall_fall_back(void)
{
    vehicle_set_direct(VEHICLE_DIRECT_BACKWARD);
    vehicle_set_speed(VEHICLE_SETPOINT_FALL_BACK);

    uint32_t error_start = HAL_GetTick();
    while(adchall_node.value >= adchall_node.const_h.magnetic_value) {
        timeout_error(error_start, &error_state.vehicle_over_hall_fall_back);
    }

    vehicle_ensure_stop();
}

/* 保護未完成動作卻已超出hall範圍 -------------------------------------*/
static void protect_over_hall(void)
{
    if (!runtime_switch.debug_protect_over_hall) return;

    vehicle_ensure_stop();

    if (adchall_node.value < adchall_node.const_h.magnetic_value) return;

    //防止 原地旋轉前 衝過hall_sensor速度仍未停止，後退並強制進入原地旋轉
    if (map_data.status[map_data.current_count] == agv_rotate)
    {
        vehicle_over_hall_fall_back();
    }

    //防止 結束後 衝過hall_sensor 速度仍未停止，進行後退
    if (map_data.status[map_data.current_count] == agv_end)
    {
        vehicle_over_hall_fall_back();
    }
}

/**
  * @brief 決定移動MODE
  */
static int text_end = 0;
static void decide_move_mode(void)
{
    switch(map_data.status[map_data.current_count])
    {
        case agv_straight:
            agv_forward_leave_strong_magnet();

            // 改為agv_next，直到離開HALL，使else之後能renew status
            map_data.status[map_data.current_count] = agv_next;
            break;
        case agv_rotate:
            protect_over_hall();
            vehicle_rotate_in_place(
                map_data.real_rotate_count[map_data.current_count],
                map_data.direction_c[map_data.current_count],
                VEHICLE_setpoint_rotate
                );

            // 改為agv_next，直到離開HALL，使else之後能renew status
            map_data.status[map_data.current_count] = agv_next;
            break;
        case agv_end:
            protect_over_hall();
            init_map_data_direction_and_address(
                &map_data_start,
                map_data.address_id[map_data.current_count - 1],
                map_data.direction_8[map_data.current_count - 1]
                );
            // 終止目前沒有要做甚麼所以先停止動作
            while (1)
            {
                vehicle_ensure_stop();
                text_end = 1;
            }
            break;
        default:
            break;
    }
}

void agv_state_data_setup(void)
{
    agv_state.address_id =  map_data.address_id[0];
    agv_state.direction_8 = map_data.direction_8[0];
    agv_state.vehicle_currnet_mode = map_data.status[0];
}

/**
 * @brief 循跡模式
 */
void vehicle_track_mode(void)
{
    if (
           adchall_track_left.value  >  adchall_track_left.const_h.magnetic_value
        && adchall_track_right.value <= adchall_track_right.const_h.magnetic_value
    ) {
        vehicle_parameter.last_tick_on_mag = HAL_GetTick();
        motor_set_state(&motor_left, MOTOR_STATE_CONTROL);
        motor_set_state(&motor_right, MOTOR_STATE_SLOW);
    }
    else if
    (
           adchall_track_left.value  <= adchall_track_left.const_h.magnetic_value
        && adchall_track_right.value >  adchall_track_right.const_h.magnetic_value
    ) {
        vehicle_parameter.last_tick_on_mag = HAL_GetTick();
        motor_set_state(&motor_left, MOTOR_STATE_SLOW);
        motor_set_state(&motor_right, MOTOR_STATE_CONTROL);
    }
    else
    {
        if (adchall_direction.value <= adchall_direction.const_h.magnetic_value)
        {
            vehicle_parameter.last_tick_on_mag = HAL_GetTick();
        }
        motor_set_state(&motor_left, MOTOR_STATE_CONTROL);
        motor_set_state(&motor_right, MOTOR_STATE_CONTROL);
    }
    if (HAL_GetTick() - vehicle_parameter.last_tick_on_mag >= UNFIND_MAG_TIME)
    {
        vehicle_set_mode(VEHICLE_MODE_SEARCH);
    }
}

void vehicle_navigation(void)
{
    if (adchall_node.value < adchall_node.const_h.magnetic_value)
    {
        decide_move_mode();
    }
    else
    {
        if (map_data.status[map_data.current_count] == agv_next)
        {
            map_data.current_count++ ;
            agv_state.address_id = map_data.address_id[map_data.current_count];
            agv_state.direction_8  = map_data.direction_8[map_data.current_count];
        }
        else
        {
            vehicle_set_mode(VEHICLE_MODE_TRACK);
        }
    }
}
