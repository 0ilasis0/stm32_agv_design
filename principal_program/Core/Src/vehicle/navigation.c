#include "tim.h"
#include "vehicle/navigation.h"
#include "vehicle/rotate.h"
#include "main/fn_state.h"
#include "main/map.h"
#include "main/adc.h"
#include "main/config.h"



/**
  * @brief AGV 倒退直到離開強力磁鐵感應
  */
static void vehicle_over_hall_fall_back(void)
{
    vehicle_set_motion(motion_backward);
    vehicle_set_speed(VEHICLE_setpoint_fall_back);

    uint32_t error_start = HAL_GetTick();
    while(adc_hall.sensor_node >= adc_hall.strong_magnet_value) {
        timeout_error(error_start, &error_state.vehicle_over_hall_fall_back);
    }

    vehicle_ensure_stop();
}

/* 保護未完成動作卻已超出hall範圍 -------------------------------------*/
static void protect_over_hall(void)
{
    if (!runtime_switch.debug_protect_over_hall) return;

    vehicle_ensure_stop();

    if (adc_hall.sensor_node < adc_hall.strong_magnet_value) return;

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

void track_mode(void)
{
    // breakdown_all_hall_lost();
    if (
           adc_hall.sensor_track_right <= adc_hall.magnetic_stripe_value
        && adc_hall.sensor_track_left  >  adc_hall.magnetic_stripe_value
    ) {
        motor_set_state(&motor_right, MOTOR_STATE_SLOW);
    }
    else if
    (
           adc_hall.sensor_track_left  <= adc_hall.magnetic_stripe_value
        && adc_hall.sensor_track_right >  adc_hall.magnetic_stripe_value
    ) {
        motor_set_state(&motor_left, MOTOR_STATE_SLOW);
    }
    else
    {
        motor_set_state(&motor_left, MOTOR_STATE_CONTROL);
        // motor_set_rps_pcn(&motor_left, vehicle_state.speed);
        motor_set_state(&motor_left, MOTOR_STATE_CONTROL);
        // motor_set_rps_pcn(&motor_right, vehicle_state.speed);
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
            rotate_in_place();

            // 改為agv_next，直到離開HALL，使else之後能renew status
            map_data.status[map_data.current_count] = agv_next;
            break;
        case agv_end:
            protect_over_hall();
            init_map_data_direction_and_address(
                &map_data, map_data.address_id[map_data.current_count - 1],
                map_data.direction[map_data.current_count - 1]
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

void vehicle_navigation (void)
{
    if (adc_hall.sensor_node < adc_hall.strong_magnet_value)
    {
        decide_move_mode();
    }
    else
    {
        if (map_data.status[map_data.current_count] == agv_next)
        {
            map_data.current_count++ ;
            agv_state.address_id = map_data.address_id[map_data.current_count];
            agv_state.direction  = map_data.direction[map_data.current_count];
        }
        else
        {
            vehicle_set_mode(VEHICLE_MODE_TRACK);
        }
    }
}

/**
  * @brief 當所有相關的霍爾感測器都失去磁條訊號時，嘗試重新搜尋並回到磁條路徑上
  */
// static void breakdown_all_hall_lost (void)
// {
//     if (!runtime_switch.debug_breakdown_all_hall_lost) return;

//     if (
//             adc_hall.sensor_direction   > adc_hall.magnetic_stripe_value
//         &&  adc_hall.sensor_node        > adc_hall.magnetic_stripe_value
//         &&  adc_hall.sensor_track_right > adc_hall.magnetic_stripe_value
//         &&  adc_hall.sensor_track_left  > adc_hall.magnetic_stripe_value
//     ) {
//         vehicle_ensure_stop();
//         vehicle_search_magnetic_path (motion_clockwise, 3000);
//         vehicle_search_magnetic_path (motion_c_clockwise, 6000);
//         if (runtime_switch.search_magnetic_path == 1)
//         {
//             while (true)
//             {
//                 error_state.breakdown_all_hall_lost__path_not_found = FNS_NOT_MOVE;
//                 osDelay(10);
//             }
//         }

//         runtime_switch.search_magnetic_path = 1;
//     }
// }

/**
  * @brief 在指定時間內，讓裝置順或逆旋轉，直到偵測到磁條，並停止
  */
// static void vehicle_search_magnetic_path (VehicleMotion search_direction, uint16_t time)
// {
//     if (!runtime_switch.search_magnetic_path) return;

//     vehicle_set_motion(search_direction);
//     vehicle_set_speed(VEHICLE_setpoint_rotate);

//     uint32_t past_time = HAL_GetTick();
//     while (HAL_GetTick() - past_time <= time) {
//         // if hall sensor sensing magnetic force
//         if (
//                adc_hall.sensor_direction    < adc_hall.magnetic_stripe_value
//             || adc_hall.sensor_track_left   < adc_hall.magnetic_stripe_value
//             || adc_hall.sensor_track_right  < adc_hall.magnetic_stripe_value
//             || adc_hall.sensor_node         < adc_hall.magnetic_stripe_value
//         ) {
//             runtime_switch.search_magnetic_path = 0;
//             break;
//         }

//         timeout_error(past_time, &error_state.vehicle_search_magnetic_path);
//     }

//     vehicle_ensure_stop();
// }
