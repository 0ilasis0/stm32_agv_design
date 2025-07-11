#include "tim.h"
#include "vehicle/basic.h"
#include "vehicle/navigation.h"
#include "vehicle/rotate.h"
#include "vehicle/search.h"
#include "main/fn_state.h"
#include "adc/main.h"
#include "main/config.h"

MapData agv_state;
uint8_t current_count = 0;

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
    if (map_data_all.map_data[current_count].mode == VEHICLE_MODE_ROTATE)
    {
        vehicle_over_hall_fall_back();
    }

    //防止 結束後 衝過hall_sensor 速度仍未停止，進行後退
    if (map_data_all.map_data[current_count].mode == VEHICLE_MODE_END)
    {
        vehicle_over_hall_fall_back();
    }
}

/**
 * @brief 循跡模式
 */
void vehicle_track_mode()
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
        // vehicle_set_mode(VEHICLE_MODE_SEARCH);
    }
}

static void get_mission_from_another_stm32 (void)
{
    map_data_all.current_count ++;
    current_count = map_data_all.current_count;
    agv_state = map_data_all.map_data[current_count];
}

bool navigation_triggered = true;
uint32_t time_stop;
void vehicle_navigation(void)
{
    // 更新資料後需要設定的
    // vehicle_ensure_stop();
    // get_mission_from_another_stm32();
    // current_count++;
    // vehicle_set_mode(agv_state.mode);
    // vehicle_set_direct(agv_state.vehicle_direction);
    // vehicle_set_speed(agv_state.speed_setpoint);

    if (
           adchall_node.value < adchall_node.const_h.magnetic_value
        && navigation_triggered == false
        )
    {
        vehicle_ensure_stop();
        get_mission_from_another_stm32();
        vehicle_set_mode(agv_state.mode);
        vehicle_set_direct(agv_state.vehicle_direction);
        vehicle_set_speed(agv_state.speed_setpoint);

        time_stop = HAL_GetTick();
        navigation_triggered = true;
    }
    else if (
           adchall_node.value > adchall_node.const_h.magnetic_value
        && navigation_triggered == true
        && HAL_GetTick() - time_stop > MAGNATIC_STRIPE_TIME_DIF
        )
    {
        navigation_triggered = false;
    }

    if (vehicle_parameter.mode == VEHICLE_MODE_TRACK)
    {
        vehicle_set_direct(VEHICLE_DIRECT_FORWARD);
        vehicle_set_speed(VEHICLE_SETPOINT_TRACK);
    }
}
