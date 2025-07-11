#include "tim.h"
#include "vehicle/basic.h"
#include "vehicle/navigation.h"
#include "vehicle/rotate.h"
#include "vehicle/search.h"
#include "main/fn_state.h"
#include "adc/main.h"
#include "main/config.h"

MapData agv_state;

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
    if (map_data_all.map_data[map_data_all.current_count].mode == VEHICLE_MODE_ROTATE)
    {
        vehicle_over_hall_fall_back();
    }

    //防止 結束後 衝過hall_sensor 速度仍未停止，進行後退
    if (map_data_all.map_data[map_data_all.current_count].mode == VEHICLE_MODE_END)
    {
        vehicle_over_hall_fall_back();
    }
}

void agv_state_renew (
    MapIdF address_id,
    MapDirF direction,
    VehicleDirect vehicle_direction,
    MapDirF real_rotate_count,
    VehicleMode mode
)
{
    agv_state.address_id = address_id;
    agv_state.direction = direction;
    agv_state.vehicle_direction = vehicle_direction;
    agv_state.real_rotate_count = real_rotate_count;
    agv_state.mode = mode;
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

static void renew_agv_state_another_stm32 (void)
{
    map_data_all.current_count ++;
    agv_state_renew(
        map_data_all.map_data[map_data_all.current_count].address_id,
        map_data_all.map_data[map_data_all.current_count].direction,
        map_data_all.map_data[map_data_all.current_count].vehicle_direction,
        map_data_all.map_data[map_data_all.current_count].real_rotate_count,
        map_data_all.map_data[map_data_all.current_count].mode
    );
}

bool navigation_triggered = true;
uint32_t time_stop;
void vehicle_navigation(void)
{
    if (
           adchall_node.value < adchall_node.const_h.magnetic_value
        && navigation_triggered == false
        )
    {
        renew_agv_state_another_stm32();
        vehicle_set_mode(agv_state.mode);

        time_stop = HAL_GetTick();
        navigation_triggered = true;
    }
    else if (
           adchall_node.value > adchall_node.const_h.magnetic_value
        && navigation_triggered == true
        && HAL_GetTick() - time_stop > NAVUGATION_TIME_DIFF
        )
    {
        navigation_triggered = false;
    }

    if (agv_state.mode == VEHICLE_MODE_TRACK)
    {
        vehicle_set_direct(VEHICLE_DIRECT_FORWARD);
        vehicle_set_speed(VEHICLE_setpoint_straight);
    }
}
