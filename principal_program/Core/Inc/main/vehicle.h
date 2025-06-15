#pragma once

#include "motor/main.h"
#include "main/map.h"

#define HALL_MAGNITUTE_EDGE 1870  //1730

extern uint32_t hall_magnetic_stripe_value;
extern uint32_t hall_strong_magnet_value;
/*測試用--------------------------------------*/
extern uint32_t hall_sensor_direction;
/*測試用--------------------------------------*/
extern MAP_DATA map_data;

typedef enum {
    motion_forward,
    motion_backward,
    motion_clockwise,
    motion_counter_clockwise
} MOTIONCOMMAND;
void vehicle_motion_and_speed_control(MOTIONCOMMAND mode, uint8_t sepoint_value);
void vehicle_track_mode(void);
void vehicle_rotate_in_place(void);
ROTATE_STATUS vehicle_get_rotate_direction(int8_t start_dir, int8_t end_dir);
void rotate_control_direction (ROTATE_STATUS rotate_mode_right, ROTATE_STATUS rotate_mode_left);
void vehicle_renew_vehicle_rotation_status (uint8_t count_until_zero);
void vehicle_over_hall_fall_back(void);
void vehicle_ensure_motor_stop(void);
void vehicle_test_no_load_speed(uint16_t mile_sec);
void over_hall_fall_back_time_based(uint32_t  previous_time_fall_back_dif);
uint8_t vehicle_pass_magnetic_stripe_calculate(
    ROTATE_STATUS rotate_direction_mode,
    uint16_t current_id_input,
    uint8_t from_dir,
    uint8_t to_dir
    );
void vehicle_breakdown_all_hall_lost (void);
void vehicle_search_magnetic_path (MOTIONCOMMAND search_direction, uint16_t time);
MOTIONCOMMAND vehicle_rotate_status_to_motioncommand (ROTATE_STATUS mode);
