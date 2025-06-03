#ifndef USER_VEHICLE_H
#define USER_VEHICLE_H

#include "user/motor.h"
#include "user/map.h"



typedef enum {
    motion_forward,
    motion_backward,
    motion_clockwise,
    motion_counter_clockwise
} MOTIONCOMMAND;

extern const uint32_t hall_magnetic_stripe_value;
extern const uint32_t hall_strong_magnet_value;
/*測試用--------------------------------------*/
extern uint32_t hall_sensor_direction;
/*測試用--------------------------------------*/
extern MAP_DATA map_data;

bool motor_speed_setpoint_set(MOTOR_PARAMETER* motor, uint8_t value);
void track_mode(void);
void rotate_in_place(void);
ROTATE_STATUS get_rotate_direction(void);
void motor_motion_control(MOTIONCOMMAND mode);
void veh_direction(MOTOR_PARAMETER *motor, ROTATE_STATUS direction);
void rotate_control_direction (ROTATE_STATUS rotate_mode_right, ROTATE_STATUS rotate_mode_left);
void renew_vehicle_rotation_status (uint8_t count_until_zero);
void over_hall_fall_back(void);
void ensure_motor_stop(void);
void test_no_load_speed(uint16_t mile_sec);
void over_hall_fall_back_time_based(uint32_t  previous_time_fall_back_dif);
uint8_t pass_magnetic_stripe_calculate(ROTATE_STATUS rotate_direction_mode);
void search_magnetic_path (MOTIONCOMMAND search_direction, uint16_t time);

#endif
