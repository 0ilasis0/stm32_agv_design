#ifndef USER_VEHICLE_H
#define USER_VEHICLE_H

#include "user/motor.h"
#include "user/map.h"

typedef enum {
    agv_straight,                                   // 循跡狀態mode
    agv_rotate,                                     // 原地旋轉mode
    agv_end,                                        // 直行mode
    agv_next
} AGV_STATUS;

typedef struct {
    ROTATE_STATUS rotate_direction;
    AGV_STATUS status;
    int direction;
    int address_id;
} VEHICLE_DATA;

typedef enum {
    motion_forward,
    motion_backward,
    motion_clockwise,
    motion_counter_clockwise
} MOTIONCOMMAND;

extern const uint32_t track_hall_critical_value;
extern const uint32_t node_hall_critical_value;
/*測試用--------------------------------------*/
extern uint32_t hall_count_direction;
/*測試用--------------------------------------*/
extern VEHICLE_DATA vehicle_current_data;
extern MAP_DATA map_current_data;

bool motor_speed_setpoint_set(MOTOR_PARAMETER* motor, uint8_t value);
void track_mode(void);
void rotate_in_place(void);
void vehicle_setup(void);
ROTATE_STATUS get_rotate_direction(void);
void motor_motion_control(MOTIONCOMMAND mode);
void veh_direction(MOTOR_PARAMETER *motor, ROTATE_STATUS direction);
void rotate_control_direction (ROTATE_STATUS rotate_mode_right, ROTATE_STATUS rotate_mode_left);
void renew_vehicle_current_direction (int renew_direction, uint32_t *previous_time);
void over_hall_fall_back(void);
void ensure_motor_stop(void);
void test_no_load_speed(uint16_t mile_sec);
void over_hall_fall_back_time_based(uint32_t  previous_time_fall_back_dif);

#endif
