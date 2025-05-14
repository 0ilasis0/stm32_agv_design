#ifndef PRINCIPAL_VEHICLE_H
#define PRINCIPAL_VEHICLE_H

#include "principal/motor.h"
#include "principal/const_and_error.h"

#define max_node 10                                 // 能走最大的路徑數

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

typedef struct {
    int current_count;
    AGV_STATUS status[max_node];
    int direction[max_node];
    int address_id[max_node];
} MAP_DATA;

extern const uint32_t track_hall_critical_value;
extern const uint32_t node_hall_critical_value;
/*測試用--------------------------------------*/
extern uint32_t hall_count_direction;
/*測試用--------------------------------------*/
extern VEHICLE_DATA vehicle_current_data;
extern MAP_DATA map_current_data;

void track_mode(void);
void rotate_in_place(void);
void vehicle_setup(void);
void renew_motor_drive(MOTOR_PARAMETER *motor, uint16_t sepoint);
ROTATE_STATUS get_rotate_direction(void);
void rotate_control_direction (ROTATE_STATUS rotate_mode_right, ROTATE_STATUS rotate_mode_left);
void renew_vehicle_current_direction (int renew_direction, uint32_t *previous_time);
void over_hall_fall_back(void);
void ensure_motor_stop(void);
void test_no_load_speed(void);
void change_duty(uint8_t right_duty, uint8_t left_duty);
void over_hall_fall_back_time_based(uint32_t  previous_time_fall_back_dif);

#endif
