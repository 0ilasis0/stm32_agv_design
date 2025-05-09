#ifndef PRINCIPAL_VEHICLE_H
#define PRINCIPAL_VEHICLE_H

#include <stdint.h>
#include "principal/motor.h"

#define max_duty  100                                          // PWM 最大占空比
#define min_duty  0                                            // PWM 最小占空比

#define max_node 10                                            // 能走最大的路徑數

typedef enum {
    agv_straight,                                   // 循跡狀態mode
    agv_rotate,                                     // 原地旋轉mode
    agv_end                                         // 直行mode
} AGV_STATUS;

// ROTATE_STATUS 移到motor.h內了QQ

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

extern VEHICLE_DATA vehicle_current_data;
extern MAP_DATA map_current_data;

void track_mode(void);
void rotate_in_place(void);
void vehicle_setup(void);
void straight_mode(void);
void pwm_limit(void);
ROTATE_STATUS get_rotate_direction(void);
void rotate_control_direction (ROTATE_STATUS rotate_mode_right, ROTATE_STATUS rotate_mode_left);
void renew_vehicle_current_direction (int renew_direction);

#endif
