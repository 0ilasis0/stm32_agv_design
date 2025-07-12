#pragma once

#include "main/config.h"
#include "main/fn_state.h"
#include "motor/main.h"

typedef enum VehicleMode
{
    // 自由
    VEHICLE_MODE_FREE,
    // 停止
    VEHICLE_MODE_END,
    // 離點(前進後循跡)
    VEHICLE_MODE_F_TRACK,
    // 循跡
    VEHICLE_MODE_TRACK,
    // 尋找
    VEHICLE_MODE_SEARCH,
    // 旋轉
    VEHICLE_MODE_ROTATE,
} VehicleMode;

typedef enum VehicleMotion
{
    VEHICLE_DIRECT_STOP,
    VEHICLE_DIRECT_FORWARD,
    VEHICLE_DIRECT_BACKWARD,
    VEHICLE_DIRECT_CLOCKWISE,
    VEHICLE_DIRECT_C_CLOCKWISE,
    VEHICLE_DIRECT_UNKNOWN = -1,
} VehicleMotion;

typedef int8_t    MapDirF;
typedef struct VehicleParameter
{
    Percentage speed;
    VehicleMode mode;
    VehicleMotion direction;
    VehicleMotion direction_inner;
    uint32_t last_tick_on_mag;
    MapDirF need_rotate_count;
} VehicleParameter;

extern VehicleParameter vehicle_parameter;

void vehicle_ensure_stop_inner(void);
void agv_forward_leave_strong_magnet (void);
void vehicle_ensure_stop(void);
void vehicle_set_mode(VehicleMode mode);
void vehicle_set_direct(VehicleMotion direction);
void vehicle_set_speed(Percentage value);
void vehicle_set_need_rotate(MapDirF value);
