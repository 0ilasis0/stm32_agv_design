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
    VEHICLE_MOTION_STOP,
    VEHICLE_MOTION_FORWARD,
    VEHICLE_MOTION_BACKWARD,
    VEHICLE_MOTION_CLOCKWISE,
    VEHICLE_MOTION_C_CLOCKWISE,
    VEHICLE_MOTION_UNKNOWN = -1,
} VehicleMotion;

typedef int8_t    MapDirF;
typedef struct VehicleParameter
{
    Percentage speed;
    VehicleMode mode;
    VehicleMotion motion;
    VehicleMotion motion_inner;
    uint32_t last_tick_on_mag;
    MapDirF need_rotate_count;
} VehicleParameter;

extern VehicleParameter vehicle_h;

void vehicle_ensure_stop_inner(void);
void agv_forward_leave_strong_magnet (void);
void vehicle_ensure_stop(void);
void vehicle_set_mode(VehicleMode mode);
void vehicle_set_motion(VehicleMotion motion);
void vehicle_set_speed(Percentage value);
void vehicle_set_need_rotate(MapDirF value);
