#pragma once

#include "main/config.h"

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
