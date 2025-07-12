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
    VEHICLE_DIRECT_STOP,
    VEHICLE_DIRECT_FORWARD,
    VEHICLE_DIRECT_BACKWARD,
    VEHICLE_DIRECT_CLOCKWISE,
    VEHICLE_DIRECT_C_CLOCKWISE,
    VEHICLE_DIRECT_UNKNOWN = -1,
} VehicleMotion;
