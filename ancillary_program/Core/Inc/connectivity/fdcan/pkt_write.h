#pragma once

#include "main/config.h"
#include "main/fn_state.h"
#include "connectivity/fdcan/basic.h"

#ifdef ANCILLARY_PROGRAM
#include "robotic_arm/main.h"
#include "main/vehicle.h"
#endif

typedef enum DataType
{
    DATA_TYPE_TEST,
    #ifdef PRINCIPAL_PROGRAM
    DATA_TYPE_LEFT_SPEED,
    DATA_TYPE_LEFT_DUTY,
    DATA_TYPE_RIGHT_SPEED,
    DATA_TYPE_RIGHT_DUTY,
    #endif
    #ifdef ANCILLARY_PROGRAM
    DATA_TYPE_ARM_BOTTOM,
    DATA_TYPE_ARM_SHOULDER,
    DATA_TYPE_ARM_ELBOW_BTM,
    DATA_TYPE_ARM_ELBOW_TOP,
    DATA_TYPE_ARM_WRIST,
    DATA_TYPE_ARM_FINGER,
    DATA_TYPE_MAP_INFO,
    #endif
} DataType;

Result fdcan_data_pkt_write(FdcanPkt* pkt, DataType type);

#ifdef ANCILLARY_PROGRAM
Result fdcan_rfid_pkt_write(FdcanPkt* pkt, uint32_t uid, uint8_t n_exist);
Result pkt_vehi_set_mode(FdcanPkt* pkt, VehicleMode mode, uint8_t value);
Result pkt_vehi_set_motion(FdcanPkt* pkt, VehicleMotion motion);
Result pkt_vehi_set_speed(FdcanPkt* pkt, Percentage value);
#endif
