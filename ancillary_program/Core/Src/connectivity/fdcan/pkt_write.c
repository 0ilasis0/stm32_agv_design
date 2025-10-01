#include "connectivity/fdcan/pkt_write.h"
#include "connectivity/cmds.h"
#include "main/vec.h"

#ifdef PRINCIPAL_PROGRAM
#include "motor/main.h"
#endif
#ifdef ANCILLARY_PROGRAM
#include "robotic_arm/main.h"
#endif

static void write_u32(uint32_t value, uint8_t* container)
{
    value = swap_u32(value);
    uint8_t *u8 = (uint8_t*)&value;
    container[0] = u8[0];
    container[1] = u8[1];
    container[2] = u8[2];
    container[3] = u8[3];
}

static void write_f32(float value, uint8_t* container)
{
    uint32_t u32;
    memcpy(&u32, &value, sizeof(u32));
    write_u32(u32, container);
}

static float ftest = 0.0;
Result fdcan_data_pkt_write(FdcanPkt* pkt, DataType type)
{
    if (pkt == NULL) return RESULT_ERROR(RES_ERR_MEMORY_ERROR);
    switch (type)
    {
        case DATA_TYPE_TEST:
        {
            pkt->id = FDCAN_TEST_ID;
            pkt->data[0] = CMD_DATA_B0_CONTROL;
            pkt->data[1] = 0xFF;
            write_f32(ftest++, pkt->data + 2);
            pkt->len = 6;
            return RESULT_OK(pkt);
        }
        #ifdef PRINCIPAL_PROGRAM
        case DATA_TYPE_LEFT_SPEED:
        {
            pkt->id = FDCAN_DATA_ID;
            pkt->data[0] = CMD_DATA_B0_CONTROL;
            pkt->data[1] = CMD_DATA_B1_LEFT_SPEED;
            write_f32(motor_left.rps_present, pkt->data + 2);
            pkt->len = 6;
            return RESULT_OK(pkt);
        }
        case DATA_TYPE_LEFT_DUTY:
        {
            pkt->id = FDCAN_DATA_ID;
            pkt->data[0] = CMD_DATA_B0_CONTROL;
            pkt->data[1] = CMD_DATA_B1_LEFT_DUTY;
            pkt->data[2] = motor_left.pwm_duty;
            pkt->len = 3;
            return RESULT_OK(pkt);
        }
        case DATA_TYPE_RIGHT_SPEED:
        {
            pkt->id = FDCAN_DATA_ID;
            pkt->data[0] = CMD_DATA_B0_CONTROL;
            pkt->data[1] = CMD_DATA_B1_RIGHT_SPEED;
            write_f32(motor_left.rps_present, pkt->data + 2);
            pkt->len = 6;
            return RESULT_OK(pkt);
        }
        case DATA_TYPE_RIGHT_DUTY:
        {
            pkt->id = FDCAN_DATA_ID;
            pkt->data[0] = CMD_DATA_B0_CONTROL;
            pkt->data[1] = CMD_DATA_B1_RIGHT_DUTY;
            pkt->data[2] = motor_right.pwm_duty;
            pkt->len = 3;
            return RESULT_OK(pkt);
        }
        #endif
        #ifdef ANCILLARY_PROGRAM
        case DATA_TYPE_ARM_BOTTOM:
        {
            pkt->id = FDCAN_ARM_DATA_ID;
            pkt->data[0] = CMD_DATA_B0_CONTROL;
            pkt->data[1] = CMD_DATA_B1_ARM_BOTTOM;
            pkt->data[2] = (arm_bottom.tim_current - ARM_TIM_MIN) / 2;
            pkt->len = 3;
            return RESULT_OK(pkt);
        }
        case DATA_TYPE_ARM_SHOULDER:
        {
            pkt->id = FDCAN_ARM_DATA_ID;
            pkt->data[0] = CMD_DATA_B0_CONTROL;
            pkt->data[1] = CMD_DATA_B1_ARM_SHOULDER;
            pkt->data[2] = (arm_shoulder.tim_current - ARM_TIM_MIN) / 2;
            pkt->len = 3;
            return RESULT_OK(pkt);
        }
        case DATA_TYPE_ARM_ELBOW_BTM:
        {
            pkt->id = FDCAN_ARM_DATA_ID;
            pkt->data[0] = CMD_DATA_B0_CONTROL;
            pkt->data[1] = CMD_DATA_B1_ARM_ELBOW_BTM;
            pkt->data[2] = (arm_elbow_btm.tim_current - ARM_TIM_MIN) / 2;
            pkt->len = 3;
            return RESULT_OK(pkt);
        }
        case DATA_TYPE_ARM_ELBOW_TOP:
        {
            pkt->id = FDCAN_ARM_DATA_ID;
            pkt->data[0] = CMD_DATA_B0_CONTROL;
            pkt->data[1] = CMD_DATA_B1_ARM_ELBOW_TOP;
            pkt->data[2] = (arm_elbow_top.tim_current - ARM_TIM_MIN) / 2;
            pkt->len = 3;
            return RESULT_OK(pkt);
        }
        case DATA_TYPE_ARM_WRIST:
        {
            pkt->id = FDCAN_ARM_DATA_ID;
            pkt->data[0] = CMD_DATA_B0_CONTROL;
            pkt->data[1] = CMD_DATA_B1_ARM_WRIST;
            pkt->data[2] = (arm_wrist.tim_current - ARM_TIM_MIN) / 2;
            pkt->len = 3;
            return RESULT_OK(pkt);
        }
        case DATA_TYPE_ARM_FINGER:
        {
            pkt->id = FDCAN_ARM_DATA_ID;
            pkt->data[0] = CMD_DATA_B0_CONTROL;
            pkt->data[1] = CMD_DATA_B1_ARM_FINGER;
            pkt->data[2] = (arm_finger.tim_current - ARM_TIM_MIN);
            pkt->len = 3;
            return RESULT_OK(pkt);
        }
        #endif
        default: break;
    }
    return RESULT_ERROR(RES_ERR_NOT_FOUND);
}

#ifdef PRINCIPAL_PROGRAM
Result fdcan_rfid_reset(FdcanPkt* pkt)
{
    pkt->id = FDCAN_MAP_ID;
    pkt->data[0] = CMD_RFID_B0_CONTROL;
    pkt->data[1] = CMD_RFID_B1_RESET_ID;
    pkt->len = 2;
    return RESULT_OK(pkt);
}
#endif

#ifdef ANCILLARY_PROGRAM
Result fdcan_rfid_pkt_write(FdcanPkt* pkt, uint32_t uid, uint8_t n_exist)
{
    if (pkt == NULL) return RESULT_ERROR(RES_ERR_MEMORY_ERROR);
    pkt->id = FDCAN_ARM_DATA_ID;
    pkt->data[0] = CMD_MAP_B0_CONTROL;
    pkt->data[1] = CMD_MAP_B1_INFO;
    write_u32(uid, pkt->data + 2);
    pkt->data[6] = n_exist;
    pkt->len = 7;
    return RESULT_OK(pkt);
}

Result pkt_vehi_set_mode(FdcanPkt* pkt, VehicleMode mode, uint8_t value)
{
    if (pkt == NULL) return RESULT_ERROR(RES_ERR_MEMORY_ERROR);
    pkt->id = FDCAN_VEHI_ID;
    pkt->data[0] = CMD_VEHI_B0_CONTROL;
    pkt->data[1] = CMD_VEHI_B1_MODE;
    switch (mode)
    {
        case VEHICLE_MODE_FREE:
        {
            pkt->data[2] = CMD_VEHI_B2_FREE;
            pkt->len = 3;
            return RESULT_OK(pkt);
        }
        case VEHICLE_MODE_END:
        {
            pkt->data[2] = CMD_VEHI_B2_END;
            pkt->len = 3;
            return RESULT_OK(pkt);
        }
        case VEHICLE_MODE_F_TRACK:
        {
            pkt->data[2] = CMD_VEHI_B2_F_TRACK;
            pkt->len = 3;
            return RESULT_OK(pkt);
        }
        case VEHICLE_MODE_TRACK:
        {
            pkt->data[2] = CMD_VEHI_B2_TRACK;
            pkt->len = 3;
            return RESULT_OK(pkt);
        }
        case VEHICLE_MODE_SEARCH:
        {
            pkt->data[2] = CMD_VEHI_B2_SEARCH;
            pkt->len = 3;
            return RESULT_OK(pkt);
        }
        case VEHICLE_MODE_ROTATE:
        {
            pkt->data[2] = CMD_VEHI_B2_ROTATE;
            pkt->data[3] = value;
            pkt->len = 4;
            return RESULT_OK(pkt);
        }
        default: break;
    }
    return RESULT_ERROR(RES_ERR_NOT_FOUND);
}

Result pkt_vehi_set_motion(FdcanPkt* pkt, VehicleMotion motion)
{
    if (pkt == NULL) return RESULT_ERROR(RES_ERR_MEMORY_ERROR);
    pkt->id = FDCAN_VEHI_ID;
    pkt->data[0] = CMD_VEHI_B0_CONTROL;
    pkt->data[1] = CMD_VEHI_B1_MOTION;
    pkt->len = 3;
    switch (motion)
    {
        case VEHICLE_MOTION_STOP:
        {
            pkt->data[2] = CMD_VEHI_B2_FREE;
            return RESULT_OK(pkt);
        }
        case VEHICLE_MOTION_FORWARD:
        {
            pkt->data[2] = CMD_VEHI_B2_END;
            return RESULT_OK(pkt);
        }
        case VEHICLE_MOTION_BACKWARD:
        {
            pkt->data[2] = CMD_VEHI_B2_F_TRACK;
            return RESULT_OK(pkt);
        }
        case VEHICLE_MOTION_CLOCKWISE:
        {
            pkt->data[2] = CMD_VEHI_B2_SEARCH;
            return RESULT_OK(pkt);
        }
        case VEHICLE_MOTION_C_CLOCKWISE:
        {
            pkt->data[2] = CMD_VEHI_B2_TRACK;
            return RESULT_OK(pkt);
        }
        default: break;
    }
    return RESULT_ERROR(RES_ERR_NOT_FOUND);
}

Result pkt_vehi_set_speed(FdcanPkt* pkt, Percentage value)
{
    if (pkt == NULL) return RESULT_ERROR(RES_ERR_MEMORY_ERROR);
    pkt->id = FDCAN_VEHI_ID;
    pkt->data[0] = CMD_VEHI_B0_CONTROL;
    pkt->data[1] = CMD_VEHI_B1_SPEED;
    pkt->data[2] = value;
    pkt->len = 3;
    return RESULT_OK(pkt);
}
#endif
