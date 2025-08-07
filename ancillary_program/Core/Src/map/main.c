#include "map/main.h"
#include "connectivity/fdcan/main.h"
#include "rfid/main.h"

HashMap* glo_map = NULL;
static bool card;

static Result map_vech_go(SimpleDirect direct)
{
    Result result;
    VecByte vec_byte;
    RESULT_CHECK_CLEANUP(vec_byte_new(&vec_byte, FDCAN_VEC_BYTE_CAP));
    VehicleMotion motion;
    Percentage speed = 20;
    VehicleMode mode;
    uint8_t rot_val = 0;
    switch (direct)
    {
        case SIMD_FOWARD:
        {
            motion = VEHICLE_MOTION_FORWARD;
            mode = VEHICLE_MODE_TRACK;
            break;
        }
        case SIMD_BACKWARD:
        {
            motion = VEHICLE_MOTION_CLOCKWISE;
            mode = VEHICLE_MODE_ROTATE;
            rot_val = 2;
            break;
        }
        case SIMD_LEFT:
        {
            motion = VEHICLE_MOTION_C_CLOCKWISE;
            mode = VEHICLE_MODE_ROTATE;
            rot_val = 1;
            break;
        }
        case SIMD_RIGHT:
        {
            motion = VEHICLE_MOTION_CLOCKWISE;
            mode = VEHICLE_MODE_ROTATE;
            rot_val = 1;
            break;
        }
        default:
        {
            motion = VEHICLE_MOTION_STOP;
            RESULT_CHECK_CLEANUP(pkt_map_info(&vec_byte, spi2_rfid.uid32, 1));
            RESULT_CHECK_CLEANUP(fdcan_trcv_buf_push(&fdcan_trsm_pkt_buf, &vec_byte, 0x11));
            break;
        }
    }
    RESULT_CHECK_CLEANUP(pkt_vehi_set_motion(&vec_byte, motion));
    RESULT_CHECK_CLEANUP(fdcan_trcv_buf_push(&fdcan_trsm_pkt_buf, &vec_byte, FDCAN_VEHI_ID));
    RESULT_CHECK_CLEANUP(pkt_vehi_set_speed(&vec_byte, speed));
    RESULT_CHECK_CLEANUP(fdcan_trcv_buf_push(&fdcan_trsm_pkt_buf, &vec_byte, FDCAN_VEHI_ID));
    RESULT_CHECK_CLEANUP(pkt_vehi_set_mode(&vec_byte, mode, rot_val));
    RESULT_CHECK_CLEANUP(fdcan_trcv_buf_push(&fdcan_trsm_pkt_buf, &vec_byte, FDCAN_VEHI_ID));
    cleanup:
    vec_byte_free(&vec_byte);
    return result;
}

Result map_update(SimpleDirect direct)
{
    Result res = HashMap_insert(glo_map, spi2_rfid.uid32, direct);
    if (!res.is_ok) Error_Handler();
    map_vech_go(direct);
    return RESULT_OK(NULL);
}

static void map_search(uint32_t uid32)
{
    Result result = HashMap_get(glo_map, uid32);
    if (RESULT_CHECK_RAW(result))
    {
        result = map_vech_go(SIMD_INVALED);
        result = HashMap_insert(glo_map, uid32, SIMD_INVALED);
    }
    else
    {
        SimpleDirect* direct = RESULT_UNWRAP_HANDLE(result);
        result = map_vech_go(*direct);
    }
}

void StartMapTask(void *argument)
{
    // osThreadExit();
    // return;

    glo_map = RESULT_UNWRAP_HANDLE(HashMap_new(0));

    for(;;)
    {
        if (card)
        {
            if (spi2_rfid.state == CARD_STATE_NONE) card = false;
            osDelay(50);
            continue;
        }
        if (spi2_rfid.state != CARD_STATE_EXIST)
        {
            osDelay(50);
            continue;
        }
        card = true;
        map_search(spi2_rfid.uid32);
        osDelay(50);
    }
}
