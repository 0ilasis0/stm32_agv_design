#include "map/main.h"
#include "connectivity/fdcan/pkt_write.h"
#include "rfid/main.h"

HashMap* glo_map = NULL;
static bool card;

static Result map_vech_go(SimpleDirect direct)
{
    Result result = RESULT_OK(NULL);
    FdcanPkt* pkt;
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
            pkt = RESULT_UNWRAP_HANDLE(fdcan_pkt_pool_alloc());
            RESULT_CHECK_HANDLE(fdcan_rfid_pkt_write(pkt, spi2_rfid.uid32, 1));
            fdcan_pkt_buf_push(&fdcan_trsm_pkt_buf, pkt);
            break;
        }
    }
    pkt = RESULT_UNWRAP_HANDLE(fdcan_pkt_pool_alloc());
    RESULT_CHECK_HANDLE(pkt_vehi_set_motion(pkt, motion));
    fdcan_pkt_buf_push(&fdcan_trsm_pkt_buf, pkt);
    pkt = RESULT_UNWRAP_HANDLE(fdcan_pkt_pool_alloc());
    RESULT_CHECK_HANDLE(pkt_vehi_set_speed(pkt, speed));
    fdcan_pkt_buf_push(&fdcan_trsm_pkt_buf, pkt);
    pkt = RESULT_UNWRAP_HANDLE(fdcan_pkt_pool_alloc());
    RESULT_CHECK_HANDLE(pkt_vehi_set_mode(pkt, mode, rot_val));
    fdcan_pkt_buf_push(&fdcan_trsm_pkt_buf, pkt);
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
    osThreadExit();
    return;

    // glo_map = RESULT_UNWRAP_HANDLE(HashMap_new(0));

    // for(;;)
    // {
    //     if (card)
    //     {
    //         if (spi2_rfid.state == CARD_STATE_NONE) card = false;
    //         osDelay(50);
    //         continue;
    //     }
    //     if (spi2_rfid.state != CARD_STATE_EXIST)
    //     {
    //         osDelay(50);
    //         continue;
    //     }
    //     card = true;
    //     map_search(spi2_rfid.uid32);
    //     osDelay(50);
    // }
}
