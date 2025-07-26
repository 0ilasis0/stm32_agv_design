#include "map/simple.h"
#include "connectivity/fdcan/main.h"
#include "connectivity/cmds.h"
#include "rfid/main.h"

SimplePoint simple_map[20] = {0};
size_t select;

FnState simple_point_go(void)
{
    FnState result;
    VecByte vec_byte;
    ERROR_CHECK_FNS_CLEANUP(vec_byte_new(&vec_byte, FDCAN_VEC_BYTE_CAP));
    VehicleMotion motion;
    Percentage speed = 20;
    VehicleMode mode;
    uint8_t rot_val = 0;
    switch (simple_map[select].direct)
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
            break;
        }
    }
    ERROR_CHECK_FNS_CLEANUP(pkt_vehi_set_motion(&vec_byte, motion));
    ERROR_CHECK_FNS_CLEANUP(fdcan_trcv_buf_push(&fdcan_trsm_pkt_buf, &vec_byte, FDCAN_VEHI_ID));
    ERROR_CHECK_FNS_CLEANUP(pkt_vehi_set_speed(&vec_byte, speed));
    ERROR_CHECK_FNS_CLEANUP(fdcan_trcv_buf_push(&fdcan_trsm_pkt_buf, &vec_byte, FDCAN_VEHI_ID));
    ERROR_CHECK_FNS_CLEANUP(pkt_vehi_set_mode(&vec_byte, mode, rot_val));
    ERROR_CHECK_FNS_CLEANUP(fdcan_trcv_buf_push(&fdcan_trsm_pkt_buf, &vec_byte, FDCAN_VEHI_ID));
    cleanup:
    vec_byte_free(&vec_byte);
    return result;
}

FnState simple_point_select(uint32_t point)
{
    FnState result;
    for (select = 0; select < 20; select++)
    {
        if (
               simple_map[select].uid == 0
        ) {
            simple_map[select].uid = spi2_rfid.uid32;
            result = simple_point_go();
            break;
        }
        else if (point == simple_map[select].uid)
        {
            result = simple_point_go();
        }
    }
    return result;
}

size_t tttt = 0;
FnState simple_point_store(SimpleDirect direct)
{
    tttt++;
    simple_map[select].direct = direct;
    return simple_point_go();;
}
