#include "connectivity/fdcan/pkt_write.h"
#include "connectivity/cmds.h"
#include "main/vec.h"

#ifdef PRINCIPAL_PROGRAM
#include "motor/main.h"
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
void fdcan_pkt_write(FdcanPkt* pkt, DataType type)
{
    switch (type)
    {
        case DATA_TYPE_TEST:
        {
            pkt->id = FDCAN_TEST_ID;
            pkt->data[0] = CMD_DATA_B0_CONTROL;
            pkt->data[1] = 0xFF;
            write_f32(ftest++, pkt->data + 2);
            pkt->len = 6;
            return;
        }
        #ifdef PRINCIPAL_PROGRAM
        case DATA_TYPE_LEFT_SPEED:
        {
            pkt->id = FDCAN_DATA_ID;
            pkt->data[0] = CMD_DATA_B0_CONTROL;
            pkt->data[1] = CMD_DATA_B1_LEFT_SPEED;
            write_f32(motor_left.rps_present, pkt->data + 2);
            pkt->len = 6;
            return;
        }
        case DATA_TYPE_LEFT_DUTY:
        {
            pkt->id = FDCAN_DATA_ID;
            pkt->data[0] = CMD_DATA_B0_CONTROL;
            pkt->data[1] = CMD_DATA_B1_LEFT_DUTY;
            pkt->data[2] = motor_left.pwm_duty;
            pkt->len = 3;
            return;
        }
        case DATA_TYPE_RIGHT_SPEED:
        {
            pkt->id = FDCAN_DATA_ID;
            pkt->data[0] = CMD_DATA_B0_CONTROL;
            pkt->data[1] = CMD_DATA_B1_RIGHT_SPEED;
            write_f32(motor_left.rps_present, pkt->data + 2);
            pkt->len = 6;
            return;
        }
        case DATA_TYPE_RIGHT_DUTY:
        {
            pkt->id = FDCAN_DATA_ID;
            pkt->data[0] = CMD_DATA_B0_CONTROL;
            pkt->data[1] = CMD_DATA_B1_RIGHT_DUTY;
            pkt->data[2] = motor_right.pwm_duty;
            pkt->len = 3;
            return;
        }
        #endif
        default: break;
    }
}
