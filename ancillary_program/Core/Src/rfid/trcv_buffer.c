#include "rfid/trcv_buffer.h"

FnState rfid_trcv_buf_setdata()
{
}

FnState rfid_trcv_buf_write(RfidTrcvBuf* self, const RC522Const *rc522_const, RC522MIFARE_Key *key, RC522Uid *uid)
{
    if (self->flags != 0b00001111) return FNS_BUF_NOT_ENOU;
    self->flags = 0;
    if (RC522_PCD_Authenticate(rc522_const, PICC_CMD_MF_AUTH_KEY_A, self->sector * 4, key, uid) != STATUS_Code_OK)
    {
        return FNS_FAIL;
    }
    if (RC522_MIFARE_Write(rc522_const, (self->sector * 4) + self->block, self->data, 16) != STATUS_Code_OK)
    {
        return FNS_FAIL;
    }
    return FNS_OK;
}
