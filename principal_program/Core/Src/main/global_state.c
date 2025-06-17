#include "main/global_state.h"
#include "uart/main.h"

GlobalState global_state = {
    .uart_tr_pkt_buf_h = &uart_tr_pkt_buf,
    .uart_rv_pkt_buf_h = &uart_rv_pkt_buf,
    .transceive_flags_h = &transceive_flags,
};
