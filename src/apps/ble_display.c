#include "ble_display.h"

#include <stdio.h>

const char *app_ble_pdu_type_name(uint8_t pdu_type)
{
    switch (pdu_type & 0x0Fu)
    {
    case 0x00u:
        return "ADV_IND";
    case 0x01u:
        return "ADV_DIRECT_IND";
    case 0x02u:
        return "ADV_NONCONN_IND";
    case 0x03u:
        return "SCAN_REQ";
    case 0x04u:
        return "SCAN_RSP";
    case 0x05u:
        return "CONNECT_IND";
    case 0x06u:
        return "ADV_SCAN_IND";
    default:
        return "RESERVED";
    }
}

int app_ble_primary_addr(const ble_packet_t *pkt, const uint8_t **addr_out)
{
    if (!pkt || !addr_out || pkt->pdu[1] < 6u)
        return 0;

    switch (pkt->pdu[0] & 0x0Fu)
    {
    case 0x00u:
    case 0x01u:
    case 0x02u:
    case 0x04u:
    case 0x06u:
    case 0x03u:
    case 0x05u:
        *addr_out = &pkt->pdu[2];
        return 1;
    default:
        return 0;
    }
}

void app_format_ble_addr(char out[18], const uint8_t *addr)
{
    snprintf(out, 18, "%02X:%02X:%02X:%02X:%02X:%02X",
             addr[5], addr[4], addr[3], addr[2], addr[1], addr[0]);
}
