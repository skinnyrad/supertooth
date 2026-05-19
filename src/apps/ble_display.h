#ifndef BLE_DISPLAY_H
#define BLE_DISPLAY_H

#include <stdint.h>

#include "ble_phy.h"

const char *app_ble_pdu_type_name(uint8_t pdu_type);
int app_ble_primary_addr(const ble_packet_t *pkt, const uint8_t **addr_out);
void app_format_ble_addr(char out[18], const uint8_t *addr);

#endif
