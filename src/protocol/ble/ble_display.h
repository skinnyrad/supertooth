#ifndef BLE_DISPLAY_H
#define BLE_DISPLAY_H

#include <stdint.h>

#include "ble_phy.h"
#include "rx_metadata.h"

const char *ble_pdu_type_name(uint8_t pdu_type);
int ble_primary_addr(const ble_packet_t *pkt, const uint8_t **addr_out);
void ble_format_addr(char out[18], const uint8_t *addr);
void ble_print_packet_summary_line(unsigned long packet_no,
                                   const ble_packet_t *pkt,
                                   const rx_metadata_t *meta);

#endif
