/**
 * @file ble_codec.h
 * @brief Reusable BLE codec helpers shared by framing and packet decode.
 */

#ifndef BLE_CODEC_H
#define BLE_CODEC_H

#include <stdint.h>

#include "ble_phy.h"

#ifdef __cplusplus
extern "C" {
#endif

void ble_dewhiten(uint8_t *data, unsigned int length_bytes, uint8_t channel_index);
uint8_t ble_bit_reverse_byte(uint8_t b);
uint32_t ble_crc_calc(const uint8_t *data, unsigned int len, uint32_t init);
unsigned int ble_payload_length_from_header(const uint8_t header[2]);
uint32_t ble_extract_crc(const uint8_t *crc_bytes);
int ble_decode_advertising_packet(const uint8_t *raw_pdu,
                                  unsigned int total_bytes,
                                  uint8_t channel_index,
                                  uint8_t preamble,
                                  ble_packet_t *out);

#ifdef __cplusplus
}
#endif

#endif /* BLE_CODEC_H */
