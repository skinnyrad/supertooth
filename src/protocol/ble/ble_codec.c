/**
 * @file ble_codec.c
 * @brief BLE codec helpers extracted from the PHY/framing state machine.
 */

#include "ble_codec.h"

#include <string.h>

void ble_dewhiten(uint8_t *data, unsigned int length_bytes, uint8_t channel_index)
{
    uint8_t lfsr = (channel_index & 0x3Fu) | 0x40u;

    for (unsigned int byte_idx = 0; byte_idx < length_bytes; byte_idx++)
    {
        for (unsigned int bit_idx = 0; bit_idx < 8u; bit_idx++)
        {
            uint8_t out_bit = lfsr & 0x01u;
            data[byte_idx] ^= (uint8_t)(out_bit << bit_idx);
            lfsr ^= (uint8_t)(out_bit << 3u);
            lfsr >>= 1u;
            lfsr |= (uint8_t)(out_bit << 6u);
        }
    }
}

uint8_t ble_bit_reverse_byte(uint8_t b)
{
    b = (uint8_t)(((b & 0xF0u) >> 4u) | ((b & 0x0Fu) << 4u));
    b = (uint8_t)(((b & 0xCCu) >> 2u) | ((b & 0x33u) << 2u));
    b = (uint8_t)(((b & 0xAAu) >> 1u) | ((b & 0x55u) << 1u));
    return b;
}

uint32_t ble_crc_calc(const uint8_t *data, unsigned int len, uint32_t init)
{
    uint32_t crc = init & 0xFFFFFFu;

    for (unsigned int i = 0; i < len; i++)
    {
        for (unsigned int bit = 0; bit < 8u; bit++)
        {
            uint8_t d = (uint8_t)((data[i] >> bit) & 1u);
            uint8_t fb = (uint8_t)(((crc >> 23u) ^ d) & 1u);
            crc = (crc << 1u) & 0xFFFFFFu;
            if (fb)
                crc ^= 0x65Bu;
        }
    }

    return crc;
}

unsigned int ble_payload_length_from_header(const uint8_t header[2])
{
    unsigned int payload_len = header[1];
    if (payload_len > (BLE_PDU_MAX_BYTES - 2u))
        payload_len = BLE_PDU_MAX_BYTES - 2u;
    return payload_len;
}

uint32_t ble_extract_crc(const uint8_t *crc_bytes)
{
    return ((uint32_t)ble_bit_reverse_byte(crc_bytes[0]) << 16u) |
           ((uint32_t)ble_bit_reverse_byte(crc_bytes[1]) << 8u) |
           (uint32_t)ble_bit_reverse_byte(crc_bytes[2]);
}

int ble_decode_advertising_packet(const uint8_t *raw_pdu,
                                  unsigned int total_bytes,
                                  uint8_t channel_index,
                                  uint8_t preamble,
                                  ble_packet_t *out)
{
    if (!raw_pdu || !out || total_bytes < (2u + BLE_CRC_BYTES) ||
        total_bytes > (BLE_PDU_MAX_BYTES + BLE_CRC_BYTES))
        return -1;

    unsigned int pdu_bytes = total_bytes - BLE_CRC_BYTES;
    uint8_t dewhitened[BLE_PDU_MAX_BYTES + BLE_CRC_BYTES];
    memcpy(dewhitened, raw_pdu, total_bytes);
    ble_dewhiten(dewhitened, total_bytes, channel_index);

    memset(out, 0, sizeof(*out));
    out->preamble = preamble;
    out->access_address = BLE_ADVERTISING_AA;
    memcpy(out->pdu, dewhitened, pdu_bytes);
    out->crc = ble_extract_crc(&dewhitened[pdu_bytes]);
    return 0;
}
