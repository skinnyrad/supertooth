/**
 * @file ble_framer.h
 * @brief BLE advertising packet framing helpers.
 */

#ifndef BLE_FRAMER_H
#define BLE_FRAMER_H

#include <stdint.h>

#ifndef BLE_PDU_MAX_BYTES
#define BLE_PDU_MAX_BYTES 258u
#endif

#ifndef BLE_CRC_BYTES
#define BLE_CRC_BYTES 3u
#endif

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
    BLE_FRAMER_ERROR = -1,
    BLE_FRAMER_SEARCHING = 0,
    BLE_FRAMER_COLLECTING = 1,
    BLE_FRAMER_PACKET_READY = 2,
} ble_framer_status_t;

typedef struct
{
    uint8_t channel_index;
    uint64_t bit_window;
    int collecting;
    unsigned int bits_collected;
    uint8_t raw_pdu[BLE_PDU_MAX_BYTES + BLE_CRC_BYTES];
    int header_decoded;
    unsigned int bits_to_collect;
    int frame_ready;
    unsigned int frame_bytes;
    uint8_t detected_preamble;
} ble_framer_t;

void ble_framer_init(ble_framer_t *framer, uint8_t channel_index);
ble_framer_status_t ble_framer_push_bit(ble_framer_t *framer, uint8_t bit);
int ble_framer_get_frame(ble_framer_t *framer,
                         uint8_t *out,
                         unsigned int *out_bytes,
                         uint8_t *out_preamble);

#ifdef __cplusplus
}
#endif

#endif /* BLE_FRAMER_H */
