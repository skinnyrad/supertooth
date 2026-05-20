/**
 * @file ble_phy.h
 * @brief BLE PHY-layer bitstream processor — real-time, push-bit API.
 *
 * Overview
 * --------
 * This module captures BLE advertising frames from a demodulated bitstream.
 * Framing state lives directly inside `ble_channel_processor_t`, which keeps
 * ownership aligned with BR/EDR: the PHY owns capture and framing, while the
 * codec layer later turns a captured frame into a decoded packet model.
 *
 * Typical usage
 * -------------
 * @code
 *   ble_channel_processor_t proc;
 *   ble_processor_init(&proc, 37);          // advertising channel 37
 *
 *   ble_status_t status = ble_push_bit(&proc, bit);
 *   if (status == BLE_VALID_PACKET) {
 *       ble_frame_t frame;
 *       ble_get_frame(&proc, &frame);
 *       // Pass frame to ble_decode_frame() ...
 *   }
 * @endcode
 */

#ifndef BLE_PHY_H
#define BLE_PHY_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ---------------------------------------------------------------------------
 * Public constants
 * ---------------------------------------------------------------------------*/

#define BLE_PDU_MAX_BYTES 258u
#define BLE_CRC_BYTES 3u
#define BLE_CRC_INIT_ADV 0x555555u
#define BLE_ADVERTISING_AA 0x8E89BED6UL
#define BLE_CH37_INDEX 37u
#define BLE_CH38_INDEX 38u
#define BLE_CH39_INDEX 39u
#define BLE_CH37_FREQ_HZ 2402000000u
#define BLE_CH38_FREQ_HZ 2426000000u
#define BLE_CH39_FREQ_HZ 2480000000u

/* ---------------------------------------------------------------------------
 * ble_frame_t — a captured BLE advertising frame
 * ---------------------------------------------------------------------------*/

typedef struct
{
    uint8_t preamble;
    uint32_t access_address;
    uint8_t raw_pdu[BLE_PDU_MAX_BYTES + BLE_CRC_BYTES];
    uint16_t raw_pdu_bytes;
} ble_frame_t;

/* ---------------------------------------------------------------------------
 * ble_status_t — return codes for ble_push_bit()
 * ---------------------------------------------------------------------------*/

typedef enum
{
    BLE_ERROR = -1,
    BLE_SEARCHING = 0,
    BLE_COLLECTING = 1,
    BLE_VALID_PACKET = 2,
} ble_status_t;

/* ---------------------------------------------------------------------------
 * ble_channel_processor_t — per-channel decoder state
 * ---------------------------------------------------------------------------*/

typedef struct
{
    uint8_t channel_index;
    uint64_t bit_window;
    int collecting;
    unsigned int bits_collected;
    uint8_t raw_pdu[BLE_PDU_MAX_BYTES + BLE_CRC_BYTES];
    int header_decoded;
    unsigned int bits_to_collect;
    unsigned int frame_bytes;
    uint8_t detected_preamble;
    ble_frame_t last_frame;
    int frame_ready;
} ble_channel_processor_t;

/* ---------------------------------------------------------------------------
 * API
 * ---------------------------------------------------------------------------*/

void ble_processor_init(ble_channel_processor_t *proc, uint8_t channel_index);
ble_status_t ble_push_bit(ble_channel_processor_t *proc, uint8_t bit);
int ble_get_frame(ble_channel_processor_t *proc, ble_frame_t *out);

#ifdef __cplusplus
}
#endif

#endif /* BLE_PHY_H */