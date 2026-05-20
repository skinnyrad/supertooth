/**
 * @file ble_framer.c
 * @brief BLE advertising packet framing implementation.
 */

#include "ble_framer.h"

#include <string.h>

#include "ble_codec.h"

#define MAX_BITS_TO_COLLECT ((BLE_PDU_MAX_BYTES + BLE_CRC_BYTES) * 8u)
#define DETECTION_WINDOW_BITS 40u
#define PATTERN_AA_SHIFT 8u
#define PATTERN_PREAMBLE1 0x55ULL
#define PATTERN_PREAMBLE2 0xAAULL
#define PATTERN1 (PATTERN_PREAMBLE1 | (BLE_ADVERTISING_AA << PATTERN_AA_SHIFT))
#define PATTERN2 (PATTERN_PREAMBLE2 | (BLE_ADVERTISING_AA << PATTERN_AA_SHIFT))
#define WINDOW_MASK ((1ULL << DETECTION_WINDOW_BITS) - 1ULL)

static int window_matches(uint64_t window, uint8_t *preamble_out)
{
    if (window == PATTERN1)
    {
        if (preamble_out)
            *preamble_out = (uint8_t)PATTERN_PREAMBLE1;
        return 1;
    }
    if (window == PATTERN2)
    {
        if (preamble_out)
            *preamble_out = (uint8_t)PATTERN_PREAMBLE2;
        return 1;
    }
    return 0;
}

static void reset_collection(ble_framer_t *framer)
{
    framer->collecting = 0;
    framer->bits_collected = 0;
    framer->header_decoded = 0;
    framer->bits_to_collect = 0;
}

void ble_framer_init(ble_framer_t *framer, uint8_t channel_index)
{
    if (!framer)
        return;

    memset(framer, 0, sizeof(*framer));
    framer->channel_index = channel_index;
}

ble_framer_status_t ble_framer_push_bit(ble_framer_t *framer, uint8_t bit)
{
    if (!framer)
        return BLE_FRAMER_ERROR;

    uint8_t b = bit ? 1u : 0u;

    if (framer->collecting)
    {
        unsigned int byte_idx = framer->bits_collected / 8u;
        unsigned int bit_idx = framer->bits_collected % 8u;

        if (byte_idx >= (BLE_PDU_MAX_BYTES + BLE_CRC_BYTES))
        {
            reset_collection(framer);
            return BLE_FRAMER_ERROR;
        }

        if (b)
            framer->raw_pdu[byte_idx] |= (uint8_t)(1u << bit_idx);

        framer->bits_collected++;

        if (!framer->header_decoded && framer->bits_collected >= 16u)
        {
            uint8_t header[2];
            memcpy(header, framer->raw_pdu, sizeof(header));
            ble_dewhiten(header, sizeof(header), framer->channel_index);
            framer->bits_to_collect =
                (2u + ble_payload_length_from_header(header) + BLE_CRC_BYTES) * 8u;
            framer->header_decoded = 1;
        }

        unsigned int target = framer->header_decoded ? framer->bits_to_collect : MAX_BITS_TO_COLLECT;
        if (framer->bits_collected >= target)
        {
            framer->frame_ready = 1;
            framer->frame_bytes = target / 8u;
            reset_collection(framer);
            return BLE_FRAMER_PACKET_READY;
        }

        return BLE_FRAMER_COLLECTING;
    }

    framer->bit_window = ((framer->bit_window >> 1u) | ((uint64_t)b << 39u)) & WINDOW_MASK;
    if (window_matches(framer->bit_window, &framer->detected_preamble))
    {
        framer->collecting = 1;
        framer->bits_collected = 0;
        framer->bit_window = 0;
        memset(framer->raw_pdu, 0, sizeof(framer->raw_pdu));
        framer->frame_ready = 0;
        framer->frame_bytes = 0;
        return BLE_FRAMER_COLLECTING;
    }

    return BLE_FRAMER_SEARCHING;
}

int ble_framer_get_frame(ble_framer_t *framer,
                         uint8_t *out,
                         unsigned int *out_bytes,
                         uint8_t *out_preamble)
{
    if (!framer || !framer->frame_ready || !out || !out_bytes || !out_preamble)
        return -1;

    memcpy(out, framer->raw_pdu, framer->frame_bytes);
    *out_bytes = framer->frame_bytes;
    *out_preamble = framer->detected_preamble;
    framer->frame_ready = 0;
    framer->frame_bytes = 0;
    memset(framer->raw_pdu, 0, sizeof(framer->raw_pdu));
    return 0;
}
