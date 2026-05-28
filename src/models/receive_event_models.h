/**
 * @file receive_event_models.h
 * @brief Shared receive-event metadata and event wrapper types.
 */

#ifndef RECEIVE_EVENT_MODELS_H
#define RECEIVE_EVENT_MODELS_H

#include <stdint.h>

#include "ble_bitstream_decoder.h"
#include "bredr_bitstream_decoder.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Minimal radio-agnostic metadata attached to a receive event.
 *
 * The structure is intentionally small: each field is meant to have a clear
 * consumer in framing, tracking, or presentation code.
 */
typedef struct
{
    /** Distinguishes multiple radios, replay streams, or synthetic sources. */
    uint32_t source_id;

    /** Stream-local sample index for ordering and correlation. */
    uint64_t start_sample;

    /** Center frequency in Hz for tuning context and channel mapping. */
    uint32_t center_frequency_hz;

    /** Logical channel index when known; otherwise set by the caller as needed. */
    uint16_t channel_index;

    /** Normalized receive power in dBr, computed by shared RSSI helpers. */
    float rssi_dbr;

    /** Decoder/framer confidence on a 0-255 scale. */
    uint8_t confidence;
} rx_metadata_t;

typedef struct
{
    rx_metadata_t meta;
    ble_frame_t frame;
} ble_event_t;

typedef struct
{
    rx_metadata_t meta;
    bredr_frame_t frame;
} bredr_event_t;

#ifdef __cplusplus
}
#endif

#endif /* RECEIVE_EVENT_MODELS_H */