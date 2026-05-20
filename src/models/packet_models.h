/**
 * @file packet_models.h
 * @brief Shared packet model types for service and application boundaries.
 */

#ifndef PACKET_MODELS_H
#define PACKET_MODELS_H

#include <stdint.h>

#include "ble_phy.h"
#include "bredr_phy.h"
#include "rx_metadata.h"

#ifdef __cplusplus
extern "C" {
#endif

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

#endif /* PACKET_MODELS_H */
