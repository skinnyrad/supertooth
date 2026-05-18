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

typedef enum
{
    PROTO_BLE = 0,
    PROTO_BREDR = 1,
} protocol_t;

typedef struct
{
    protocol_t protocol;
    rx_metadata_t meta;
    const uint8_t *raw_bits;
    uint32_t raw_bit_count;
} framed_packet_t;

typedef struct
{
    protocol_t protocol;
    rx_metadata_t meta;
    union
    {
        ble_packet_t ble;
        bredr_packet_t bredr;
    } u;
} decoded_packet_t;

#ifdef __cplusplus
}
#endif

#endif /* PACKET_MODELS_H */
