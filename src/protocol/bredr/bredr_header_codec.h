/**
 * @file bredr_header_codec.h
 * @brief Reusable BR/EDR header and payload codec helpers.
 */

#ifndef BREDR_HEADER_CODEC_H
#define BREDR_HEADER_CODEC_H

#include <stdint.h>

#include "bredr_phy.h"

#ifdef __cplusplus
extern "C" {
#endif

uint8_t bredr_reverse_byte(uint8_t b);
uint8_t bredr_compute_hec(uint16_t data, uint8_t uap);

void bredr_decode_fec_header_raw(uint64_t header_raw,
                                 uint8_t *lt_addr,
                                 uint8_t *type,
                                 uint8_t *flow,
                                 uint8_t *arqn,
                                 uint8_t *seqn,
                                 uint8_t *hec);

void bredr_decode_header_bits(const bredr_packet_t *pkt,
                              uint8_t clk6,
                              uint8_t bits[18]);

unsigned int bredr_on_air_payload_bits(uint8_t type_code);

unsigned int bredr_extract_payload_bytes(const uint8_t *raw_symbols,
                                         unsigned int bits_collected,
                                         uint8_t *payload_out,
                                         unsigned int payload_capacity);

#ifdef __cplusplus
}
#endif

#endif /* BREDR_HEADER_CODEC_H */
