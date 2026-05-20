/**
 * @file bredr_codec.h
 * @brief Reusable BR/EDR codec helpers.
 */

#ifndef BREDR_CODEC_H
#define BREDR_CODEC_H

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

/**
 * @brief Decode and unwhiten the 18 logical BR/EDR header bits.
 *
 * The result is 18 unwhitened bits in air order:
 *   bits  0- 2  LT_ADDR  (bit 0 = first transmitted)
 *   bits  3- 6  TYPE     (bit 0 = first transmitted)
 *   bit   7     FLOW
 *   bit   8     ARQN
 *   bit   9     SEQN
 *   bits 10-17  HEC      (bit 10 = first transmitted, i.e. LFSR bit 7)
 *
 * @param frame  Frame with a valid `header_raw` field (has_header must be 1).
 * @param clk6   CLK1-6 whitening key (0-63).
 * @param bits   Output array of 18 bits, one per element (0 or 1).
 */
void bredr_decode_header_bits(const bredr_frame_t *frame,
                              uint8_t clk6,
                              uint8_t bits[18]);

unsigned int bredr_on_air_payload_bits(uint8_t type_code);

unsigned int bredr_extract_payload_bytes(const uint8_t *raw_symbols,
                                         unsigned int bits_collected,
                                         uint8_t *payload_out,
                                         unsigned int payload_capacity);

/**
 * @brief Generate a 64-bit BR/EDR sync word from a 24-bit LAP.
 *
 * Implements the (64,30) linear block code used to encode the LAP into
 * the access code sync word (Bluetooth Core Spec Vol 2, Part B, §6.3.3).
 *
 * @param lap  24-bit Lower Address Part.
 * @return     64-bit sync word in host order (bit 0 = first transmitted).
 */
uint64_t bredr_gen_syncword(uint32_t lap);

/**
 * @brief Verify the HEC of a BR/EDR header for a given CLK1-6 and UAP.
 *
 * Combines FEC majority-vote decode, header unwhitening, and HEC
 * verification in one call.  Returns 0 if `frame` is NULL or has no header.
 *
 * @param frame  Captured frame with a valid `header_raw` field.
 * @param uap    8-bit Upper Address Part.
 * @param clk6   CLK1-6 whitening key (0–63).
 * @return       1 if the HEC matches, 0 otherwise.
 */
int bredr_hec_ok_for_clk6(const bredr_frame_t *frame, uint8_t uap, uint8_t clk6);

#ifdef __cplusplus
}
#endif

#endif /* BREDR_CODEC_H */