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

#define BREDR_FHS_INFO_BYTES 18u

typedef enum
{
    BREDR_DECODE_RAW_ONLY = 0,
    BREDR_DECODE_HEADER_ONLY,
    BREDR_DECODE_FAMILY_ONLY,
    BREDR_DECODE_PARTIAL_PAYLOAD,
    BREDR_DECODE_FULL_PAYLOAD,
} bredr_decode_status_t;

typedef enum
{
    BREDR_DECODE_LIMIT_NONE = 0,
    BREDR_DECODE_LIMIT_NO_HEADER,
    BREDR_DECODE_LIMIT_MISSING_CONTEXT,
    BREDR_DECODE_LIMIT_HEC_FAILED,
    BREDR_DECODE_LIMIT_UNSUPPORTED_PACKET_TYPE,
    BREDR_DECODE_LIMIT_UNSUPPORTED_PAYLOAD_CODING,
    BREDR_DECODE_LIMIT_TRUNCATED_PAYLOAD,
    BREDR_DECODE_LIMIT_INVALID_ACL_HEADER,
} bredr_decode_limit_t;

typedef enum
{
    BREDR_PAYLOAD_FAMILY_NONE = 0,
    BREDR_PAYLOAD_FAMILY_CONTROL,
    BREDR_PAYLOAD_FAMILY_FHS,
    BREDR_PAYLOAD_FAMILY_ACL,
    BREDR_PAYLOAD_FAMILY_SCO,
    BREDR_PAYLOAD_FAMILY_ESCO,
    BREDR_PAYLOAD_FAMILY_HYBRID,
    BREDR_PAYLOAD_FAMILY_UNKNOWN,
} bredr_payload_family_t;

typedef enum
{
    BREDR_PAYLOAD_CODING_NONE = 0,
    BREDR_PAYLOAD_CODING_FEC_2_3,
    BREDR_PAYLOAD_CODING_FEC_1_3,
    BREDR_PAYLOAD_CODING_UNKNOWN,
} bredr_payload_coding_t;

typedef enum
{
    BREDR_LLID_RESERVED = 0,
    BREDR_LLID_CONTINUATION = 1,
    BREDR_LLID_L2CAP_START = 2,
    BREDR_LLID_CONTROL = 3,
} bredr_llid_t;

typedef struct
{
    uint8_t have_uap;
    uint8_t uap;
    uint8_t have_clk1_6;
    uint8_t clk1_6;
} bredr_decode_context_t;

typedef struct
{
    uint8_t lt_addr;
    uint8_t type;
    uint8_t flow;
    uint8_t arqn;
    uint8_t seqn;
    uint8_t hec;
    uint8_t hec_ok;
} bredr_decoded_header_t;

typedef struct
{
    uint64_t parity_bits;
    uint32_t lap;
    uint8_t eir;
    uint8_t reserved;
    uint8_t sr;
    uint8_t sp;
    uint8_t uap;
    uint16_t nap;
    uint32_t class_of_device;
    uint8_t lt_addr;
    uint32_t clk27_2;
    uint8_t reserved_tail;
} bredr_fhs_packet_t;

typedef struct
{
    uint16_t pdu_length;
    uint16_t cid;
    uint8_t payload_len;
} bredr_l2cap_packet_t;

typedef struct
{
    uint8_t code;
    uint8_t identifier;
    uint16_t length;
} bredr_l2cap_signal_t;

typedef struct
{
    uint8_t tid;
    uint8_t opcode;
    uint8_t has_ext_opcode;
    uint8_t ext_opcode;
    uint16_t params_len;
    uint8_t params[BREDR_MAX_PAYLOAD_BYTES];
} bredr_lmp_packet_t;

typedef struct
{
    bredr_llid_t llid;
    uint8_t flow;
    uint16_t length;
    uint8_t payload_header_bytes;
    uint16_t body_len;
    uint8_t body[BREDR_MAX_PAYLOAD_BYTES];
    uint8_t has_l2cap;
    bredr_l2cap_packet_t l2cap;
    uint8_t has_l2cap_signal;
    bredr_l2cap_signal_t l2cap_signal;
    uint8_t has_lmp;
    bredr_lmp_packet_t lmp;
} bredr_acl_packet_t;

typedef struct
{
    uint16_t payload_bytes;
    uint8_t is_esco;
} bredr_sync_packet_t;

typedef struct
{
    uint16_t payload_bytes;
} bredr_unknown_payload_t;

typedef struct
{
    uint8_t type;
    bredr_decode_status_t status;
    bredr_decode_limit_t limit;
    bredr_payload_family_t family;
    bredr_payload_coding_t coding;
    uint16_t raw_payload_bytes;
    uint16_t decoded_payload_bytes;
    uint8_t has_decoded_header;
    bredr_decoded_header_t header;
    union
    {
        bredr_fhs_packet_t fhs;
        bredr_acl_packet_t acl;
        bredr_sync_packet_t sync;
        bredr_unknown_payload_t unknown;
    } payload;
} bredr_packet_t;

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
const char *bredr_packet_type_name(uint8_t type_code);
const char *bredr_payload_family_name(bredr_payload_family_t family);
const char *bredr_decode_limit_desc(bredr_decode_limit_t limit);
const char *bredr_llid_name(bredr_llid_t llid);
int bredr_decode_frame(const bredr_frame_t *frame,
                       const bredr_decode_context_t *ctx,
                       bredr_packet_t *out);

#ifdef __cplusplus
}
#endif

#endif /* BREDR_CODEC_H */