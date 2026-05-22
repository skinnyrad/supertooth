/**
 * @file bredr_codec.c
 * @brief BR/EDR codec helpers extracted from the PHY state machine.
 */

#include "bredr_codec.h"

#include <string.h>

static const unsigned int s_payload_bits[16] = {
    0, 0, 240, 480, 216, 240, 240, 240,
    560, 232, 1500, 1464, 1440, 1440, 2736, 2712,
};

static const char *const s_bredr_type_names[16] = {
    "NULL", "POLL", "FHS", "DM1",
    "DH1", "HV1", "HV2", "HV3",
    "DV", "AUX1", "DM3", "DH3",
    "EV4", "EV5", "DM5", "DH5"
};

static const uint8_t s_whitening_indices[64] = {
    99, 85, 17, 50, 102, 58, 108, 45, 92, 62, 32, 118, 88, 11, 80, 2,
    37, 69, 55,  8,  20, 40,  74,114, 15,106, 30, 78, 53, 72, 28,26,
    68,  7, 39,113, 105, 77,  71, 25, 84, 49,  57, 44, 61,117, 10, 1,
   123,124, 22,125, 111, 23,  42,126,  6,112,  76, 24, 48, 43,116, 0
};

static const uint8_t s_whitening_data[127] = {
    1,1,1,0,0,0,1,1,1,0,1,1,0,0,0,1,0,1,0,0,1,0,1,1,1,1,1,0,
    1,0,1,0,1,0,0,0,0,1,0,1,1,0,1,1,1,1,0,0,1,1,1,0,0,1,0,1,
    0,1,1,0,0,1,1,0,0,0,0,0,1,1,0,1,1,0,1,0,1,1,1,0,1,0,0,0,
    1,1,0,0,1,0,0,0,1,0,0,0,0,0,0,1,0,0,1,0,0,1,1,0,1,0,0,1,
    1,1,1,0,1,1,1,0,0,0,0,1,1,1,1
};

static uint8_t read_packed_bit(const uint8_t *data, unsigned int bit_pos)
{
    unsigned int byte_idx = bit_pos / 8u;
    unsigned int bit_idx = bit_pos % 8u;
    return (uint8_t)((data[byte_idx] >> bit_idx) & 1u);
}

static void write_packed_bit(uint8_t *data, unsigned int bit_pos, uint8_t bit)
{
    unsigned int byte_idx = bit_pos / 8u;
    unsigned int bit_idx = bit_pos % 8u;
    if (bit & 1u)
        data[byte_idx] |= (uint8_t)(1u << bit_idx);
}

static uint32_t read_packed_field(const uint8_t *data,
                                  unsigned int bit_offset,
                                  unsigned int bit_count)
{
    uint32_t value = 0u;

    for (unsigned int i = 0u; i < bit_count; i++)
        value |= (uint32_t)read_packed_bit(data, bit_offset + i) << i;

    return value;
}

static bredr_payload_family_t bredr_classify_family(uint8_t type_code)
{
    switch (type_code & 0x0Fu)
    {
    case 0x00u:
    case 0x01u:
        return BREDR_PAYLOAD_FAMILY_CONTROL;
    case 0x02u:
        return BREDR_PAYLOAD_FAMILY_FHS;
    case 0x03u:
    case 0x04u:
    case 0x09u:
    case 0x0Au:
    case 0x0Bu:
    case 0x0Eu:
    case 0x0Fu:
        return BREDR_PAYLOAD_FAMILY_ACL;
    case 0x05u:
    case 0x06u:
    case 0x07u:
        return BREDR_PAYLOAD_FAMILY_SCO;
    case 0x08u:
        return BREDR_PAYLOAD_FAMILY_HYBRID;
    case 0x0Cu:
    case 0x0Du:
        return BREDR_PAYLOAD_FAMILY_ESCO;
    default:
        return BREDR_PAYLOAD_FAMILY_UNKNOWN;
    }
}

static bredr_payload_coding_t bredr_classify_coding(uint8_t type_code)
{
    switch (type_code & 0x0Fu)
    {
    case 0x02u:
    case 0x03u:
    case 0x06u:
    case 0x0Au:
    case 0x0Cu:
    case 0x0Eu:
        return BREDR_PAYLOAD_CODING_FEC_2_3;
    case 0x05u:
        return BREDR_PAYLOAD_CODING_FEC_1_3;
    case 0x00u:
    case 0x01u:
    case 0x04u:
    case 0x07u:
    case 0x08u:
    case 0x09u:
    case 0x0Bu:
    case 0x0Du:
    case 0x0Fu:
        return BREDR_PAYLOAD_CODING_NONE;
    default:
        return BREDR_PAYLOAD_CODING_UNKNOWN;
    }
}

static int bredr_acl_payload_header_bytes(uint8_t type_code)
{
    switch (type_code & 0x0Fu)
    {
    case 0x03u:
    case 0x04u:
    case 0x08u:
    case 0x09u:
        return 1;
    case 0x0Au:
    case 0x0Bu:
    case 0x0Eu:
    case 0x0Fu:
        return 2;
    default:
        return 0;
    }
}

static int bredr_payload_supports_direct_parse(uint8_t type_code)
{
    switch (type_code & 0x0Fu)
    {
    case 0x04u:
    case 0x09u:
    case 0x0Bu:
    case 0x0Fu:
        return 1;
    default:
        return 0;
    }
}

static unsigned int bredr_whitening_index_after_bits(uint8_t clk6,
                                                     unsigned int logical_bits)
{
    return (unsigned int)((s_whitening_indices[clk6 & 0x3Fu] + logical_bits) % 127u);
}

static unsigned int bredr_unwhiten_payload_bytes(const uint8_t *src,
                                                 unsigned int src_bits,
                                                 uint8_t clk6,
                                                 unsigned int logical_offset_bits,
                                                 uint8_t *dst,
                                                 unsigned int dst_capacity)
{
    unsigned int bytes = (src_bits + 7u) / 8u;
    unsigned int index;

    if (bytes > dst_capacity)
        bytes = dst_capacity;

    memset(dst, 0, dst_capacity);
    index = bredr_whitening_index_after_bits(clk6, logical_offset_bits);

    for (unsigned int bit_pos = 0u; bit_pos < src_bits; bit_pos++)
    {
        uint8_t bit = read_packed_bit(src, bit_pos) ^ s_whitening_data[index];
        write_packed_bit(dst, bit_pos, bit);
        index = (index + 1u) % 127u;
    }

    return bytes;
}

static void bredr_fill_decoded_header(const uint8_t bits[18], bredr_decoded_header_t *out)
{
    out->lt_addr = (uint8_t)(bits[0] | (bits[1] << 1) | (bits[2] << 2));
    out->type = (uint8_t)(bits[3] | (bits[4] << 1) | (bits[5] << 2) | (bits[6] << 3));
    out->flow = bits[7];
    out->arqn = bits[8];
    out->seqn = bits[9];
    out->hec = 0u;
    for (int i = 0; i < 8; i++)
        out->hec |= (uint8_t)(bits[10 + i] << (7 - i));
}

static int bredr_decode_header(const bredr_frame_t *frame,
                               uint8_t uap,
                               uint8_t clk6,
                               bredr_decoded_header_t *out)
{
    uint8_t bits[18];
    uint16_t hdr_data = 0u;

    bredr_decode_header_bits(frame, clk6, bits);
    bredr_fill_decoded_header(bits, out);

    for (int i = 0; i < 10; i++)
        hdr_data |= (uint16_t)(bits[i] << i);

    out->hec_ok = (uint8_t)(bredr_compute_hec(hdr_data, uap) == out->hec);
    return out->hec_ok;
}

static int bredr_parse_lmp(const uint8_t *body,
                           unsigned int body_len,
                           bredr_lmp_packet_t *out)
{
    uint8_t first_opcode;
    unsigned int params_offset;

    if (!body || !out || body_len == 0u)
        return 0;

    memset(out, 0, sizeof(*out));
    out->tid = body[0] & 0x01u;
    first_opcode = (uint8_t)(body[0] >> 1u);
    out->opcode = first_opcode;

    if (first_opcode >= 124u)
    {
        if (body_len < 2u)
            return 0;
        out->has_ext_opcode = 1u;
        out->ext_opcode = body[1];
        params_offset = 2u;
    }
    else
    {
        params_offset = 1u;
    }

    out->params_len = (uint16_t)(body_len - params_offset);
    if (out->params_len > sizeof(out->params))
        out->params_len = (uint16_t)sizeof(out->params);
    memcpy(out->params, body + params_offset, out->params_len);
    return 1;
}

static int bredr_parse_acl_payload(const uint8_t *payload,
                                   unsigned int payload_len,
                                   uint8_t type_code,
                                   bredr_acl_packet_t *out,
                                   bredr_decode_limit_t *limit)
{
    int header_bytes = bredr_acl_payload_header_bytes(type_code);
    unsigned int body_len;

    if (!payload || !out || header_bytes == 0)
    {
        if (limit)
            *limit = BREDR_DECODE_LIMIT_INVALID_ACL_HEADER;
        return 0;
    }
    if (payload_len < (unsigned int)header_bytes)
    {
        if (limit)
            *limit = BREDR_DECODE_LIMIT_TRUNCATED_PAYLOAD;
        return 0;
    }

    memset(out, 0, sizeof(*out));
    out->payload_header_bytes = (uint8_t)header_bytes;
    out->llid = (bredr_llid_t)(payload[0] & 0x03u);
    out->flow = (uint8_t)((payload[0] >> 2u) & 0x01u);
    out->length = (uint16_t)((payload[0] >> 3u) & 0x1Fu);
    if (header_bytes == 2)
        out->length |= (uint16_t)((payload[1] & 0x1Fu) << 5u);

    if (out->length < out->payload_header_bytes)
    {
        if (limit)
            *limit = BREDR_DECODE_LIMIT_INVALID_ACL_HEADER;
        return 0;
    }

    body_len = (unsigned int)(out->length - out->payload_header_bytes);
    if (payload_len < (unsigned int)out->payload_header_bytes + body_len)
    {
        if (limit)
            *limit = BREDR_DECODE_LIMIT_TRUNCATED_PAYLOAD;
        return 0;
    }

    out->body_len = (uint16_t)body_len;
    memcpy(out->body, payload + out->payload_header_bytes, body_len);

    if ((out->llid == BREDR_LLID_L2CAP_START || out->llid == BREDR_LLID_CONTINUATION) && body_len >= 4u)
    {
        out->has_l2cap = 1u;
        out->l2cap.pdu_length = (uint16_t)(out->body[0] | ((uint16_t)out->body[1] << 8u));
        out->l2cap.cid = (uint16_t)(out->body[2] | ((uint16_t)out->body[3] << 8u));
        out->l2cap.payload_len = (uint8_t)((body_len >= 4u) ? (body_len - 4u) : 0u);
        if ((out->l2cap.cid == 0x0001u || out->l2cap.cid == 0x0005u) && body_len >= 8u)
        {
            out->has_l2cap_signal = 1u;
            out->l2cap_signal.code = out->body[4];
            out->l2cap_signal.identifier = out->body[5];
            out->l2cap_signal.length = (uint16_t)(out->body[6] | ((uint16_t)out->body[7] << 8u));
        }
    }
    else if (out->llid == BREDR_LLID_CONTROL)
    {
        out->has_lmp = (uint8_t)bredr_parse_lmp(out->body, body_len, &out->lmp);
    }

    if (limit)
        *limit = BREDR_DECODE_LIMIT_NONE;
    return 1;
}

/**
 * @brief Apply 1/3-rate FEC majority-vote decode to 18 tripled bits.
 *
 * Extracts each logical bit from its three copies in `header_raw` using
 * majority vote and writes the result into `bits[0..17]`.
 */
static void bredr_fec_majority_vote(uint64_t header_raw, uint8_t bits[18])
{
    for (int i = 0; i < 18; i++)
    {
        uint8_t a = (uint8_t)((header_raw >> (3 * i + 0)) & 1u);
        uint8_t b = (uint8_t)((header_raw >> (3 * i + 1)) & 1u);
        uint8_t c = (uint8_t)((header_raw >> (3 * i + 2)) & 1u);
        bits[i] = (uint8_t)((a & b) | (b & c) | (c & a));
    }
}

uint8_t bredr_reverse_byte(uint8_t b)
{
    b = (uint8_t)(((b & 0xF0u) >> 4) | ((b & 0x0Fu) << 4));
    b = (uint8_t)(((b & 0xCCu) >> 2) | ((b & 0x33u) << 2));
    b = (uint8_t)(((b & 0xAAu) >> 1) | ((b & 0x55u) << 1));
    return b;
}

uint8_t bredr_compute_hec(uint16_t data, uint8_t uap)
{
    uint8_t lfsr = uap;

    for (int i = 0; i < 10; i++)
    {
        uint8_t fb = ((lfsr >> 7u) & 1u) ^ ((data >> i) & 1u);
        lfsr = (uint8_t)(lfsr << 1u);
        if (fb)
            lfsr ^= 0xA7u;
    }
    return lfsr;
}

void bredr_decode_fec_header_raw(uint64_t header_raw,
                                 uint8_t *lt_addr,
                                 uint8_t *type,
                                 uint8_t *flow,
                                 uint8_t *arqn,
                                 uint8_t *seqn,
                                 uint8_t *hec)
{
    uint8_t hdr[18];
    bredr_fec_majority_vote(header_raw, hdr);

    if (lt_addr)
        *lt_addr = (uint8_t)(hdr[0] | (hdr[1] << 1) | (hdr[2] << 2));
    if (type)
        *type = (uint8_t)(hdr[3] | (hdr[4] << 1) | (hdr[5] << 2) | (hdr[6] << 3));
    if (flow)
        *flow = hdr[7];
    if (arqn)
        *arqn = hdr[8];
    if (seqn)
        *seqn = hdr[9];
    if (hec)
        *hec = (uint8_t)(hdr[10] | (hdr[11] << 1) | (hdr[12] << 2) | (hdr[13] << 3) |
                         (hdr[14] << 4) | (hdr[15] << 5) | (hdr[16] << 6) | (hdr[17] << 7));
}

void bredr_decode_header_bits(const bredr_frame_t *frame, uint8_t clk6, uint8_t bits[18])
{
    bredr_fec_majority_vote(frame->header_raw, bits);

    int index = (int)s_whitening_indices[clk6 & 0x3fu];
    for (int i = 0; i < 18; i++)
    {
        bits[i] ^= s_whitening_data[index];
        index = (index + 1) % 127;
    }
}

unsigned int bredr_on_air_payload_bits(uint8_t type_code)
{
    return s_payload_bits[type_code & 0x0Fu];
}

unsigned int bredr_extract_payload_bytes(const uint8_t *raw_symbols,
                                         unsigned int bits_collected,
                                         uint8_t *payload_out,
                                         unsigned int payload_capacity)
{
    unsigned int payload_bits = (bits_collected > 54u) ? (bits_collected - 54u) : 0u;
    unsigned int payload_bytes = (payload_bits + 7u) / 8u;
    if (payload_bytes > payload_capacity)
        payload_bytes = payload_capacity;

    for (unsigned int i = 0u; i < payload_bytes; i++)
    {
        uint8_t byte = 0u;
        for (unsigned int bit_idx = 0u; bit_idx < 8u; bit_idx++)
        {
            unsigned int pos = 54u + i * 8u + bit_idx;
            if ((i * 8u + bit_idx) >= payload_bits)
                break;
            byte |= (uint8_t)(read_packed_bit(raw_symbols, pos) << bit_idx);
        }
        payload_out[i] = byte;
    }

    return payload_bytes;
}

/* ---------------------------------------------------------------------------
 * Sync-word generation
 *
 * (64,30) linear block code, polynomial 0260534236651, modified for the
 * barker code.  Row i is XOR'd into the codeword when bit (23-i) of the
 * LAP is set.  Source: libbtbb bluetooth_packet.c (reference only).
 * ---------------------------------------------------------------------------*/

static const uint64_t s_sw_matrix[24] = {
    0xfe000002a0d1c014ULL, 0x01000003f0b9201fULL,
    0x008000033ae40edbULL, 0x004000035fca99b9ULL,
    0x002000036d5dd208ULL, 0x00100001b6aee904ULL,
    0x00080000db577482ULL, 0x000400006dabba41ULL,
    0x00020002f46d43f4ULL, 0x000100017a36a1faULL,
    0x00008000bd1b50fdULL, 0x000040029c3536aaULL,
    0x000020014e1a9b55ULL, 0x0000100265b5d37eULL,
    0x0000080132dae9bfULL, 0x000004025bd5ea0bULL,
    0x00000203ef526bd1ULL, 0x000001033511ab3cULL,
    0x000000819a88d59eULL, 0x00000040cd446acfULL,
    0x00000022a41aabb3ULL, 0x0000001390b5cb0dULL,
    0x0000000b0ae27b52ULL, 0x0000000585713da9ULL};

/* Default codeword (output when LAP == 0), already incorporating the PN
 * sequence and barker code modification. */
static const uint64_t SW_DEFAULT_CW = 0xb0000002c7820e7eULL;

uint64_t bredr_gen_syncword(uint32_t lap)
{
    uint64_t codeword = SW_DEFAULT_CW;
    for (int i = 0; i < 24; i++)
        if (lap & (0x800000u >> i))
            codeword ^= s_sw_matrix[i];
    return codeword;
}

/* ---------------------------------------------------------------------------
 * HEC verification
 * ---------------------------------------------------------------------------*/

int bredr_hec_ok_for_clk6(const bredr_frame_t *frame, uint8_t uap, uint8_t clk6)
{
    if (!frame || !frame->has_header)
        return 0;

    uint8_t bits[18];
    bredr_decode_header_bits(frame, (uint8_t)(clk6 & 0x3fu), bits);

    uint16_t hdr_data = 0;
    for (int i = 0; i < 10; i++)
        hdr_data |= (uint16_t)(bits[i] << i);

    uint8_t received_hec = 0;
    for (int i = 0; i < 8; i++)
        received_hec |= (uint8_t)(bits[10 + i] << (7 - i));

    return bredr_compute_hec(hdr_data, uap) == received_hec;
}

const char *bredr_packet_type_name(uint8_t type_code)
{
    return s_bredr_type_names[type_code & 0x0Fu];
}

const char *bredr_payload_family_name(bredr_payload_family_t family)
{
    switch (family)
    {
    case BREDR_PAYLOAD_FAMILY_NONE:
        return "None";
    case BREDR_PAYLOAD_FAMILY_CONTROL:
        return "Control";
    case BREDR_PAYLOAD_FAMILY_FHS:
        return "FHS";
    case BREDR_PAYLOAD_FAMILY_ACL:
        return "ACL";
    case BREDR_PAYLOAD_FAMILY_SCO:
        return "SCO";
    case BREDR_PAYLOAD_FAMILY_ESCO:
        return "eSCO";
    case BREDR_PAYLOAD_FAMILY_HYBRID:
        return "Hybrid";
    case BREDR_PAYLOAD_FAMILY_UNKNOWN:
    default:
        return "Unknown";
    }
}

const char *bredr_decode_limit_desc(bredr_decode_limit_t limit)
{
    switch (limit)
    {
    case BREDR_DECODE_LIMIT_NONE:
        return "none";
    case BREDR_DECODE_LIMIT_NO_HEADER:
        return "no header available";
    case BREDR_DECODE_LIMIT_MISSING_CONTEXT:
        return "missing UAP/CLK1-6 context";
    case BREDR_DECODE_LIMIT_HEC_FAILED:
        return "header HEC failed";
    case BREDR_DECODE_LIMIT_UNSUPPORTED_PACKET_TYPE:
        return "packet type not yet supported";
    case BREDR_DECODE_LIMIT_UNSUPPORTED_PAYLOAD_CODING:
        return "payload coding not yet supported";
    case BREDR_DECODE_LIMIT_TRUNCATED_PAYLOAD:
        return "payload shorter than indicated";
    case BREDR_DECODE_LIMIT_INVALID_ACL_HEADER:
        return "invalid ACL payload header";
    default:
        return "unknown decode limit";
    }
}

const char *bredr_llid_name(bredr_llid_t llid)
{
    switch (llid)
    {
    case BREDR_LLID_RESERVED:
        return "Reserved";
    case BREDR_LLID_CONTINUATION:
        return "Continuation";
    case BREDR_LLID_L2CAP_START:
        return "L2CAP Start";
    case BREDR_LLID_CONTROL:
        return "Control/LMP";
    default:
        return "Unknown";
    }
}

int bredr_decode_frame(const bredr_frame_t *frame,
                       const bredr_decode_context_t *ctx,
                       bredr_packet_t *out)
{
    uint8_t dewhitened[BREDR_MAX_PAYLOAD_BYTES];

    if (!frame || !out)
        return -1;

    memset(out, 0, sizeof(*out));
    out->status = BREDR_DECODE_RAW_ONLY;
    out->limit = BREDR_DECODE_LIMIT_NONE;
    out->family = BREDR_PAYLOAD_FAMILY_NONE;
    out->coding = BREDR_PAYLOAD_CODING_NONE;
    out->raw_payload_bytes = (uint16_t)bredr_frame_payload_bytes(frame);

    if (!frame->has_header)
    {
        out->limit = BREDR_DECODE_LIMIT_NO_HEADER;
        return 0;
    }

    if (!ctx || !ctx->have_uap || !ctx->have_clk1_6)
    {
        out->limit = BREDR_DECODE_LIMIT_MISSING_CONTEXT;
        return 0;
    }

    if (!bredr_decode_header(frame, ctx->uap, ctx->clk1_6, &out->header))
    {
        out->limit = BREDR_DECODE_LIMIT_HEC_FAILED;
        return 0;
    }

    out->has_decoded_header = 1u;
    out->type = out->header.type;
    out->family = bredr_classify_family(out->type);
    out->coding = bredr_classify_coding(out->type);
    out->status = BREDR_DECODE_HEADER_ONLY;

    if (out->family == BREDR_PAYLOAD_FAMILY_CONTROL)
    {
        out->status = BREDR_DECODE_FULL_PAYLOAD;
        return 0;
    }

    out->status = BREDR_DECODE_FAMILY_ONLY;

    if (!bredr_payload_supports_direct_parse(out->type))
    {
        out->limit = (out->coding == BREDR_PAYLOAD_CODING_NONE)
                         ? BREDR_DECODE_LIMIT_UNSUPPORTED_PACKET_TYPE
                         : BREDR_DECODE_LIMIT_UNSUPPORTED_PAYLOAD_CODING;

        if (out->family == BREDR_PAYLOAD_FAMILY_SCO || out->family == BREDR_PAYLOAD_FAMILY_ESCO)
        {
            out->payload.sync.payload_bytes = out->raw_payload_bytes;
            out->payload.sync.is_esco = (uint8_t)(out->family == BREDR_PAYLOAD_FAMILY_ESCO);
        }
        else if (out->family == BREDR_PAYLOAD_FAMILY_FHS)
        {
            out->payload.unknown.payload_bytes = out->raw_payload_bytes;
        }
        else
        {
            out->payload.unknown.payload_bytes = out->raw_payload_bytes;
        }
        return 0;
    }

    out->decoded_payload_bytes = (uint16_t)bredr_unwhiten_payload_bytes(frame->payload,
                                                                         frame->payload_bits,
                                                                         ctx->clk1_6,
                                                                         18u,
                                                                         dewhitened,
                                                                         sizeof(dewhitened));

    if (out->family == BREDR_PAYLOAD_FAMILY_ACL)
    {
        if (!bredr_parse_acl_payload(dewhitened,
                                     out->decoded_payload_bytes,
                                     out->type,
                                     &out->payload.acl,
                                     &out->limit))
        {
            out->status = BREDR_DECODE_PARTIAL_PAYLOAD;
            return 0;
        }

        out->status = BREDR_DECODE_FULL_PAYLOAD;
        return 0;
    }

    out->limit = BREDR_DECODE_LIMIT_UNSUPPORTED_PACKET_TYPE;
    return 0;
}