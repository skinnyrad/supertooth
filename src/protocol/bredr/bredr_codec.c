/**
 * @file bredr_codec.c
 * @brief BR/EDR codec helpers extracted from the PHY state machine.
 */

#include "bredr_codec.h"

static const unsigned int s_payload_bits[16] = {
    0, 0, 240, 480, 216, 240, 240, 240,
    560, 232, 1500, 1464, 1440, 1440, 2736, 2712,
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