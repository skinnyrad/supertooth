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
    for (int i = 0; i < 18; i++)
    {
        uint8_t a = (uint8_t)((header_raw >> (3 * i + 0)) & 1u);
        uint8_t b = (uint8_t)((header_raw >> (3 * i + 1)) & 1u);
        uint8_t c = (uint8_t)((header_raw >> (3 * i + 2)) & 1u);
        hdr[i] = (uint8_t)((a & b) | (b & c) | (c & a));
    }

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
    for (int i = 0; i < 18; i++)
    {
        uint8_t a = (uint8_t)((frame->header_raw >> (3*i + 0)) & 1u);
        uint8_t b = (uint8_t)((frame->header_raw >> (3*i + 1)) & 1u);
        uint8_t c = (uint8_t)((frame->header_raw >> (3*i + 2)) & 1u);
        bits[i] = (a & b) | (b & c) | (c & a);
    }

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