/**
 * @file test_fec_2_3.c
 * @brief Verify bredr_fec_decode_2_3() against the Bluetooth spec
 *        (15,10) shortened Hamming code sample vectors.
 *
 * Spec: Bluetooth Core Specification, Vol 2, Part B, Section 7.4.1
 *       "Rate 2/3 FEC -- (15,10) Shortened Hamming Code".
 *
 * Codeword bits are sent left-to-right over the air. The packed input
 * convention used by the codec stores the first air bit at LSB bit 0.
 */

#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "bredr_codec.h"

static int g_failures = 0;

#define TEST_ASSERT(cond)                                                                              \
    do                                                                                                \
    {                                                                                                 \
        if (!(cond))                                                                                  \
        {                                                                                             \
            fprintf(stderr, "ASSERT FAILED %s:%d: %s\n", __FILE__, __LINE__, #cond);                  \
            g_failures++;                                                                             \
        }                                                                                             \
    } while (0)

static void pack_air_bits(const char *bits, uint8_t *out, size_t out_bytes)
{
    memset(out, 0, out_bytes);
    size_t pos = 0u;
    for (const char *p = bits; *p != '\0'; p++)
    {
        if (*p == '0' || *p == '1')
        {
            if (*p == '1')
                out[pos / 8u] |= (uint8_t)(1u << (pos % 8u));
            pos++;
        }
    }
}

static void check_codeword(uint16_t expected, int expected_errors, const char *air_bits)
{
    uint8_t input[2] = {0};
    uint8_t output[2] = {0};
    unsigned int output_bits = 0u;
    int ret;

    pack_air_bits(air_bits, input, sizeof(input));
    ret = bredr_fec_decode_2_3(input, 15u, output, &output_bits);

    if (ret < 0)
    {
        fprintf(stderr, "decode returned %d for %s (expected %u, 0x%03X)\n",
                ret, air_bits, expected, expected);
        g_failures++;
        return;
    }

    uint16_t got = (uint16_t)(output[0] | ((uint16_t)(output[1] & 0x03u) << 8));

    TEST_ASSERT(ret == expected_errors);
    TEST_ASSERT(output_bits == 10u);
    TEST_ASSERT(got == expected);

    if (got != expected)
        fprintf(stderr, "  codeword %s: expected 0x%03X, got 0x%03X\n",
                air_bits, expected, got);
}

int main(void)
{
    /* Bluetooth spec: Rate 2/3 FEC -- (15,10) Shortened Hamming Code.
     * Data (hex) | Codeword (10 data bits, space, 5 parity bits). */
    check_codeword(0x001u, 0, "100000000011010");
    check_codeword(0x002u, 0, "010000000001101");
    check_codeword(0x004u, 0, "001000000011100");
    check_codeword(0x008u, 0, "000100000001110");
    check_codeword(0x010u, 0, "000010000000111");
    check_codeword(0x020u, 0, "000001000011001");
    check_codeword(0x040u, 0, "000000100010110");
    check_codeword(0x080u, 0, "000000010001011");
    check_codeword(0x100u, 0, "000000001011111");
    check_codeword(0x200u, 0, "000000000110101");

    if (g_failures != 0)
    {
        fprintf(stderr, "test_fec_2_3: %d assertion(s) failed\n", g_failures);
        return 1;
    }

    puts("test_fec_2_3: 10/10 BT spec (15,10) Hamming codewords decoded");
    return 0;
}
