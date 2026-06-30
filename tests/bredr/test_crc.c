/**
 * @file test_crc.c
 * @brief Verify bredr_payload_crc() against the Bluetooth spec sample
 *        payload from §7.1.1 "CRC sample data".
 *
 * Spec: Bluetooth Core Specification, Vol 2, Part B, §7.1.1.
 *
 * Sample:
 *   data[0..9] = 0x4e 0x01 0x02 0x03 0x04 0x05 0x06 0x07 0x08 0x09
 *   UAP        = 0x47
 *   CRC        = 0x6d 0xd2   (low byte first, matching air order)
 *
 * Each byte in the codeword is sent LSB first, which matches the
 * packed-bit convention used by the CRC implementation.
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

int main(void)
{
    static const uint8_t data[10] = {
        0x4e, 0x01, 0x02, 0x03, 0x04,
        0x05, 0x06, 0x07, 0x08, 0x09,
    };
    static const uint8_t  uap            = 0x47u;
    static const uint16_t expected_crc   = 0xd26du; /* spec: 0x6d 0xd2 (low, high) */

    uint16_t crc = bredr_payload_crc(data, sizeof(data) * 8u, uap);

    TEST_ASSERT(crc == expected_crc);

    if (crc != expected_crc)
    {
        fprintf(stderr, "  CRC mismatch: expected 0x%04X, got 0x%04X\n",
                expected_crc, crc);
    }

    /* Sanity: the same payload with a different UAP must produce a
     * different CRC -- the UAP seeds the register and a change must
     * ripple through the LFSR. */
    uint16_t crc_other_uap = bredr_payload_crc(data, sizeof(data) * 8u, 0x00u);
    TEST_ASSERT(crc_other_uap != expected_crc);

    if (g_failures != 0)
    {
        fprintf(stderr, "test_crc: %d assertion(s) failed\n", g_failures);
        return 1;
    }

    puts("test_crc: BT spec payload CRC (UAP=0x47) matches 0x6DD2");
    return 0;
}
