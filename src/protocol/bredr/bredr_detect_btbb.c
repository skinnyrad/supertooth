/**
 * @file bredr_detect_btbb.c
 * @brief Transitional libbtbb-backed BR/EDR access-code detection wrapper.
 */

#include "bredr_detect_btbb.h"

#include <btbb.h>
#include <stddef.h>
#include <string.h>

void bredr_detect_btbb_init(uint8_t max_ac_errors)
{
    btbb_init(max_ac_errors);
}

void bredr_detect_btbb_init_survey(void)
{
    btbb_init_survey();
}

int bredr_detect_btbb_find_access_code(const uint8_t *bits,
                                       unsigned int bit_count,
                                       int channel,
                                       uint32_t clkn,
                                       bredr_detect_result_t *out)
{
    if (!bits || !out)
        return -1;
    memset(out, 0, sizeof(*out));
    memset(out, 0, sizeof(*out));

    btbb_packet *pkt = NULL;
    int offset = btbb_find_ac((char *)bits, (int)bit_count, LAP_ANY, 2, &pkt);
    if (offset < 0 || pkt == NULL)
        return 0;

    out->lap = btbb_packet_get_lap(pkt);
    if (out->lap != 0x9e8b33u)
    {
        btbb_packet_set_data(pkt, (char *)bits + offset, 400, (uint8_t)channel, clkn);
        out->ac_errors = btbb_packet_get_ac_errors(pkt);
        out->offset_bits = offset;
        out->found = 1;
    }

    btbb_packet_unref(pkt);
    return out->found ? 1 : 0;
}
