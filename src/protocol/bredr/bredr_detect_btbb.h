/**
 * @file bredr_detect_btbb.h
 * @brief Repository-owned wrapper for transitional libbtbb BR/EDR detection.
 */

#ifndef BREDR_DETECT_BTBB_H
#define BREDR_DETECT_BTBB_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    int found;
    int offset_bits;
    uint32_t lap;
    int ac_errors;
} bredr_detect_result_t;

void bredr_detect_btbb_init(uint8_t max_ac_errors);
void bredr_detect_btbb_init_survey(void);

int bredr_detect_btbb_find_access_code(const uint8_t *bits,
                                       unsigned int bit_count,
                                       int channel,
                                       uint32_t clkn,
                                       bredr_detect_result_t *out);

#ifdef __cplusplus
}
#endif

#endif /* BREDR_DETECT_BTBB_H */
