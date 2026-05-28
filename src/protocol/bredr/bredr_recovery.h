/**
 * @file bredr_recovery.h
 * @brief Backend-neutral BR/EDR recovery interface owned by Supertooth.
 */

#ifndef BREDR_RECOVERY_H
#define BREDR_RECOVERY_H

#include <stdint.h>

#include "bredr_bitstream_decoder.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct bredr_recovery_state bredr_recovery_state_t;

typedef struct
{
    uint8_t uap;
    uint8_t clk6_hint;
} bredr_recovery_result_t;

void bredr_recovery_global_init(uint8_t max_ac_errors);
bredr_recovery_state_t *bredr_recovery_state_create(uint32_t lap);
void bredr_recovery_state_destroy(bredr_recovery_state_t *state);
void bredr_recovery_state_reset(bredr_recovery_state_t *state, uint32_t lap);

int bredr_recovery_process_packet(bredr_recovery_state_t *state,
                                  const bredr_frame_t *frame,
                                  int channel,
                                  uint32_t clkn,
                                  bredr_recovery_result_t *out);

#ifdef __cplusplus
}
#endif

#endif /* BREDR_RECOVERY_H */
