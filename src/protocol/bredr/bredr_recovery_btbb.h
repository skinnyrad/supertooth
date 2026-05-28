/**
 * @file bredr_recovery_btbb.h
 * @brief Repository-owned wrapper around the transitional libbtbb backend.
 */

#ifndef BREDR_RECOVERY_BTBB_H
#define BREDR_RECOVERY_BTBB_H

#include <stdint.h>

#include "bredr_bitstream_decoder.h"
#include "bredr_codec.h"
#include "bredr_recovery_backend.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct bredr_recovery_btbb_state bredr_recovery_btbb_state_t;

void bredr_recovery_btbb_global_init(uint8_t max_ac_errors);
bredr_recovery_btbb_state_t *bredr_recovery_btbb_state_create(uint32_t lap);
void bredr_recovery_btbb_state_destroy(bredr_recovery_btbb_state_t *state);
void bredr_recovery_btbb_state_reset(bredr_recovery_btbb_state_t *state, uint32_t lap);

int bredr_recovery_btbb_process_packet(bredr_recovery_btbb_state_t *state,
                                       const bredr_frame_t *frame,
                                       int channel,
                                       uint32_t clkn,
                                       uint8_t *uap_out,
                                       uint8_t *clk6_hint_out);

const bredr_recovery_backend_ops_t *bredr_recovery_btbb_backend(void);

#ifdef __cplusplus
}
#endif

#endif /* BREDR_RECOVERY_BTBB_H */
