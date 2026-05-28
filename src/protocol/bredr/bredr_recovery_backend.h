/**
 * @file bredr_recovery_backend.h
 * @brief Internal backend interface for BR/EDR recovery implementations.
 */

#ifndef BREDR_RECOVERY_BACKEND_H
#define BREDR_RECOVERY_BACKEND_H

#include <stdint.h>

#include "bredr_bitstream_decoder.h"
#include "bredr_recovery.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef void bredr_recovery_backend_state_t;

typedef struct
{
    void (*global_init)(uint8_t max_ac_errors);
    bredr_recovery_backend_state_t *(*state_create)(uint32_t lap);
    void (*state_destroy)(bredr_recovery_backend_state_t *state);
    void (*state_reset)(bredr_recovery_backend_state_t *state, uint32_t lap);
    int (*process_packet)(bredr_recovery_backend_state_t *state,
                          const bredr_frame_t *frame,
                          int channel,
                          uint32_t clkn,
                          bredr_recovery_result_t *out);
} bredr_recovery_backend_ops_t;

const bredr_recovery_backend_ops_t *bredr_recovery_default_backend(void);

#ifdef __cplusplus
}
#endif

#endif /* BREDR_RECOVERY_BACKEND_H */
