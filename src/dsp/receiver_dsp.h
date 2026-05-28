#ifndef RECEIVER_DSP_H
#define RECEIVER_DSP_H

#include "receiver_session_internal.h"

int receiver_dispatcher_rx_cb(hackrf_transfer *transfer);
int receiver_ble_setup(receiver_session_t *session,
                       receiver_ble_pipeline_t pipeline);
void receiver_ble_destroy(receiver_session_t *session);
void receiver_ble_process_block(receiver_ble_ctx_t *ble,
                                sample_block_t *blk);

void receiver_bredr_update_layout(receiver_session_t *session);
int receiver_bredr_setup_channel_ctx(receiver_session_t *session);
void receiver_bredr_destroy_channel_ctx(receiver_session_t *session);
void receiver_bredr_process_channel(receiver_bredr_channel_ctx_t *ctx,
                                                sample_block_t *blk);

/**
 * @brief Convert a HackRF interleaved int8_t IQ sample to normalised float complex.
 *
 * @param s  Interleaved I/Q byte buffer from hackrf_transfer.buffer.
 * @param i  Sample index.
 * @return   Complex sample with I and Q in [-1.0, 1.0).
 */
static inline float complex hackrf_iq_to_complex(const int8_t *s, unsigned int i)
{
    return s[2u * i] / 128.0f + (s[2u * i + 1u] / 128.0f) * _Complex_I;
}

#endif
