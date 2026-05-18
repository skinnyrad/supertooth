#ifndef RECEIVER_DSP_H
#define RECEIVER_DSP_H

#include "receiver_session_internal.h"

int receiver_btle_rx_cb(hackrf_transfer *transfer);

void receiver_bredr_update_layout(receiver_session_t *session);
int receiver_bredr_setup_channel_ctx(receiver_session_t *session);
void receiver_bredr_destroy_channel_ctx(receiver_session_t *session);
void receiver_bredr_process_channel(receiver_bredr_channel_ctx_t *ctx,
                                    const receiver_bredr_block_t *blk);
int receiver_bredr_rx_cb(hackrf_transfer *transfer);

int receiver_hybrid_setup(receiver_session_t *session);
void receiver_hybrid_destroy(receiver_session_t *session);
void receiver_hybrid_process_bredr(receiver_hybrid_bredr_ctx_t *ctx,
                                   const receiver_bredr_block_t *blk);
void receiver_hybrid_process_ble(receiver_hybrid_ble_ctx_t *ble,
                                 const receiver_bredr_block_t *blk);
int receiver_hybrid_cb(hackrf_transfer *transfer);

#endif
