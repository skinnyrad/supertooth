#ifndef BREDR_CHANNEL_PROCESSOR_H
#define BREDR_CHANNEL_PROCESSOR_H

#include "receiver_session.h"

void receiver_bredr_update_layout(receiver_session_t *session);
int receiver_bredr_channel_processor_setup(receiver_session_t *session);
void receiver_bredr_channel_processor_destroy(receiver_session_t *session);
void receiver_bredr_channel_processor_process(bredr_channel_processor_t *ctx,
                                             sample_block_t *blk);

#endif
