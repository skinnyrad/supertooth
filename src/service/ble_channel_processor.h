#ifndef BLE_CHANNEL_PROCESSOR_H
#define BLE_CHANNEL_PROCESSOR_H

#include "receiver_session.h"

int receiver_ble_channel_processor_setup(receiver_session_t *session,
                                        receiver_ble_pipeline_t pipeline);
void receiver_ble_channel_processor_destroy(receiver_session_t *session);
void receiver_ble_channel_processor_process(ble_channel_processor_t *ble,
                                           sample_block_t *blk);

#endif
