#include "receiver_dsp.h"

#include <stddef.h>
#include <stdlib.h>
#include <string.h>

rx_metadata_t receiver_make_metadata(uint64_t start_sample,
                                     uint32_t center_frequency_hz,
                                     uint16_t channel_index,
                                     float rssi_dbr,
                                     uint8_t confidence)
{
    rx_metadata_t meta = {
        .source_id = RECEIVER_SOURCE_ID_DEFAULT,
        .start_sample = start_sample,
        .center_frequency_hz = center_frequency_hz,
        .channel_index = channel_index,
        .rssi_dbr = rssi_dbr,
        .confidence = confidence,
    };
    return meta;
}

void receiver_fill_bredr_piconet_snapshot(const bredr_piconet_t *pnet,
                                          receiver_bredr_piconet_snapshot_t *out)
{
    if (!pnet || !out)
        return;

    memset(out, 0, sizeof(*out));
    out->lap = pnet->lap;
    out->uap_found = pnet->uap_found;
    out->uap = pnet->uap;
    out->clk_known = pnet->clk_known;
    out->central_clk_1_6 = pnet->central_clk_1_6;
    out->last_successful_rx_clk_1600 = pnet->last_successful_rx_clk_1600;
    out->tracking_state = pnet->tracking_state;
    out->total_packets = pnet->total_packets;
    out->combined_rssi_seen = pnet->combined_rssi_seen;
    out->combined_rssi = pnet->combined_rssi;
    out->master_rssi_seen = pnet->master_rssi_seen;
    out->master_rssi = pnet->master_rssi;
    memcpy(out->slave_rssi_seen, pnet->slave_rssi_seen, sizeof(out->slave_rssi_seen));
    memcpy(out->slave_rssi, pnet->slave_rssi, sizeof(out->slave_rssi));
}

void receiver_bredr_session_init(receiver_session_t *session,
                                 const receiver_bredr_config_t *config,
                                 const receiver_bredr_callbacks_t *callbacks)
{
    memset(&session->bredr_store, 0, sizeof(session->bredr_store));
    memset(session->bredr_ctx, 0, RECEIVER_BREDR_MAX_CHANNELS * sizeof(*session->bredr_ctx));
    memset(session->bredr_block_pool, 0, RECEIVER_BREDR_BLOCK_POOL_SIZE * sizeof(*session->bredr_block_pool));
    session->stop_requested = 0;
    session->debug = config->debug;
    session->bredr_config = *config;
    session->bredr_worker_threads = NULL;
    session->bredr_worker_count = 0u;
    session->bredr_samples_received = 0ULL;
    session->bredr_shutdown_requested = 0u;
    session->bredr_pool_write_idx = 0u;
    session->bredr_dropped_blocks = 0ul;
    session->bredr_total_bits = 0ULL;
    session->bredr_total_packets = 0ul;
    session->bredr_header_packets = 0ul;
    session->bredr_id_packets = 0ul;
    if (callbacks)
        session->bredr_callbacks = *callbacks;
    else
        memset(&session->bredr_callbacks, 0, sizeof(session->bredr_callbacks));

    receiver_bredr_update_layout(session);
    bredr_piconet_store_init(&session->bredr_store);
    bredr_piconet_store_set_rssi_averaging(&session->bredr_store, config->rssi_averaging_window);
}

receiver_session_t *receiver_session_create(void)
{
    receiver_session_t *session = (receiver_session_t *)calloc(1, sizeof(*session));
    if (!session)
        return NULL;

    session->pkt_start_abs = -1;
    session->prev_status = BLE_SEARCHING;
    pthread_mutex_init(&session->decoded_packet_mutex, NULL);
    pthread_mutex_init(&session->stop_mutex, NULL);
    pthread_cond_init(&session->stop_cv, NULL);

    session->raw = (float complex *)calloc(RECEIVER_BLE_BUFFER_SIZE / 2u, sizeof(*session->raw));
    session->bredr_ctx = (receiver_bredr_channel_ctx_t *)calloc(RECEIVER_BREDR_MAX_CHANNELS,
                                                                sizeof(*session->bredr_ctx));
    session->bredr_block_pool = (receiver_bredr_block_t *)calloc(RECEIVER_BREDR_BLOCK_POOL_SIZE,
                                                                  sizeof(*session->bredr_block_pool));
    session->hybrid_ble_ctx = (receiver_hybrid_ble_ctx_t *)calloc(1, sizeof(*session->hybrid_ble_ctx));
    if (!session->raw || !session->bredr_ctx || !session->bredr_block_pool || !session->hybrid_ble_ctx)
    {
        receiver_session_destroy(session);
        return NULL;
    }

    return session;
}

void receiver_session_destroy(receiver_session_t *session)
{
    if (session)
    {
        pthread_cond_destroy(&session->stop_cv);
        pthread_mutex_destroy(&session->stop_mutex);
        pthread_mutex_destroy(&session->decoded_packet_mutex);
        free(session->hybrid_ble_ctx);
        free(session->bredr_block_pool);
        free(session->bredr_ctx);
        free(session->raw);
    }
    free(session);
}

void receiver_session_request_stop(receiver_session_t *session)
{
    if (!session)
        return;
    session->stop_requested = 1;
    /* pthread_cond_broadcast is async-signal-safe per POSIX (no mutex needed). */
    pthread_cond_broadcast(&session->stop_cv);
}

size_t receiver_session_bredr_piconet_count(receiver_session_t *session)
{
    if (!session)
        return 0u;

    pthread_mutex_lock(&session->decoded_packet_mutex);
    size_t count = bredr_piconet_store_count(&session->bredr_store);
    pthread_mutex_unlock(&session->decoded_packet_mutex);
    return count;
}

int receiver_session_bredr_piconet_snapshot(receiver_session_t *session,
                                            size_t index,
                                            receiver_bredr_piconet_snapshot_t *out)
{
    if (!session || !out)
        return -1;

    pthread_mutex_lock(&session->decoded_packet_mutex);
    const bredr_piconet_t *pnet = bredr_piconet_store_get(&session->bredr_store, index);
    if (!pnet)
    {
        pthread_mutex_unlock(&session->decoded_packet_mutex);
        return -1;
    }

    receiver_fill_bredr_piconet_snapshot(pnet, out);
    pthread_mutex_unlock(&session->decoded_packet_mutex);
    return 0;
}
