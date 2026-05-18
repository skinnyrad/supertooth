#include "receiver_dsp.h"

#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

static void *receiver_hybrid_bredr_worker(void *arg)
{
    receiver_bredr_channel_ctx_t *ctx = (receiver_bredr_channel_ctx_t *)arg;
    receiver_session_t *session = ctx->session;
    for (;;)
    {
        pthread_mutex_lock(&ctx->queue_mutex);
        while (!session->hybrid_shutdown_requested && ctx->block_count == 0u)
            pthread_cond_wait(&ctx->queue_cv, &ctx->queue_mutex);
        if (session->hybrid_shutdown_requested)
        {
            pthread_mutex_unlock(&ctx->queue_mutex);
            break;
        }
        unsigned int block_idx = ctx->block_idx_ring[ctx->block_read_idx];
        ctx->block_read_idx = (ctx->block_read_idx + 1u) % RECEIVER_BREDR_CHANNEL_RING_SIZE;
        ctx->block_count--;
        pthread_mutex_unlock(&ctx->queue_mutex);
        receiver_bredr_process_channel(ctx, &session->bredr_block_pool[block_idx]);
        __atomic_sub_fetch(&session->bredr_block_pool[block_idx].refcount, 1u, __ATOMIC_ACQ_REL);
    }
    return NULL;
}

static void *receiver_hybrid_ble_worker(void *arg)
{
    receiver_hybrid_ble_ctx_t *ble = (receiver_hybrid_ble_ctx_t *)arg;
    receiver_session_t *session = ble->session;
    for (;;)
    {
        pthread_mutex_lock(&ble->queue_mutex);
        while (!session->hybrid_shutdown_requested && ble->block_count == 0u)
            pthread_cond_wait(&ble->queue_cv, &ble->queue_mutex);
        if (session->hybrid_shutdown_requested)
        {
            pthread_mutex_unlock(&ble->queue_mutex);
            break;
        }
        unsigned int block_idx = ble->block_idx_ring[ble->block_read_idx];
        ble->block_read_idx = (ble->block_read_idx + 1u) % RECEIVER_BREDR_CHANNEL_RING_SIZE;
        ble->block_count--;
        pthread_mutex_unlock(&ble->queue_mutex);
        receiver_hybrid_process_ble(ble, &session->bredr_block_pool[block_idx]);
        __atomic_sub_fetch(&session->bredr_block_pool[block_idx].refcount, 1u, __ATOMIC_ACQ_REL);
    }
    return NULL;
}

static void receiver_hybrid_bredr_bridge(const decoded_packet_t *packet,
                                         const receiver_bredr_piconet_snapshot_t *piconet,
                                         void *user)
{
    (void)piconet;
    receiver_session_t *session = (receiver_session_t *)user;
    if (!session || !packet || packet->protocol != PROTO_BREDR)
        return;

    session->hybrid_total_packets++;
    if (!session->hybrid_callbacks.on_bredr_packet)
        return;

    const bredr_packet_t *pkt = &packet->u.bredr;
    uint32_t clkn = receiver_bredr_sample_to_clkn(session, packet->meta.start_sample);
    session->hybrid_callbacks.on_bredr_packet(packet->meta.channel_index,
                                              pkt->lap,
                                              clkn,
                                              pkt->ac_errors,
                                              &packet->meta,
                                              session->hybrid_callbacks.user);
}


int receiver_session_run_hybrid(receiver_session_t *session,
                                const receiver_hybrid_config_t *config,
                                const receiver_hybrid_callbacks_t *callbacks,
                                receiver_hybrid_stats_t *stats_out)
{
    if (!session || !config)
        return -1;
    memset(&session->bredr_store, 0, sizeof(session->bredr_store));
    memset(session->bredr_ctx, 0, sizeof(session->bredr_ctx));
    memset(session->bredr_block_pool, 0, sizeof(session->bredr_block_pool));
    session->stop_requested = 0;
    session->debug = config->debug;
    session->hybrid_config = *config;
    session->hybrid_total_packets = 0;
    session->hybrid_dropped_blocks = 0;
    session->hybrid_shutdown_requested = 0;
    session->bredr_samples_received = 0ULL;
    session->bredr_pool_write_idx = 0u;
    session->bredr_dropped_blocks = 0ul;
    session->bredr_total_bits = 0ULL;
    session->bredr_total_packets = 0ul;
    session->bredr_header_packets = 0ul;
    session->bredr_id_packets = 0ul;
    session->bredr_config.channel_count = RECEIVER_BREDR_MAX_CHANNELS;
    session->bredr_config.bottom_channel = 0u;
    session->bredr_config.rssi_averaging_window = 16u;
    session->bredr_config.lap_filter = 0u;
    session->bredr_config.lap_filter_enabled = 0;
    session->bredr_config.debug = config->debug;
    if (callbacks) session->hybrid_callbacks = *callbacks;
    else memset(&session->hybrid_callbacks, 0, sizeof(session->hybrid_callbacks));
    session->bredr_callbacks.on_packet = receiver_hybrid_bredr_bridge;
    session->bredr_callbacks.user = session;
    receiver_bredr_update_layout(session);
    bredr_piconet_set_rssi_averaging(session->bredr_config.rssi_averaging_window);
    bredr_piconet_store_init(&session->bredr_store);
    if (receiver_bredr_setup_channel_ctx(session) != 0)
    {
        bredr_piconet_store_free(&session->bredr_store);
        receiver_bredr_destroy_channel_ctx(session);
        return -1;
    }
    if (receiver_hybrid_setup_ble(session) != 0)
    {
        receiver_hybrid_destroy_ble(session);
        receiver_bredr_destroy_channel_ctx(session);
        bredr_piconet_store_free(&session->bredr_store);
        return -1;
    }

    session->hybrid_worker_count = RECEIVER_BREDR_MAX_CHANNELS + 1u;
    session->hybrid_worker_threads = (pthread_t *)calloc(session->hybrid_worker_count, sizeof(pthread_t));
    if (!session->hybrid_worker_threads)
    {
        receiver_hybrid_destroy_ble(session);
        receiver_bredr_destroy_channel_ctx(session);
        bredr_piconet_store_free(&session->bredr_store);
        return -1;
    }
    for (unsigned int i = 0; i < RECEIVER_BREDR_MAX_CHANNELS; i++)
        pthread_create(&session->hybrid_worker_threads[i], NULL, receiver_hybrid_bredr_worker, &session->bredr_ctx[i]);
    pthread_create(&session->hybrid_worker_threads[RECEIVER_BREDR_MAX_CHANNELS], NULL, receiver_hybrid_ble_worker, &session->hybrid_ble_ctx);

    hackrf_device *device = NULL;
    int result = hackrf_connect(&device);
    if (result == HACKRF_SUCCESS)
    {
        hackrf_config_t radio_config = {
            .lo_freq_hz = 2411500000ULL,
            .sample_rate = 20000000u,
            .lna_gain = 32u,
            .vga_gain = 32u,
        };
        result = hackrf_configure(device, &radio_config);
        if (result == HACKRF_SUCCESS)
            result = hackrf_start_rx(device, receiver_hybrid_cb, session);
        if (result == HACKRF_SUCCESS)
        {
            while (!session->stop_requested)
                sleep(1);
            hackrf_stop_rx(device);
        }
        hackrf_disconnect(device);
    }

    session->hybrid_shutdown_requested = 1u;
    for (unsigned int i = 0; i < RECEIVER_BREDR_MAX_CHANNELS; i++)
    {
        pthread_mutex_lock(&session->bredr_ctx[i].queue_mutex);
        pthread_cond_signal(&session->bredr_ctx[i].queue_cv);
        pthread_mutex_unlock(&session->bredr_ctx[i].queue_mutex);
    }
    pthread_mutex_lock(&session->hybrid_ble_ctx.queue_mutex);
    pthread_cond_signal(&session->hybrid_ble_ctx.queue_cv);
    pthread_mutex_unlock(&session->hybrid_ble_ctx.queue_mutex);
    for (unsigned int i = 0; i < session->hybrid_worker_count; i++)
        pthread_join(session->hybrid_worker_threads[i], NULL);
    free(session->hybrid_worker_threads);
    session->hybrid_worker_threads = NULL;

    if (stats_out)
    {
        memset(stats_out, 0, sizeof(*stats_out));
        stats_out->total_packets = session->hybrid_total_packets;
        stats_out->dropped_blocks = session->hybrid_dropped_blocks;
        stats_out->bredr_channel_count = RECEIVER_BREDR_MAX_CHANNELS;
        for (unsigned int i = 0; i < RECEIVER_BREDR_MAX_CHANNELS; i++)
            stats_out->bredr_channel_dropped_blocks[i] = session->bredr_ctx[i].dropped_blocks;
        stats_out->ble_dropped_blocks = session->hybrid_ble_ctx.dropped_blocks;
    }

    receiver_hybrid_destroy_ble(session);
    receiver_bredr_destroy_channel_ctx(session);
    bredr_piconet_store_free(&session->bredr_store);
    return result;
}
