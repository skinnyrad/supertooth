#include "receiver_dsp.h"

#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

static void *receiver_hybrid_bredr_worker(void *arg)
{
    receiver_hybrid_bredr_ctx_t *ctx = (receiver_hybrid_bredr_ctx_t *)arg;
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
        receiver_hybrid_process_bredr(ctx, &session->hybrid_block_pool[block_idx]);
        __atomic_sub_fetch(&session->hybrid_block_pool[block_idx].refcount, 1u, __ATOMIC_ACQ_REL);
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
        receiver_hybrid_process_ble(ble, &session->hybrid_block_pool[block_idx]);
        __atomic_sub_fetch(&session->hybrid_block_pool[block_idx].refcount, 1u, __ATOMIC_ACQ_REL);
    }
    return NULL;
}


int receiver_session_run_hybrid(receiver_session_t *session,
                                const receiver_hybrid_config_t *config,
                                const receiver_hybrid_callbacks_t *callbacks,
                                receiver_hybrid_stats_t *stats_out)
{
    if (!session || !config)
        return -1;
    session->stop_requested = 0;
    session->debug = config->debug;
    session->hybrid_config = *config;
    session->hybrid_total_packets = 0;
    session->hybrid_dropped_blocks = 0;
    session->hybrid_samples_received = 0;
    session->hybrid_shutdown_requested = 0;
    session->hybrid_pool_write_idx = 0;
    if (callbacks) session->hybrid_callbacks = *callbacks;
    else memset(&session->hybrid_callbacks, 0, sizeof(session->hybrid_callbacks));
    if (receiver_hybrid_setup(session) != 0)
        return -1;

    bredr_detect_btbb_init(2u);
    bredr_detect_btbb_init_survey();

    session->hybrid_worker_count = RECEIVER_BREDR_MAX_CHANNELS + 1u;
    session->hybrid_worker_threads = (pthread_t *)calloc(session->hybrid_worker_count, sizeof(pthread_t));
    if (!session->hybrid_worker_threads)
    {
        receiver_hybrid_destroy(session);
        return -1;
    }
    for (unsigned int i = 0; i < RECEIVER_BREDR_MAX_CHANNELS; i++)
        pthread_create(&session->hybrid_worker_threads[i], NULL, receiver_hybrid_bredr_worker, &session->hybrid_bredr_ctx[i]);
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
        pthread_mutex_lock(&session->hybrid_bredr_ctx[i].queue_mutex);
        pthread_cond_signal(&session->hybrid_bredr_ctx[i].queue_cv);
        pthread_mutex_unlock(&session->hybrid_bredr_ctx[i].queue_mutex);
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
            stats_out->bredr_channel_dropped_blocks[i] = session->hybrid_bredr_ctx[i].dropped_blocks;
        stats_out->ble_dropped_blocks = session->hybrid_ble_ctx.dropped_blocks;
    }

    receiver_hybrid_destroy(session);
    return result;
}
