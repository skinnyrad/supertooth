#include "receiver_dsp.h"

#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

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

static void receiver_hybrid_bredr_bridge(const bredr_event_t *event,
                                         const receiver_bredr_piconet_snapshot_t *piconet,
                                         void *user)
{
    receiver_session_t *session = (receiver_session_t *)user;
    if (!session || !event)
        return;

    receiver_hybrid_callbacks_t callbacks;
    receiver_bredr_piconet_snapshot_t snapshot;
    const receiver_bredr_piconet_snapshot_t *snapshot_ptr = NULL;

    pthread_mutex_lock(&session->decoded_packet_mutex);
    session->hybrid_total_packets++;
    callbacks = session->hybrid_callbacks;
    if (piconet)
    {
        snapshot = *piconet;
        snapshot_ptr = &snapshot;
    }
    pthread_mutex_unlock(&session->decoded_packet_mutex);

    if (callbacks.on_bredr_packet)
        callbacks.on_bredr_packet(event, snapshot_ptr, callbacks.user);
}

static int receiver_hybrid_start_thread_pool(receiver_session_t *session)
{
    session->hybrid_shutdown_requested = 0u;
    session->hybrid_worker_count = 0u;
    session->hybrid_worker_threads =
        (pthread_t *)calloc(RECEIVER_BREDR_MAX_CHANNELS + 1u, sizeof(pthread_t));
    if (!session->hybrid_worker_threads)
        return -1;

    for (unsigned int i = 0; i < RECEIVER_BREDR_MAX_CHANNELS; i++)
    {
        if (pthread_create(&session->hybrid_worker_threads[i], NULL,
                           receiver_hybrid_bredr_worker, &session->bredr_ctx[i]) != 0)
            return -1;
        session->hybrid_worker_count++;
    }

    if (pthread_create(&session->hybrid_worker_threads[RECEIVER_BREDR_MAX_CHANNELS], NULL,
                       receiver_hybrid_ble_worker, session->hybrid_ble_ctx) != 0)
        return -1;

    session->hybrid_worker_count++;
    return 0;
}

static void receiver_hybrid_stop_thread_pool(receiver_session_t *session)
{
    session->hybrid_shutdown_requested = 1u;
    for (unsigned int i = 0; i < RECEIVER_BREDR_MAX_CHANNELS; i++)
    {
        pthread_mutex_lock(&session->bredr_ctx[i].queue_mutex);
        pthread_cond_signal(&session->bredr_ctx[i].queue_cv);
        pthread_mutex_unlock(&session->bredr_ctx[i].queue_mutex);
    }
    pthread_mutex_lock(&session->hybrid_ble_ctx->queue_mutex);
    pthread_cond_signal(&session->hybrid_ble_ctx->queue_cv);
    pthread_mutex_unlock(&session->hybrid_ble_ctx->queue_mutex);
    for (unsigned int i = 0; i < session->hybrid_worker_count; i++)
        pthread_join(session->hybrid_worker_threads[i], NULL);
    free(session->hybrid_worker_threads);
    session->hybrid_worker_threads = NULL;
    session->hybrid_worker_count = 0u;
}


int receiver_session_run_hybrid(receiver_session_t *session,
                                const receiver_hybrid_config_t *config,
                                const receiver_hybrid_callbacks_t *callbacks,
                                receiver_hybrid_stats_t *stats_out)
{
    if (!session || !config)
        return -1;

    receiver_bredr_config_t bredr_config = {
        .channel_count = RECEIVER_BREDR_MAX_CHANNELS,
        .bottom_channel = 0u,
        .rssi_averaging_window = RECEIVER_BREDR_DEFAULT_RSSI_AVERAGING_WINDOW,
        .lap_filter = 0u,
        .lap_filter_enabled = 0,
        .debug = config->debug,
    };
    receiver_bredr_callbacks_t bredr_callbacks = {
        .on_packet = receiver_hybrid_bredr_bridge,
        .user = session,
    };
    session->hybrid_config = *config;
    session->hybrid_total_packets = 0;
    session->hybrid_dropped_blocks = 0;
    session->hybrid_shutdown_requested = 0;
    session->hybrid_worker_threads = NULL;
    session->hybrid_worker_count = 0u;
    if (callbacks)
        session->hybrid_callbacks = *callbacks;
    else
        memset(&session->hybrid_callbacks, 0, sizeof(session->hybrid_callbacks));
    receiver_bredr_session_init(session, &bredr_config, &bredr_callbacks);
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

    if (receiver_hybrid_start_thread_pool(session) != 0)
    {
        if (session->hybrid_worker_threads)
            receiver_hybrid_stop_thread_pool(session);
        receiver_hybrid_destroy_ble(session);
        receiver_bredr_destroy_channel_ctx(session);
        bredr_piconet_store_free(&session->bredr_store);
        return -1;
    }

    hackrf_device *device = NULL;
    int result = hackrf_connect(&device);
    if (result == HACKRF_SUCCESS)
    {
        hackrf_config_t radio_config = {
            .lo_freq_hz = RECEIVER_HYBRID_LO_FREQ_HZ,
            .sample_rate = RECEIVER_HYBRID_SAMPLE_RATE,
            .lna_gain = RECEIVER_BREDR_LNA_GAIN,
            .vga_gain = RECEIVER_BREDR_VGA_GAIN,
        };
        result = hackrf_configure(device, &radio_config);
        if (result == HACKRF_SUCCESS)
            result = hackrf_start_rx(device, receiver_hybrid_cb, session);
        if (result == HACKRF_SUCCESS)
        {
            /* Block until receiver_session_request_stop() signals stop_cv. */
            pthread_mutex_lock(&session->stop_mutex);
            while (!session->stop_requested)
            {
                struct timespec ts;
                clock_gettime(CLOCK_REALTIME, &ts);
                ts.tv_nsec += 200000000L;
                if (ts.tv_nsec >= 1000000000L)
                {
                    ts.tv_sec++;
                    ts.tv_nsec -= 1000000000L;
                }
                pthread_cond_timedwait(&session->stop_cv, &session->stop_mutex, &ts);
            }
            pthread_mutex_unlock(&session->stop_mutex);
            hackrf_stop_rx(device);
        }
        hackrf_disconnect(device);
    }

    receiver_hybrid_stop_thread_pool(session);

    if (stats_out)
    {
        memset(stats_out, 0, sizeof(*stats_out));
        stats_out->total_packets = session->hybrid_total_packets;
        stats_out->dropped_blocks = session->hybrid_dropped_blocks;
        stats_out->bredr_channel_count = RECEIVER_BREDR_MAX_CHANNELS;
        for (unsigned int i = 0; i < RECEIVER_BREDR_MAX_CHANNELS; i++)
            stats_out->bredr_channel_dropped_blocks[i] = session->bredr_ctx[i].dropped_blocks;
        stats_out->ble_dropped_blocks = session->hybrid_ble_ctx->dropped_blocks;
    }

    receiver_hybrid_destroy_ble(session);
    receiver_bredr_destroy_channel_ctx(session);
    bredr_piconet_store_free(&session->bredr_store);
    return result;
}
