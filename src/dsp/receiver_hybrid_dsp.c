#include "receiver_dsp.h"
#include "rssi_measurements.h"

#include <math.h>
#include <string.h>

int receiver_hybrid_setup(receiver_session_t *session)
{
    float lowest_ctx_freq_offset =
        -(RECEIVER_BREDR_MAX_CHANNELS / 2.0f - 0.5f) * (float)RECEIVER_BREDR_CHANNEL_BW;

    for (unsigned int i = 0; i < RECEIVER_BREDR_MAX_CHANNELS; i++)
    {
        receiver_hybrid_bredr_ctx_t *ctx = &session->hybrid_bredr_ctx[i];
        memset(ctx, 0, sizeof(*ctx));
        ctx->session = session;
        ctx->ctx_index = i;
        ctx->bredr_channel = i;
        float channel_offset_freq = i * (float)RECEIVER_BREDR_CHANNEL_BW + lowest_ctx_freq_offset;
        float normalized_freq = 2.0f * (float)M_PI * channel_offset_freq / 20000000.0f;
        ctx->nco = nco_crcf_create(LIQUID_NCO);
        ctx->firdec = firdecim_crcf_create_kaiser(10, 7, 60.0f);
        ctx->demod = cpfskdem_create(1, 0.3f, 2, 3, 0.5f, LIQUID_CPFSK_GMSK);
        if (!ctx->nco || !ctx->firdec || !ctx->demod)
            return -1;
        nco_crcf_set_frequency(ctx->nco, normalized_freq);
        if (pthread_mutex_init(&ctx->queue_mutex, NULL) != 0)
            return -1;
        if (pthread_cond_init(&ctx->queue_cv, NULL) != 0)
        {
            pthread_mutex_destroy(&ctx->queue_mutex);
            return -1;
        }
    }

    receiver_hybrid_ble_ctx_t *ble = &session->hybrid_ble_ctx;
    memset(ble, 0, sizeof(*ble));
    ble->session = session;
    ble->nco = nco_crcf_create(LIQUID_NCO);
    ble->firdec = firdecim_crcf_create_kaiser(10, 7, 60.0f);
    ble->demod = cpfskdem_create(1, 0.5f, 2, 3, 0.5f, LIQUID_CPFSK_GMSK);
    if (!ble->nco || !ble->firdec || !ble->demod)
        return -1;
    nco_crcf_set_frequency(ble->nco, 2.0f * (float)M_PI * (-9.5e6f) / 20000000.0f);
    ble_processor_init(&ble->ble_proc, 37u);
    ble->prev_status = BLE_SEARCHING;
    ble->pkt_start_decimated = -1;
    if (pthread_mutex_init(&ble->queue_mutex, NULL) != 0)
        return -1;
    if (pthread_cond_init(&ble->queue_cv, NULL) != 0)
    {
        pthread_mutex_destroy(&ble->queue_mutex);
        return -1;
    }
    return 0;
}

void receiver_hybrid_destroy(receiver_session_t *session)
{
    for (unsigned int i = 0; i < RECEIVER_BREDR_MAX_CHANNELS; i++)
    {
        receiver_hybrid_bredr_ctx_t *ctx = &session->hybrid_bredr_ctx[i];
        if (ctx->demod) cpfskdem_destroy(ctx->demod);
        if (ctx->firdec) firdecim_crcf_destroy(ctx->firdec);
        if (ctx->nco) nco_crcf_destroy(ctx->nco);
        pthread_cond_destroy(&ctx->queue_cv);
        pthread_mutex_destroy(&ctx->queue_mutex);
        ctx->demod = NULL; ctx->firdec = NULL; ctx->nco = NULL;
    }
    receiver_hybrid_ble_ctx_t *ble = &session->hybrid_ble_ctx;
    if (ble->demod) cpfskdem_destroy(ble->demod);
    if (ble->firdec) firdecim_crcf_destroy(ble->firdec);
    if (ble->nco) nco_crcf_destroy(ble->nco);
    pthread_cond_destroy(&ble->queue_cv);
    pthread_mutex_destroy(&ble->queue_mutex);
    ble->demod = NULL; ble->firdec = NULL; ble->nco = NULL;
}

void receiver_hybrid_process_bredr(receiver_hybrid_bredr_ctx_t *ctx,
                                   const receiver_bredr_block_t *blk)
{
    receiver_session_t *session = ctx->session;
    nco_crcf_mix_block_down(ctx->nco, blk->samples, ctx->mixed, blk->num_samples);
    unsigned int decimated_samples = blk->num_samples / 10u;
    firdecim_crcf_execute_block(ctx->firdec, ctx->mixed, decimated_samples, ctx->decimated);
    for (unsigned int i = 0; i < decimated_samples; i += 2u)
    {
        unsigned int bit = cpfskdem_demodulate(ctx->demod, &ctx->decimated[i]);
        ctx->bits[i / 2u] = (uint8_t)(bit & 1u);
    }

    unsigned long long clkn_base_sample = blk->block_base_sample;
    uint32_t clkn_guess = (uint32_t)((clkn_base_sample / 20ULL) / 312.5);
    bredr_detect_result_t detect = {0};
    if (bredr_detect_btbb_find_access_code(ctx->bits, decimated_samples / 2u,
                                           (int)ctx->bredr_channel, clkn_guess, &detect) <= 0)
        return;

    unsigned long long absolute_sample = blk->block_base_sample + (unsigned long long)(detect.offset_bits * 20);
    unsigned long long bit_position = absolute_sample / 20ULL;
    uint32_t clkn = (uint32_t)(bit_position / 312.5);
    unsigned int rssi_start = (unsigned int)detect.offset_bits * 2u;
    unsigned int rssi_end = rssi_start + 144u;
    if (rssi_end > decimated_samples)
        rssi_end = decimated_samples;
    float rssi_dbr = receiver_rssi_from_mean_power_range(ctx->decimated, rssi_start, rssi_end, 0.0f);
    int ac_errors = detect.ac_errors;
    if (ac_errors < 0)
        ac_errors = 0;
    uint8_t confidence = (ac_errors >= 255) ? 0u : (uint8_t)(255 - ac_errors);
    rx_metadata_t meta = receiver_make_metadata(absolute_sample,
                                                (uint32_t)(RECEIVER_BREDR_CHANNEL_0_FREQ +
                                                           (double)ctx->bredr_channel *
                                                               RECEIVER_BREDR_CHANNEL_BW),
                                                (uint16_t)ctx->bredr_channel,
                                                rssi_dbr,
                                                confidence);
    pthread_mutex_lock(&session->bredr_packet_mutex);
    session->hybrid_total_packets++;
    if (session->hybrid_callbacks.on_bredr_packet)
        session->hybrid_callbacks.on_bredr_packet(ctx->ctx_index, detect.lap,
                                                  clkn, detect.ac_errors, &meta,
                                                  session->hybrid_callbacks.user);
    pthread_mutex_unlock(&session->bredr_packet_mutex);
}

void receiver_hybrid_process_ble(receiver_hybrid_ble_ctx_t *ble,
                                 const receiver_bredr_block_t *blk)
{
    receiver_session_t *session = ble->session;
    nco_crcf_mix_block_down(ble->nco, blk->samples, ble->mixed, blk->num_samples);
    unsigned int decimated_samples = blk->num_samples / 10u;
    firdecim_crcf_execute_block(ble->firdec, ble->mixed, decimated_samples, ble->decimated);
    unsigned long long buf_start = blk->block_base_sample / 10u;
    for (unsigned int i = 0; i < decimated_samples; i += 2u)
    {
        unsigned int sym = cpfskdem_demodulate(ble->demod, &ble->decimated[i]);
        uint8_t bit = (uint8_t)(sym & 0x01u);
        ble_status_t status = ble_push_bit(&ble->ble_proc, bit);
        if (ble->prev_status == BLE_SEARCHING && status == BLE_COLLECTING)
            ble->pkt_start_decimated = (long long)(buf_start + i);
        ble->prev_status = status;
        if (status != BLE_VALID_PACKET)
            continue;
        unsigned int rssi_start = 0u;
        unsigned int rssi_end = 0u;
        if (ble->pkt_start_decimated >= 0)
        {
            long long rel = ble->pkt_start_decimated - (long long)buf_start;
            rssi_start = (rel < 0) ? 0u : (unsigned int)rel;
            rssi_end = i + 2u;
            if (rssi_end > decimated_samples)
                rssi_end = decimated_samples;
        }
        ble_packet_t pkt;
        if (ble_get_packet(&ble->ble_proc, &pkt) != 0)
            continue;
        float rssi_dbr = receiver_rssi_from_mean_power_range(ble->decimated, rssi_start, rssi_end, 0.0f);
        unsigned long long abs_sample = (buf_start + i) * 10ULL;
        rx_metadata_t meta = receiver_make_metadata(abs_sample,
                                                    2402000000u,
                                                    37u,
                                                    rssi_dbr,
                                                    255u);
        pthread_mutex_lock(&session->bredr_packet_mutex);
        session->hybrid_total_packets++;
        if (session->hybrid_callbacks.on_ble_packet)
            session->hybrid_callbacks.on_ble_packet(&pkt, &meta,
                                                    session->hybrid_callbacks.user);
        pthread_mutex_unlock(&session->bredr_packet_mutex);
    }
}

int receiver_hybrid_cb(hackrf_transfer *transfer)
{
    receiver_session_t *session = (receiver_session_t *)transfer->rx_ctx;
    if (!session || session->stop_requested)
        return -1;
    unsigned int num_samples = (unsigned int)(transfer->valid_length / 2u);
    if (num_samples > RECEIVER_BREDR_BUFFER_SIZE)
        num_samples = RECEIVER_BREDR_BUFFER_SIZE;
    int block_idx = -1;
    for (unsigned int i = 0; i < RECEIVER_BREDR_BLOCK_POOL_SIZE; i++)
    {
        unsigned int idx = (session->hybrid_pool_write_idx + i) % RECEIVER_BREDR_BLOCK_POOL_SIZE;
        if (__atomic_load_n(&session->hybrid_block_pool[idx].refcount, __ATOMIC_ACQUIRE) == 0u)
        {
            block_idx = (int)idx;
            break;
        }
    }
    if (block_idx < 0)
    {
        session->hybrid_dropped_blocks++;
        return 0;
    }
    receiver_bredr_block_t *blk = &session->hybrid_block_pool[(unsigned int)block_idx];
    blk->num_samples = num_samples;
    blk->block_base_sample = session->hybrid_samples_received;
    blk->refcount = 0u;
    session->hybrid_samples_received += num_samples;
    int8_t *samples = (int8_t *)transfer->buffer;
    for (unsigned int i = 0; i < num_samples; i++)
        blk->samples[i] = samples[2u * i] / 128.0f + (samples[2u * i + 1u] / 128.0f) * _Complex_I;
    session->hybrid_pool_write_idx = ((unsigned int)block_idx + 1u) % RECEIVER_BREDR_BLOCK_POOL_SIZE;
    __atomic_thread_fence(__ATOMIC_RELEASE);
    for (unsigned int ch = 0; ch < RECEIVER_BREDR_MAX_CHANNELS; ch++)
    {
        receiver_hybrid_bredr_ctx_t *ctx = &session->hybrid_bredr_ctx[ch];
        pthread_mutex_lock(&ctx->queue_mutex);
        if (ctx->block_count == RECEIVER_BREDR_CHANNEL_RING_SIZE)
        {
            unsigned int old_idx = ctx->block_idx_ring[ctx->block_read_idx];
            ctx->block_read_idx = (ctx->block_read_idx + 1u) % RECEIVER_BREDR_CHANNEL_RING_SIZE;
            ctx->block_count--;
            ctx->dropped_blocks++;
            __atomic_sub_fetch(&session->hybrid_block_pool[old_idx].refcount, 1u, __ATOMIC_ACQ_REL);
        }
        ctx->block_idx_ring[ctx->block_write_idx] = (unsigned int)block_idx;
        ctx->block_write_idx = (ctx->block_write_idx + 1u) % RECEIVER_BREDR_CHANNEL_RING_SIZE;
        ctx->block_count++;
        __atomic_add_fetch(&session->hybrid_block_pool[(unsigned int)block_idx].refcount, 1u, __ATOMIC_ACQ_REL);
        pthread_cond_signal(&ctx->queue_cv);
        pthread_mutex_unlock(&ctx->queue_mutex);
    }
    receiver_hybrid_ble_ctx_t *ble = &session->hybrid_ble_ctx;
    pthread_mutex_lock(&ble->queue_mutex);
    if (ble->block_count == RECEIVER_BREDR_CHANNEL_RING_SIZE)
    {
        unsigned int old_idx = ble->block_idx_ring[ble->block_read_idx];
        ble->block_read_idx = (ble->block_read_idx + 1u) % RECEIVER_BREDR_CHANNEL_RING_SIZE;
        ble->block_count--;
        ble->dropped_blocks++;
        __atomic_sub_fetch(&session->hybrid_block_pool[old_idx].refcount, 1u, __ATOMIC_ACQ_REL);
    }
    ble->block_idx_ring[ble->block_write_idx] = (unsigned int)block_idx;
    ble->block_write_idx = (ble->block_write_idx + 1u) % RECEIVER_BREDR_CHANNEL_RING_SIZE;
    ble->block_count++;
    __atomic_add_fetch(&session->hybrid_block_pool[(unsigned int)block_idx].refcount, 1u, __ATOMIC_ACQ_REL);
    pthread_cond_signal(&ble->queue_cv);
    pthread_mutex_unlock(&ble->queue_mutex);
    return 0;
}
