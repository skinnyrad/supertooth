#include "receiver_dsp.h"
#include "rssi_measurements.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

void receiver_bredr_update_layout(receiver_session_t *session)
{
    unsigned int channel_count = session->bredr_config.channel_count;
    session->bredr_sample_rate = (uint32_t)(channel_count * (unsigned int)RECEIVER_BREDR_CHANNEL_BW);
    if (channel_count == 2u)
        session->bredr_sample_rate = 4000000u;
    session->bredr_decim_factor = session->bredr_sample_rate / 2000000u;
    session->bredr_raw_samps_per_bit = session->bredr_decim_factor * RECEIVER_BREDR_SYMBOL_STEP;

    double lowest_ctx_freq_offset =
        -(channel_count / 2.0 - 0.5) * RECEIVER_BREDR_CHANNEL_BW;
    double lowest_channel_freq_hz =
        RECEIVER_BREDR_CHANNEL_0_FREQ +
        session->bredr_config.bottom_channel * RECEIVER_BREDR_CHANNEL_BW;
    session->bredr_lo_freq_hz = (uint64_t)llround(lowest_channel_freq_hz - lowest_ctx_freq_offset);
}

int receiver_bredr_setup_channel_ctx(receiver_session_t *session)
{
    float lowest_ctx_freq_offset =
        -(session->bredr_config.channel_count / 2.0f - 0.5f) * (float)RECEIVER_BREDR_CHANNEL_BW;

    for (unsigned int i = 0; i < session->bredr_config.channel_count; i++)
    {
        receiver_bredr_channel_ctx_t *ctx = &session->bredr_ctx[i];
        memset(ctx, 0, sizeof(*ctx));
        ctx->session = session;
        ctx->bredr_channel = session->bredr_config.bottom_channel + i;

        float channel_offset_freq =
            (float)i * (float)RECEIVER_BREDR_CHANNEL_BW + lowest_ctx_freq_offset;
        float normalized_freq =
            2.0f * (float)M_PI * channel_offset_freq / (float)session->bredr_sample_rate;

        ctx->nco = nco_crcf_create(LIQUID_NCO);
        ctx->firdec = firdecim_crcf_create_kaiser(session->bredr_decim_factor, 7, 60.0f);
        ctx->demod = cpfskdem_create(1, 0.3f, RECEIVER_BREDR_SYMBOL_STEP, 3, 0.5f,
                                     LIQUID_CPFSK_GMSK);
        if (!ctx->nco || !ctx->firdec || !ctx->demod)
            return -1;

        nco_crcf_set_frequency(ctx->nco, normalized_freq);
        bredr_processor_init(&ctx->proc, BREDR_AC_ERRORS_DEFAULT);
        if (pthread_mutex_init(&ctx->queue_mutex, NULL) != 0)
            return -1;
        if (pthread_cond_init(&ctx->queue_cv, NULL) != 0)
        {
            pthread_mutex_destroy(&ctx->queue_mutex);
            return -1;
        }
    }

    return 0;
}

void receiver_bredr_destroy_channel_ctx(receiver_session_t *session)
{
    for (unsigned int i = 0; i < session->bredr_config.channel_count; i++)
    {
        receiver_bredr_channel_ctx_t *ctx = &session->bredr_ctx[i];
        if (ctx->demod)
        {
            cpfskdem_destroy(ctx->demod);
            ctx->demod = NULL;
        }
        if (ctx->firdec)
        {
            firdecim_crcf_destroy(ctx->firdec);
            ctx->firdec = NULL;
        }
        if (ctx->nco)
        {
            nco_crcf_destroy(ctx->nco);
            ctx->nco = NULL;
        }
        pthread_cond_destroy(&ctx->queue_cv);
        pthread_mutex_destroy(&ctx->queue_mutex);
    }
}

void receiver_bredr_process_channel(receiver_bredr_channel_ctx_t *ctx,
                                    const receiver_bredr_block_t *blk)
{
    receiver_session_t *session = ctx->session;
    nco_crcf_mix_block_down(ctx->nco, blk->samples, ctx->mixed, blk->num_samples);

    unsigned int decimated_samples = blk->num_samples / session->bredr_decim_factor;
    firdecim_crcf_execute_block(ctx->firdec, ctx->mixed, decimated_samples, ctx->decimated);

    unsigned long long local_bits = 0ULL;

    for (unsigned int i = 0; i + RECEIVER_BREDR_SYMBOL_STEP <= decimated_samples;
         i += RECEIVER_BREDR_SYMBOL_STEP)
    {
        unsigned int raw_sym = cpfskdem_demodulate(ctx->demod, &ctx->decimated[i]);
        uint8_t bit = (uint8_t)(raw_sym & 1u);
        bredr_status_t s = bredr_push_bit(&ctx->proc, bit);
        local_bits++;
        if (s != BREDR_VALID_PACKET)
            continue;

        bredr_packet_t pkt;
        if (bredr_get_packet(&ctx->proc, &pkt) != 0)
            continue;
        if (session->bredr_config.lap_filter_enabled &&
            ((pkt.lap & 0xFFFFFFu) != session->bredr_config.lap_filter))
            continue;

        unsigned long long bit_in_block = (unsigned long long)(i / RECEIVER_BREDR_SYMBOL_STEP);
        unsigned long long bits_back = pkt.has_header
            ? (58ULL + (unsigned long long)pkt.payload_bytes * 8ULL)
            : 0ULL;
        unsigned long long ac_bit_in_block = (bit_in_block >= bits_back)
            ? (bit_in_block - bits_back)
            : 0ULL;
        unsigned long long abs_raw =
            blk->block_base_sample + ac_bit_in_block * session->bredr_raw_samps_per_bit;
        unsigned int rssi_start = (unsigned int)(ac_bit_in_block * RECEIVER_BREDR_SYMBOL_STEP);
        unsigned int rssi_end = rssi_start + BREDR_AC_SAMPLES;
        if (rssi_end > decimated_samples)
            rssi_end = decimated_samples;
        uint32_t clkn = receiver_bredr_sample_to_clkn(session, abs_raw);
        pkt.rx_clk_ref = abs_raw;
        pkt.rx_clk_1600 = receiver_bredr_sample_to_rx_clk_1600(session, abs_raw);
        pkt.rssi = receiver_rssi_from_mean_power_range(ctx->decimated, rssi_start, rssi_end, 0.0f);
        rx_metadata_t meta = receiver_make_metadata(abs_raw,
                                                    (uint32_t)(RECEIVER_BREDR_CHANNEL_0_FREQ +
                                                               (double)ctx->bredr_channel *
                                                                   RECEIVER_BREDR_CHANNEL_BW),
                                                    (uint16_t)ctx->bredr_channel,
                                                    pkt.rssi,
                                                    (uint8_t)(255u - pkt.ac_errors));
        decoded_packet_t decoded = {
            .protocol = PROTO_BREDR,
            .meta = meta,
        };
        decoded.u.bredr = pkt;

        pthread_mutex_lock(&session->bredr_packet_mutex);
        bredr_piconet_t *pnet =
            bredr_piconet_store_add_packet(&session->bredr_store, &decoded, clkn);
        receiver_bredr_piconet_snapshot_t snapshot;
        receiver_bredr_piconet_snapshot_t *snapshot_ptr = NULL;
        if (pnet)
        {
            receiver_fill_bredr_piconet_snapshot(pnet, &snapshot);
            snapshot_ptr = &snapshot;
        }
        session->bredr_total_packets++;
        if (pkt.has_header)
            session->bredr_header_packets++;
        else
            session->bredr_id_packets++;
        if (session->bredr_callbacks.on_packet)
            session->bredr_callbacks.on_packet(&decoded, snapshot_ptr,
                                               session->bredr_callbacks.user);
        pthread_mutex_unlock(&session->bredr_packet_mutex);
    }

    __atomic_add_fetch(&session->bredr_total_bits, local_bits, __ATOMIC_RELAXED);
}

int receiver_bredr_rx_cb(hackrf_transfer *transfer)
{
    receiver_session_t *session = (receiver_session_t *)transfer->rx_ctx;
    if (!session || session->stop_requested)
        return -1;

    int8_t *samples = (int8_t *)transfer->buffer;
    unsigned int num_samples = (unsigned int)(transfer->valid_length / 2u);
    if (num_samples > RECEIVER_BREDR_BUFFER_SIZE)
        num_samples = RECEIVER_BREDR_BUFFER_SIZE;

    unsigned long long block_base = session->bredr_samples_received;
    session->bredr_samples_received += num_samples;

    int block_idx = -1;
    for (unsigned int i = 0; i < RECEIVER_BREDR_BLOCK_POOL_SIZE; i++)
    {
        unsigned int idx = (session->bredr_pool_write_idx + i) % RECEIVER_BREDR_BLOCK_POOL_SIZE;
        if (__atomic_load_n(&session->bredr_block_pool[idx].refcount, __ATOMIC_ACQUIRE) == 0u)
        {
            block_idx = (int)idx;
            break;
        }
    }
    if (block_idx < 0)
    {
        session->bredr_dropped_blocks++;
        if (session->debug)
            fprintf(stderr, "[debug] dropped callback block: block pool exhausted (%u)\n",
                    RECEIVER_BREDR_BLOCK_POOL_SIZE);
        return session->stop_requested ? -1 : 0;
    }

    receiver_bredr_block_t *blk = &session->bredr_block_pool[(unsigned int)block_idx];
    blk->num_samples = num_samples;
    blk->block_base_sample = block_base;
    blk->refcount = 0u;

    for (unsigned int i = 0; i < num_samples; i++)
    {
        float i_sample = samples[2u * i] / 128.0f;
        float q_sample = samples[2u * i + 1u] / 128.0f;
        blk->samples[i] = i_sample + q_sample * _Complex_I;
    }

    session->bredr_pool_write_idx = ((unsigned int)block_idx + 1u) % RECEIVER_BREDR_BLOCK_POOL_SIZE;
    __atomic_thread_fence(__ATOMIC_RELEASE);

    for (unsigned int ch = 0; ch < session->bredr_config.channel_count; ch++)
    {
        receiver_bredr_channel_ctx_t *ctx = &session->bredr_ctx[ch];
        pthread_mutex_lock(&ctx->queue_mutex);
        if (ctx->block_count == RECEIVER_BREDR_CHANNEL_RING_SIZE)
        {
            unsigned int old_idx = ctx->block_idx_ring[ctx->block_read_idx];
            ctx->block_read_idx = (ctx->block_read_idx + 1u) % RECEIVER_BREDR_CHANNEL_RING_SIZE;
            ctx->block_count--;
            ctx->dropped_blocks++;
            if (session->debug)
                fprintf(stderr, "[debug] ch=%02u queue full (%u), dropping oldest (total=%lu)\n",
                        ctx->bredr_channel, RECEIVER_BREDR_CHANNEL_RING_SIZE, ctx->dropped_blocks);
            __atomic_sub_fetch(&session->bredr_block_pool[old_idx].refcount, 1u, __ATOMIC_ACQ_REL);
        }
        ctx->block_idx_ring[ctx->block_write_idx] = (unsigned int)block_idx;
        ctx->block_write_idx = (ctx->block_write_idx + 1u) % RECEIVER_BREDR_CHANNEL_RING_SIZE;
        ctx->block_count++;
        __atomic_add_fetch(&session->bredr_block_pool[(unsigned int)block_idx].refcount,
                           1u, __ATOMIC_ACQ_REL);
        pthread_cond_signal(&ctx->queue_cv);
        pthread_mutex_unlock(&ctx->queue_mutex);
    }

    return session->stop_requested ? -1 : 0;
}
