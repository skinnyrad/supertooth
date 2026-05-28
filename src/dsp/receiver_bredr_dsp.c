#include "receiver_dsp.h"
#include "rssi_measurements.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

static float receiver_bredr_rssi_from_history(receiver_bredr_channel_ctx_t *ctx,
                                              uint64_t start_sample,
                                              uint64_t end_sample)
{
    if (!ctx || !ctx->rssi_history || start_sample >= end_sample)
        return RECEIVER_RSSI_INVALID;

    uint64_t history_base = ctx->rssi_history_next_sample - (uint64_t)ctx->rssi_history_valid;
    if (end_sample <= history_base)
        return RECEIVER_RSSI_INVALID;
    if (start_sample < history_base)
        start_sample = history_base;

    uint64_t history_limit = history_base + (uint64_t)ctx->rssi_history_valid;
    if (start_sample >= history_limit)
        return RECEIVER_RSSI_INVALID;
    if (end_sample > history_limit)
        end_sample = history_limit;
    if (start_sample >= end_sample)
        return RECEIVER_RSSI_INVALID;

    float complex *history = NULL;
    if (windowcf_read(ctx->rssi_history, &history) != 0 || !history)
        return RECEIVER_RSSI_INVALID;

    return receiver_rssi_from_mean_power_range(history,
                                               (unsigned int)(start_sample - history_base),
                                               (unsigned int)(end_sample - history_base),
                                               RECEIVER_RSSI_INVALID);
}

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
        ctx->rssi_history_capacity = RECEIVER_BREDR_BUFFER_SIZE / session->bredr_decim_factor;
        if (ctx->rssi_history_capacity < BREDR_AC_DETECT_SAMPLES)
            ctx->rssi_history_capacity = BREDR_AC_DETECT_SAMPLES;
        ctx->rssi_history_capacity += BREDR_AC_DETECT_SAMPLES;
        if (ctx->rssi_history_capacity > RECEIVER_BREDR_BUFFER_SIZE)
            ctx->rssi_history_capacity = RECEIVER_BREDR_BUFFER_SIZE;
        ctx->rssi_history = windowcf_create(ctx->rssi_history_capacity);
        ctx->prev_status = BREDR_SEARCHING;
        ctx->pending_rssi_dbr = RECEIVER_RSSI_INVALID;
        if (!ctx->nco || !ctx->firdec || !ctx->demod || !ctx->rssi_history)
            return -1;

        nco_crcf_set_frequency(ctx->nco, normalized_freq);
        bredr_processor_init(&ctx->proc, BREDR_AC_ERRORS_DEFAULT);
        if (sample_reader_init(&ctx->reader, &session->sample_dispatcher) != 0)
            return -1;
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
        if (ctx->rssi_history)
        {
            windowcf_destroy(ctx->rssi_history);
            ctx->rssi_history = NULL;
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
        sample_reader_destroy(&ctx->reader);
    }
}

void receiver_bredr_process_channel(receiver_bredr_channel_ctx_t *ctx,
                                    sample_block_t *blk)
{
    receiver_session_t *session = ctx->session;
    uint64_t block_start_bit_index = ctx->proc.total_bits_seen;
    uint64_t block_start_decimated_sample = ctx->rssi_history_next_sample;
    nco_crcf_mix_block_down(ctx->nco, blk->samples, ctx->mixed, blk->num_samples);

    unsigned int decimated_samples = blk->num_samples / session->bredr_decim_factor;
    firdecim_crcf_execute_block(ctx->firdec, ctx->mixed, decimated_samples, ctx->decimated);
    if (decimated_samples != 0u)
    {
        windowcf_write(ctx->rssi_history, ctx->decimated, decimated_samples);
        if (ctx->rssi_history_valid + decimated_samples >= ctx->rssi_history_capacity)
            ctx->rssi_history_valid = ctx->rssi_history_capacity;
        else
            ctx->rssi_history_valid += decimated_samples;
        ctx->rssi_history_next_sample += decimated_samples;
    }

    unsigned long long local_bits = 0ULL;

    for (unsigned int i = 0; i + RECEIVER_BREDR_SYMBOL_STEP <= decimated_samples;
         i += RECEIVER_BREDR_SYMBOL_STEP)
    {
        unsigned int raw_sym = cpfskdem_demodulate(ctx->demod, &ctx->decimated[i]);
        uint8_t bit = (uint8_t)(raw_sym & 1u);
        bredr_status_t prev_status = ctx->prev_status;
        bredr_status_t s = bredr_push_bit(&ctx->proc, bit);
        local_bits++;

        if (prev_status == BREDR_SEARCHING && s != BREDR_SEARCHING)
        {
            uint64_t ac_end_sample =
                block_start_decimated_sample + (uint64_t)i + RECEIVER_BREDR_SYMBOL_STEP;
            uint64_t ac_start_sample =
                (ac_end_sample >= BREDR_AC_DETECT_SAMPLES)
                ? (ac_end_sample - (uint64_t)BREDR_AC_DETECT_SAMPLES)
                : 0u;
            ctx->pending_rssi_dbr =
                receiver_bredr_rssi_from_history(ctx, ac_start_sample, ac_end_sample);
            ctx->pending_rssi_valid = !isnan(ctx->pending_rssi_dbr);
        }
        else if (s == BREDR_SEARCHING || s == BREDR_ERROR)
        {
            ctx->pending_rssi_dbr = RECEIVER_RSSI_INVALID;
            ctx->pending_rssi_valid = 0;
        }

        ctx->prev_status = (s == BREDR_VALID_PACKET) ? BREDR_SEARCHING : s;

        if (s != BREDR_VALID_PACKET)
            continue;

        bredr_frame_t frame;
        if (bredr_get_frame(&ctx->proc, &frame) != 0)
            continue;
        if (session->bredr_config.lap_filter_enabled &&
            ((frame.lap & 0xFFFFFFu) != session->bredr_config.lap_filter))
            continue;

        unsigned long long abs_raw = blk->block_base_sample;
        if (frame.start_bit_index >= block_start_bit_index)
        {
            unsigned long long bits_from_block_start =
                frame.start_bit_index - block_start_bit_index;
            abs_raw += bits_from_block_start *
                       (unsigned long long)session->bredr_raw_samps_per_bit;
        }
        else
        {
            unsigned long long bits_before_block =
                block_start_bit_index - frame.start_bit_index;
            unsigned long long samples_before_block =
                bits_before_block * (unsigned long long)session->bredr_raw_samps_per_bit;
            abs_raw = (samples_before_block <= blk->block_base_sample)
                ? (blk->block_base_sample - samples_before_block)
                : 0ULL;
        }
        float rssi_dbr = ctx->pending_rssi_valid ? ctx->pending_rssi_dbr : RECEIVER_RSSI_INVALID;
        rx_metadata_t meta = receiver_make_metadata(abs_raw,
                                                    (uint32_t)(RECEIVER_BREDR_CHANNEL_0_FREQ +
                                                               (double)ctx->bredr_channel *
                                                                   RECEIVER_BREDR_CHANNEL_BW),
                                                    (uint16_t)ctx->bredr_channel,
                                                    rssi_dbr,
                                                    (uint8_t)(255u - frame.ac_errors));
        bredr_event_t event = {
            .meta = meta,
            .frame = frame,
        };

        pthread_mutex_lock(&session->decoded_packet_mutex);
        bredr_piconet_t *pnet =
            bredr_piconet_store_add_packet(&session->bredr_store, &event,
                                           session->bredr_sample_rate);
        receiver_bredr_callbacks_t callbacks = session->bredr_callbacks;
        receiver_bredr_piconet_snapshot_t snapshot;
        receiver_bredr_piconet_snapshot_t *snapshot_ptr = NULL;
        if (pnet)
        {
            receiver_fill_bredr_piconet_snapshot(pnet, &snapshot);
            snapshot_ptr = &snapshot;
        }
        session->bredr_total_packets++;
        if (frame.has_header)
            session->bredr_header_packets++;
        else
            session->bredr_id_packets++;
        pthread_mutex_unlock(&session->decoded_packet_mutex);

        if (callbacks.on_packet)
            callbacks.on_packet(&event, snapshot_ptr, callbacks.user);

        ctx->pending_rssi_dbr = RECEIVER_RSSI_INVALID;
        ctx->pending_rssi_valid = 0;
    }

    __atomic_add_fetch(&session->bredr_total_bits, local_bits, __ATOMIC_RELAXED);
}

int receiver_dispatcher_rx_cb(hackrf_transfer *transfer)
{
    receiver_session_t *session = (receiver_session_t *)transfer->rx_ctx;
    if (!session || session->stop_requested)
        return -1;

    int8_t *samples = (int8_t *)transfer->buffer;
    unsigned int num_samples = (unsigned int)(transfer->valid_length / 2u);
    if (num_samples > RECEIVER_BREDR_BUFFER_SIZE)
        num_samples = RECEIVER_BREDR_BUFFER_SIZE;

    unsigned long long block_base = session->sample_dispatcher.samples_received;
    session->sample_dispatcher.samples_received += num_samples;

    sample_block_t *blk = sample_dispatcher_acquire_block(&session->sample_dispatcher);
    if (!blk)
    {
        session->sample_dispatcher.dropped_blocks++;
        if (session->debug)
            fprintf(stderr, "[debug] dropped callback block: block pool exhausted (%u)\n",
                    SAMPLE_DISPATCHER_BLOCK_CAPACITY);
        return session->stop_requested ? -1 : 0;
    }

    blk->num_samples = num_samples;
    blk->block_base_sample = block_base;

    for (unsigned int i = 0; i < num_samples; i++)
        blk->samples[i] = hackrf_iq_to_complex(samples, i);

    __atomic_thread_fence(__ATOMIC_RELEASE);
    sample_dispatcher_push_block(&session->sample_dispatcher, blk);
    sample_block_release(blk);

    return session->stop_requested ? -1 : 0;
}
