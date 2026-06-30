#include "ble_channel_processor.h"
#include "rssi_measurements.h"

#include <math.h>
#include <string.h>

static void receiver_ble_emit_event(ble_channel_processor_t *ble,
                                    const ble_event_t *event)
{
    receiver_session_t *session = ble->session;

    if (ble->pipeline == RECEIVER_BLE_PIPELINE_HYBRID)
    {
        receiver_hybrid_callbacks_t callbacks;

        pthread_mutex_lock(&session->decoded_packet_mutex);
        session->hybrid_total_packets++;
        callbacks = session->hybrid_callbacks;
        pthread_mutex_unlock(&session->decoded_packet_mutex);

        if (callbacks.on_ble_packet)
            callbacks.on_ble_packet(event, callbacks.user);
        return;
    }

    if (session->ble_callbacks.on_packet)
        session->ble_callbacks.on_packet(event, session->ble_callbacks.user);
}

int receiver_ble_channel_processor_setup(receiver_session_t *session,
                                        receiver_ble_pipeline_t pipeline)
{
    if (!session || !session->ble_ctx)
        return -1;

    ble_channel_processor_t *ble = session->ble_ctx;
    memset(ble, 0, sizeof(*ble));
    ble->session = session;
    ble->pipeline = pipeline;
    ble->input_decimation = 1u;
    ble->sample_scale = 1u;
    ble->center_frequency_hz = (uint32_t)session->ble_config.lo_freq_hz;
    ble->channel_index = (uint16_t)session->ble_config.ble_channel;

    if (pipeline == RECEIVER_BLE_PIPELINE_HYBRID)
    {
        ble->nco = nco_crcf_create(LIQUID_NCO);
        ble->firdec = firdecim_crcf_create_kaiser(RECEIVER_HYBRID_DECIMATION, 7, 60.0f);
        ble->input_decimation = RECEIVER_HYBRID_DECIMATION;
        ble->sample_scale = RECEIVER_HYBRID_DECIMATION;
        ble->center_frequency_hz = BLE_CH37_FREQ_HZ;
        ble->channel_index = BLE_CH37_INDEX;
        if (!ble->nco || !ble->firdec)
            return -1;
        nco_crcf_set_frequency(ble->nco,
                               2.0f * (float)M_PI * RECEIVER_HYBRID_BLE_FREQ_OFFSET_HZ /
                                   (float)RECEIVER_HYBRID_SAMPLE_RATE);
    }

    ble->demod = cpfskdem_create(1u, 0.5f, RECEIVER_BLE_SAMPLES_PER_SYMBOL, 3u, 0.5f,
                                 LIQUID_CPFSK_GMSK);
    if (!ble->demod)
        return -1;

    ble_bitstream_decoder_init(&ble->proc, ble->channel_index);
    ble->prev_status = BLE_SEARCHING;
    ble->pkt_start_sample = -1;
    if (sample_reader_init(&ble->reader, &session->sample_dispatcher) != 0)
    {
        if (ble->firdec)
        {
            firdecim_crcf_destroy(ble->firdec);
            ble->firdec = NULL;
        }
        if (ble->nco)
        {
            nco_crcf_destroy(ble->nco);
            ble->nco = NULL;
        }
        cpfskdem_destroy(ble->demod);
        ble->demod = NULL;
        return -1;
    }

    return 0;
}

void receiver_ble_channel_processor_destroy(receiver_session_t *session)
{
    if (!session || !session->ble_ctx)
        return;

    ble_channel_processor_t *ble = session->ble_ctx;
    if (ble->demod)
    {
        cpfskdem_destroy(ble->demod);
        ble->demod = NULL;
    }
    if (ble->firdec)
    {
        firdecim_crcf_destroy(ble->firdec);
        ble->firdec = NULL;
    }
    if (ble->nco)
    {
        nco_crcf_destroy(ble->nco);
        ble->nco = NULL;
    }
    sample_reader_destroy(&ble->reader);
    memset(ble, 0, sizeof(*ble));
}

void receiver_ble_channel_processor_process(ble_channel_processor_t *ble,
                                           sample_block_t *blk)
{
    float complex *samples = blk->samples;
    unsigned int sample_count = blk->num_samples;
    unsigned long long buf_start = blk->block_base_sample;

    if (ble->pipeline == RECEIVER_BLE_PIPELINE_HYBRID)
    {
        nco_crcf_mix_block_down(ble->nco, blk->samples, ble->mixed, blk->num_samples);
        sample_count = blk->num_samples / ble->input_decimation;
        firdecim_crcf_execute_block(ble->firdec, ble->mixed, sample_count, ble->decimated);
        samples = ble->decimated;
        buf_start /= ble->input_decimation;
    }

    unsigned int num_bits = sample_count / RECEIVER_BLE_SAMPLES_PER_SYMBOL;
    for (unsigned int s = 0; s < num_bits; s++)
    {
        unsigned int sample_index = s * RECEIVER_BLE_SAMPLES_PER_SYMBOL;
        unsigned int sym = cpfskdem_demodulate(ble->demod, &samples[sample_index]);
        uint8_t bit = (uint8_t)(sym & 0x01u);
        ble_status_t status = ble_bitstream_decoder_push_bit(&ble->proc, bit);

        if (ble->prev_status == BLE_SEARCHING && status == BLE_COLLECTING)
            ble->pkt_start_sample = (long long)(buf_start + sample_index);
        ble->prev_status = status;

        if (status != BLE_VALID_PACKET)
            continue;

        unsigned int i_start = 0u;
        unsigned int i_end = 0u;
        if (ble->pkt_start_sample >= 0)
        {
            long long rel_start = ble->pkt_start_sample - (long long)buf_start;
            i_start = (rel_start < 0) ? 0u : (unsigned int)rel_start;
            i_end = (s + 1u) * RECEIVER_BLE_SAMPLES_PER_SYMBOL;
            if (i_end > sample_count)
                i_end = sample_count;
        }

        ble_frame_t frame;
        if (ble_bitstream_decoder_get_frame(&ble->proc, &frame) != 0)
            continue;

        float rssi_dbr =
            receiver_rssi_from_mean_power_range(samples, i_start, i_end,
                                                RECEIVER_RSSI_INVALID);
        unsigned long long abs_sample_index =
            (buf_start + sample_index) * (unsigned long long)ble->sample_scale;
        rx_metadata_t meta =
            receiver_make_metadata(abs_sample_index,
                                   ble->pipeline == RECEIVER_BLE_PIPELINE_HYBRID
                                       ? RECEIVER_HYBRID_SAMPLE_RATE
                                       : RECEIVER_BLE_SAMPLE_RATE,
                                   ble->center_frequency_hz,
                                   ble->channel_index,
                                   rssi_dbr);
        ble_event_t event = {
            .meta = meta,
            .frame = frame,
        };
        receiver_ble_emit_event(ble, &event);
    }
}
