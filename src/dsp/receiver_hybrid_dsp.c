#include "receiver_dsp.h"
#include "rssi_measurements.h"

#include <math.h>
#include <string.h>

int receiver_hybrid_setup_ble(receiver_session_t *session)
{
    receiver_hybrid_ble_ctx_t *ble = session->hybrid_ble_ctx;
    memset(ble, 0, sizeof(*ble));
    ble->session = session;
    ble->nco = nco_crcf_create(LIQUID_NCO);
    ble->firdec = firdecim_crcf_create_kaiser(RECEIVER_HYBRID_DECIMATION, 7, 60.0f);
    ble->demod = cpfskdem_create(1, 0.5f, RECEIVER_BLE_SAMPLES_PER_SYMBOL, 3, 0.5f,
                                 LIQUID_CPFSK_GMSK);
    if (!ble->nco || !ble->firdec || !ble->demod)
        return -1;
    nco_crcf_set_frequency(ble->nco,
                           2.0f * (float)M_PI * RECEIVER_HYBRID_BLE_FREQ_OFFSET_HZ /
                               (float)RECEIVER_HYBRID_SAMPLE_RATE);
    ble_processor_init(&ble->ble_proc, BLE_CH37_INDEX);
    ble->prev_status = BLE_SEARCHING;
    ble->pkt_start_decimated = -1;
    if (sample_reader_init(&ble->reader, &session->sample_dispatcher) != 0)
        return -1;
    return 0;
}

void receiver_hybrid_destroy_ble(receiver_session_t *session)
{
    receiver_hybrid_ble_ctx_t *ble = session->hybrid_ble_ctx;
    if (ble->demod) cpfskdem_destroy(ble->demod);
    if (ble->firdec) firdecim_crcf_destroy(ble->firdec);
    if (ble->nco) nco_crcf_destroy(ble->nco);
    sample_reader_destroy(&ble->reader);
    ble->demod = NULL; ble->firdec = NULL; ble->nco = NULL;
}

void receiver_hybrid_process_ble(receiver_hybrid_ble_ctx_t *ble,
                                 sample_block_t *blk)
{
    receiver_session_t *session = ble->session;
    nco_crcf_mix_block_down(ble->nco, blk->samples, ble->mixed, blk->num_samples);
    unsigned int decimated_samples = blk->num_samples / RECEIVER_HYBRID_DECIMATION;
    firdecim_crcf_execute_block(ble->firdec, ble->mixed, decimated_samples, ble->decimated);
    unsigned long long buf_start = blk->block_base_sample / RECEIVER_HYBRID_DECIMATION;
    for (unsigned int i = 0; i + RECEIVER_BLE_SAMPLES_PER_SYMBOL <= decimated_samples;
         i += RECEIVER_BLE_SAMPLES_PER_SYMBOL)
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
            rssi_end = i + RECEIVER_BLE_SAMPLES_PER_SYMBOL;
            if (rssi_end > decimated_samples)
                rssi_end = decimated_samples;
        }
        ble_frame_t frame;
        if (ble_get_frame(&ble->ble_proc, &frame) != 0)
            continue;
        float rssi_dbr =
            receiver_rssi_from_mean_power_range(ble->decimated, rssi_start, rssi_end,
                                                RECEIVER_RSSI_INVALID);
        unsigned long long abs_sample = (buf_start + i) * RECEIVER_HYBRID_DECIMATION;
        rx_metadata_t meta = receiver_make_metadata(abs_sample,
                                                    BLE_CH37_FREQ_HZ,
                                                    BLE_CH37_INDEX,
                                                    rssi_dbr,
                                                    255u);
        ble_event_t event = {
            .meta = meta,
            .frame = frame,
        };

        receiver_hybrid_callbacks_t callbacks;
        pthread_mutex_lock(&session->decoded_packet_mutex);
        session->hybrid_total_packets++;
        callbacks = session->hybrid_callbacks;
        pthread_mutex_unlock(&session->decoded_packet_mutex);

        if (callbacks.on_ble_packet)
            callbacks.on_ble_packet(&event, callbacks.user);
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

    sample_block_t *blk = sample_dispatcher_acquire_block(&session->sample_dispatcher);
    if (!blk)
    {
        session->hybrid_dropped_blocks++;
        return 0;
    }

    blk->num_samples = num_samples;
    blk->block_base_sample = session->bredr_samples_received;
    session->bredr_samples_received += num_samples;

    int8_t *samples = (int8_t *)transfer->buffer;
    for (unsigned int i = 0; i < num_samples; i++)
        blk->samples[i] = hackrf_iq_to_complex(samples, i);

    __atomic_thread_fence(__ATOMIC_RELEASE);
    sample_dispatcher_push_block(&session->sample_dispatcher, blk);
    sample_block_release(blk);
    return 0;
}
