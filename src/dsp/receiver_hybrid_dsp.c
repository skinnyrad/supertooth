#include "receiver_dsp.h"
#include "rssi_measurements.h"

#include <math.h>
#include <string.h>

int receiver_hybrid_setup_ble(receiver_session_t *session)
{
    receiver_hybrid_ble_ctx_t *ble = &session->hybrid_ble_ctx;
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
    if (pthread_mutex_init(&ble->queue_mutex, NULL) != 0)
        return -1;
    ble->queue_mutex_initialized = 1;
    if (pthread_cond_init(&ble->queue_cv, NULL) != 0)
    {
        pthread_mutex_destroy(&ble->queue_mutex);
        ble->queue_mutex_initialized = 0;
        return -1;
    }
    ble->queue_cv_initialized = 1;
    return 0;
}

void receiver_hybrid_destroy_ble(receiver_session_t *session)
{
    receiver_hybrid_ble_ctx_t *ble = &session->hybrid_ble_ctx;
    if (ble->demod) cpfskdem_destroy(ble->demod);
    if (ble->firdec) firdecim_crcf_destroy(ble->firdec);
    if (ble->nco) nco_crcf_destroy(ble->nco);
    if (ble->queue_cv_initialized)
    {
        pthread_cond_destroy(&ble->queue_cv);
        ble->queue_cv_initialized = 0;
    }
    if (ble->queue_mutex_initialized)
    {
        pthread_mutex_destroy(&ble->queue_mutex);
        ble->queue_mutex_initialized = 0;
    }
    ble->demod = NULL; ble->firdec = NULL; ble->nco = NULL;
}

void receiver_hybrid_process_ble(receiver_hybrid_ble_ctx_t *ble,
                                 const receiver_bredr_block_t *blk)
{
    receiver_session_t *session = ble->session;
    nco_crcf_mix_block_down(ble->nco, blk->samples, ble->mixed, blk->num_samples);
    unsigned int decimated_samples = blk->num_samples / RECEIVER_HYBRID_DECIMATION;
    firdecim_crcf_execute_block(ble->firdec, ble->mixed, decimated_samples, ble->decimated);
    unsigned long long buf_start = blk->block_base_sample / RECEIVER_HYBRID_DECIMATION;
    for (unsigned int i = 0; i < decimated_samples; i += RECEIVER_BLE_SAMPLES_PER_SYMBOL)
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
        ble_packet_t pkt;
        if (ble_get_packet(&ble->ble_proc, &pkt) != 0)
            continue;
        float rssi_dbr = receiver_rssi_from_mean_power_range(ble->decimated, rssi_start, rssi_end, 0.0f);
        unsigned long long abs_sample = (buf_start + i) * RECEIVER_HYBRID_DECIMATION;
        rx_metadata_t meta = receiver_make_metadata(abs_sample,
                                                    BLE_CH37_FREQ_HZ,
                                                    BLE_CH37_INDEX,
                                                    rssi_dbr,
                                                    255u);
        pthread_mutex_lock(&session->decoded_packet_mutex);
        session->hybrid_total_packets++;
        if (session->hybrid_callbacks.on_ble_packet)
            session->hybrid_callbacks.on_ble_packet(&pkt, &meta,
                                                    session->hybrid_callbacks.user);
        pthread_mutex_unlock(&session->decoded_packet_mutex);
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
        unsigned int idx = (session->bredr_pool_write_idx + i) % RECEIVER_BREDR_BLOCK_POOL_SIZE;
        if (__atomic_load_n(&session->bredr_block_pool[idx].refcount, __ATOMIC_ACQUIRE) == 0u)
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
    receiver_bredr_block_t *blk = &session->bredr_block_pool[(unsigned int)block_idx];
    blk->num_samples = num_samples;
    blk->block_base_sample = session->bredr_samples_received;
    session->bredr_samples_received += num_samples;
    int8_t *samples = (int8_t *)transfer->buffer;
    for (unsigned int i = 0; i < num_samples; i++)
        blk->samples[i] = samples[2u * i] / 128.0f + (samples[2u * i + 1u] / 128.0f) * _Complex_I;
    session->bredr_pool_write_idx = ((unsigned int)block_idx + 1u) % RECEIVER_BREDR_BLOCK_POOL_SIZE;
    __atomic_thread_fence(__ATOMIC_RELEASE);
    receiver_dispatch_block_to_bredr_channels(session, (unsigned int)block_idx);
    receiver_hybrid_ble_ctx_t *ble = &session->hybrid_ble_ctx;
    pthread_mutex_lock(&ble->queue_mutex);
    if (ble->block_count == RECEIVER_BREDR_CHANNEL_RING_SIZE)
    {
        unsigned int old_idx = ble->block_idx_ring[ble->block_read_idx];
        ble->block_read_idx = (ble->block_read_idx + 1u) % RECEIVER_BREDR_CHANNEL_RING_SIZE;
        ble->block_count--;
        ble->dropped_blocks++;
        __atomic_sub_fetch(&session->bredr_block_pool[old_idx].refcount, 1u, __ATOMIC_ACQ_REL);
    }
    ble->block_idx_ring[ble->block_write_idx] = (unsigned int)block_idx;
    ble->block_write_idx = (ble->block_write_idx + 1u) % RECEIVER_BREDR_CHANNEL_RING_SIZE;
    ble->block_count++;
    __atomic_add_fetch(&session->bredr_block_pool[(unsigned int)block_idx].refcount, 1u, __ATOMIC_ACQ_REL);
    pthread_cond_signal(&ble->queue_cv);
    pthread_mutex_unlock(&ble->queue_mutex);
    return 0;
}
