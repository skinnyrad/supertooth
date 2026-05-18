#include "receiver_dsp.h"
#include "rssi_measurements.h"

#include <stdio.h>

int receiver_btle_rx_cb(hackrf_transfer *transfer)
{
    receiver_session_t *session = (receiver_session_t *)transfer->rx_ctx;
    if (!session)
        return -1;
    if (session->stop_requested)
        return -1;

    int8_t *samples = (int8_t *)transfer->buffer;
    unsigned int num_iq_bytes = (unsigned int)transfer->valid_length;
    unsigned int num_complex = num_iq_bytes / 2u;

    if (num_complex > (RECEIVER_BTLE_BUFFER_SIZE / 2u))
    {
        session->truncated_callback_blocks++;
        if (session->debug)
            fprintf(stderr,
                    "[debug] callback buffer truncated to %u complex samples (total=%lu)\n",
                    RECEIVER_BTLE_BUFFER_SIZE / 2u,
                    session->truncated_callback_blocks);
        num_complex = RECEIVER_BTLE_BUFFER_SIZE / 2u;
    }

    unsigned long long buf_start = session->total_samples;
    session->total_samples += num_complex;

    for (unsigned int i = 0; i < num_complex; i++)
    {
        float i_sample = samples[2u * i] / 128.0f;
        float q_sample = samples[2u * i + 1u] / 128.0f;
        session->raw[i] = i_sample + q_sample * _Complex_I;
    }

    unsigned int num_bits = num_complex / RECEIVER_BTLE_SAMPLES_PER_SYMBOL;
    for (unsigned int s = 0; s < num_bits; s++)
    {
        unsigned int sample_index = s * RECEIVER_BTLE_SAMPLES_PER_SYMBOL;
        unsigned int sym = cpfskdem_demodulate(session->demod, &session->raw[sample_index]);
        uint8_t bit = (uint8_t)(sym & 0x01u);
        ble_status_t status = ble_push_bit(&session->ble_proc, bit);

        if (session->prev_status == BLE_SEARCHING && status == BLE_COLLECTING)
            session->pkt_start_abs = (long long)(buf_start + sample_index);
        session->prev_status = status;

        if (status != BLE_VALID_PACKET)
            continue;

        unsigned int i_start = 0u;
        unsigned int i_end = 0u;
        if (session->pkt_start_abs >= 0)
        {
            long long rel_start = session->pkt_start_abs - (long long)buf_start;
            i_start = (rel_start < 0) ? 0u : (unsigned int)rel_start;
            i_end = (s + 1u) * RECEIVER_BTLE_SAMPLES_PER_SYMBOL;
            if (i_end > num_complex)
                i_end = num_complex;
        }

        ble_packet_t pkt;
        if (ble_get_packet(&session->ble_proc, &pkt) != 0)
            continue;

        session->packet_count++;

        if (session->btle_callbacks.on_packet)
        {
            float rssi_dbr = receiver_rssi_from_mean_power_range(session->raw, i_start, i_end, 0.0f);
            unsigned long long abs_sample_index = buf_start + sample_index;
            rx_metadata_t meta =
                receiver_make_metadata(abs_sample_index,
                                       (uint32_t)session->btle_config.lo_freq_hz,
                                       (uint16_t)session->btle_config.ble_channel,
                                       rssi_dbr,
                                       255u);
            decoded_packet_t decoded = {
                .protocol = PROTO_BLE,
                .meta = meta,
            };
            decoded.u.ble = pkt;
            session->btle_callbacks.on_packet(&decoded, session->btle_callbacks.user);
        }
    }

    return session->stop_requested ? -1 : 0;
}
