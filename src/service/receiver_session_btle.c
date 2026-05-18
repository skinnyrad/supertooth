#include "receiver_dsp.h"

#include <string.h>
#include <unistd.h>

int receiver_session_run_btle(receiver_session_t *session,
                              const receiver_btle_config_t *config,
                              const receiver_btle_callbacks_t *callbacks,
                              receiver_btle_stats_t *stats_out)
{
    if (!session || !config)
        return -1;

    session->stop_requested = 0;
    session->total_samples = 0;
    session->packet_count = 0;
    session->truncated_callback_blocks = 0;
    session->pkt_start_abs = -1;
    session->prev_status = BLE_SEARCHING;
    session->debug = config->debug;
    session->btle_config = *config;
    if (callbacks)
        session->btle_callbacks = *callbacks;
    else
        memset(&session->btle_callbacks, 0, sizeof(session->btle_callbacks));

    ble_processor_init(&session->ble_proc, config->ble_channel);
    session->demod = cpfskdem_create(1u, 0.5f, RECEIVER_BTLE_SAMPLES_PER_SYMBOL, 3u, 0.5f,
                                     LIQUID_CPFSK_GMSK);
    if (!session->demod)
        return -1;

    hackrf_device *device = NULL;
    int result = hackrf_connect(&device);
    if (result != HACKRF_SUCCESS)
    {
        cpfskdem_destroy(session->demod);
        session->demod = NULL;
        return result;
    }

    hackrf_config_t radio_config = {
        .lo_freq_hz = config->lo_freq_hz,
        .sample_rate = RECEIVER_BTLE_SAMPLE_RATE,
        .lna_gain = RECEIVER_BTLE_LNA_GAIN,
        .vga_gain = RECEIVER_BTLE_VGA_GAIN,
    };

    result = hackrf_configure(device, &radio_config);
    if (result == HACKRF_SUCCESS)
        result = hackrf_start_rx(device, receiver_btle_rx_cb, session);

    if (result == HACKRF_SUCCESS)
    {
        while (!session->stop_requested)
            sleep(1);
        hackrf_stop_rx(device);
    }

    hackrf_disconnect(device);
    cpfskdem_destroy(session->demod);
    session->demod = NULL;

    if (stats_out)
    {
        stats_out->total_samples = session->total_samples;
        stats_out->packet_count = session->packet_count;
        stats_out->truncated_callback_blocks = session->truncated_callback_blocks;
    }

    return result;
}
