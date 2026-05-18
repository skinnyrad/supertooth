#include "receiver_session_internal.h"

#include <stddef.h>
#include <stdlib.h>
#include <string.h>

uint32_t receiver_bredr_sample_to_rx_clk_1600(const receiver_session_t *session,
                                              unsigned long long raw_sample_index)
{
    unsigned long long num =
        raw_sample_index * (unsigned long long)RECEIVER_RX_CLK1600_TICKS_PER_SECOND +
        (unsigned long long)(session->bredr_sample_rate / 2u);
    return (uint32_t)(num / (unsigned long long)session->bredr_sample_rate);
}

uint32_t receiver_bredr_sample_to_clkn(const receiver_session_t *session,
                                       unsigned long long raw_sample_index)
{
    unsigned long long num =
        raw_sample_index * (unsigned long long)RECEIVER_CLKN_TICKS_PER_SECOND +
        (unsigned long long)(session->bredr_sample_rate / 2u);
    return (uint32_t)(num / (unsigned long long)session->bredr_sample_rate);
}

rx_metadata_t receiver_make_metadata(uint64_t start_sample,
                                     uint32_t center_frequency_hz,
                                     uint16_t channel_index,
                                     float rssi_dbr,
                                     uint8_t confidence)
{
    rx_metadata_t meta = {
        .source_id = RECEIVER_SOURCE_ID_DEFAULT,
        .start_sample = start_sample,
        .center_frequency_hz = center_frequency_hz,
        .channel_index = channel_index,
        .rssi_dbr = rssi_dbr,
        .confidence = confidence,
    };
    return meta;
}

void receiver_fill_bredr_piconet_snapshot(const bredr_piconet_t *pnet,
                                          receiver_bredr_piconet_snapshot_t *out)
{
    if (!pnet || !out)
        return;

    memset(out, 0, sizeof(*out));
    out->lap = pnet->lap;
    out->uap_found = pnet->uap_found;
    out->uap = pnet->uap;
    out->clk_known = pnet->clk_known;
    out->central_clk_1_6 = pnet->central_clk_1_6;
    out->tracking_state = pnet->tracking_state;
    out->total_packets = pnet->total_packets;
    out->combined_rssi_seen = pnet->combined_rssi_seen;
    out->combined_rssi = pnet->combined_rssi;
    out->master_rssi_seen = pnet->master_rssi_seen;
    out->master_rssi = pnet->master_rssi;
    memcpy(out->slave_rssi_seen, pnet->slave_rssi_seen, sizeof(out->slave_rssi_seen));
    memcpy(out->slave_rssi, pnet->slave_rssi, sizeof(out->slave_rssi));
}

receiver_session_t *receiver_session_create(void)
{
    receiver_session_t *session = (receiver_session_t *)calloc(1, sizeof(*session));
    if (!session)
        return NULL;

    session->pkt_start_abs = -1;
    session->prev_status = BLE_SEARCHING;
    pthread_mutex_init(&session->bredr_packet_mutex, NULL);
    return session;
}

void receiver_session_destroy(receiver_session_t *session)
{
    if (session)
        pthread_mutex_destroy(&session->bredr_packet_mutex);
    free(session);
}

void receiver_session_request_stop(receiver_session_t *session)
{
    if (session)
        session->stop_requested = 1;
}

size_t receiver_session_bredr_piconet_count(const receiver_session_t *session)
{
    return session ? bredr_piconet_store_count(&session->bredr_store) : 0u;
}

int receiver_session_bredr_piconet_snapshot(const receiver_session_t *session,
                                            size_t index,
                                            receiver_bredr_piconet_snapshot_t *out)
{
    if (!session || !out)
        return -1;

    const bredr_piconet_t *pnet = bredr_piconet_store_get(&session->bredr_store, index);
    if (!pnet)
        return -1;

    receiver_fill_bredr_piconet_snapshot(pnet, out);
    return 0;
}
