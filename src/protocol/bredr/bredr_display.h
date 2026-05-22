#ifndef BREDR_DISPLAY_H
#define BREDR_DISPLAY_H

#include <stddef.h>
#include <stdint.h>

#include "bredr_phy.h"
#include "rx_metadata.h"

typedef struct
{
    uint32_t lap;
    int uap_found;
    uint8_t uap;
    int clk_known;
    uint8_t central_clk_1_6;
    uint32_t last_successful_rx_clk_1600;
    int tracking_state;
    unsigned long total_packets;
    int combined_rssi_seen;
    float combined_rssi;
    int master_rssi_seen;
    float master_rssi;
    int slave_rssi_seen[8];
    float slave_rssi[8];
} bredr_piconet_snapshot_t;

void bredr_print_packet_details(const bredr_frame_t *frame,
                                const bredr_piconet_snapshot_t *pnet,
                                const rx_metadata_t *meta,
                                unsigned int sample_rate_hz);
void bredr_print_packet_summary_line(unsigned long packet_no,
                                     const bredr_frame_t *frame,
                                     const bredr_piconet_snapshot_t *pnet,
                                     const rx_metadata_t *meta);
void bredr_print_piconet_snapshot(const bredr_piconet_snapshot_t *pnet);
void bredr_print_rssi_snapshot(unsigned long packet_no,
                               const bredr_frame_t *frame,
                               const rx_metadata_t *meta,
                               const bredr_piconet_snapshot_t *const *piconets,
                               size_t count,
                               unsigned int master_clock_mhz);

#endif
