#ifndef RECEIVER_SESSION_H
#define RECEIVER_SESSION_H

#include <stddef.h>
#include <stdint.h>

#include "ble_phy.h"
#include "packet_models.h"
#include "rx_metadata.h"

typedef struct receiver_session receiver_session_t;

typedef struct
{
    uint8_t ble_channel;
    uint64_t lo_freq_hz;
    int debug;
} receiver_btle_config_t;

typedef struct
{
    unsigned long long total_samples;
    unsigned long packet_count;
    unsigned long truncated_callback_blocks;
} receiver_btle_stats_t;

typedef struct
{
    unsigned int channel_count;
    unsigned int bottom_channel;
    unsigned int rssi_averaging_window;
    uint32_t lap_filter;
    int lap_filter_enabled;
    int debug;
} receiver_bredr_config_t;

typedef struct
{
    unsigned long long total_bits;
    unsigned long total_packets;
    unsigned long header_packets;
    unsigned long id_packets;
    unsigned long dropped_blocks;
    unsigned long channel_dropped_blocks[20];
    unsigned int channel_count;
} receiver_bredr_stats_t;

typedef struct
{
    int debug;
} receiver_hybrid_config_t;

typedef struct
{
    unsigned long total_packets;
    unsigned long dropped_blocks;
    unsigned long bredr_channel_dropped_blocks[20];
    unsigned long ble_dropped_blocks;
    unsigned int bredr_channel_count;
} receiver_hybrid_stats_t;

typedef struct
{
    uint32_t lap;
    int uap_found;
    uint8_t uap;
    int clk_known;
    uint8_t central_clk_1_6;
    int tracking_state;
    unsigned long total_packets;
    int combined_rssi_seen;
    float combined_rssi;
    int master_rssi_seen;
    float master_rssi;
    int slave_rssi_seen[8];
    float slave_rssi[8];
} receiver_bredr_piconet_snapshot_t;

typedef void (*receiver_btle_packet_fn)(const decoded_packet_t *packet,
                                        void *user);

typedef struct
{
    receiver_btle_packet_fn on_packet;
    void *user;
} receiver_btle_callbacks_t;

typedef void (*receiver_bredr_packet_fn)(const decoded_packet_t *packet,
                                         const receiver_bredr_piconet_snapshot_t *piconet,
                                         void *user);

typedef struct
{
    receiver_bredr_packet_fn on_packet;
    void *user;
} receiver_bredr_callbacks_t;

typedef void (*receiver_hybrid_bredr_packet_fn)(unsigned int ctx_index,
                                                uint32_t lap,
                                                uint32_t clkn,
                                                int ac_errors,
                                                const rx_metadata_t *meta,
                                                void *user);

typedef void (*receiver_hybrid_ble_packet_fn)(const ble_packet_t *pkt,
                                              const rx_metadata_t *meta,
                                              void *user);

typedef struct
{
    receiver_hybrid_bredr_packet_fn on_bredr_packet;
    receiver_hybrid_ble_packet_fn on_ble_packet;
    void *user;
} receiver_hybrid_callbacks_t;

receiver_session_t *receiver_session_create(void);
void receiver_session_destroy(receiver_session_t *session);
void receiver_session_request_stop(receiver_session_t *session);

int receiver_session_run_btle(receiver_session_t *session,
                              const receiver_btle_config_t *config,
                              const receiver_btle_callbacks_t *callbacks,
                              receiver_btle_stats_t *stats_out);

int receiver_session_run_bredr(receiver_session_t *session,
                               const receiver_bredr_config_t *config,
                               const receiver_bredr_callbacks_t *callbacks,
                               receiver_bredr_stats_t *stats_out);

int receiver_session_run_hybrid(receiver_session_t *session,
                                const receiver_hybrid_config_t *config,
                                const receiver_hybrid_callbacks_t *callbacks,
                                receiver_hybrid_stats_t *stats_out);

size_t receiver_session_bredr_piconet_count(const receiver_session_t *session);
int receiver_session_bredr_piconet_snapshot(const receiver_session_t *session,
                                            size_t index,
                                            receiver_bredr_piconet_snapshot_t *out);

#endif
