#ifndef RECEIVER_SESSION_H
#define RECEIVER_SESSION_H

#include <stddef.h>
#include <stdint.h>

#include "ble_phy.h"
#include "packet_models.h"
#include "rx_metadata.h"

#define RECEIVER_BLE_SAMPLE_RATE 2000000u
#define RECEIVER_BREDR_CHANNEL_0_FREQ 2402000000.0
#define RECEIVER_BREDR_BLOCK_POOL_SIZE 64u
#define RECEIVER_BREDR_CHANNEL_RING_SIZE 8u
#define RECEIVER_BREDR_MAX_CHANNELS 20u
#define RECEIVER_BREDR_DEFAULT_RSSI_AVERAGING_WINDOW 16u
#define RECEIVER_HYBRID_LO_FREQ_HZ 2411500000ULL
#define RECEIVER_HYBRID_SAMPLE_RATE 20000000u

typedef struct receiver_session receiver_session_t;

typedef struct
{
    uint8_t ble_channel;
    uint64_t lo_freq_hz;
    int debug;
} receiver_ble_config_t;

typedef struct
{
    unsigned long long total_samples;
    unsigned long packet_count;
    unsigned long truncated_callback_blocks;
} receiver_ble_stats_t;

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
    unsigned long channel_dropped_blocks[RECEIVER_BREDR_MAX_CHANNELS];
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
    unsigned long bredr_channel_dropped_blocks[RECEIVER_BREDR_MAX_CHANNELS];
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

typedef void (*receiver_ble_packet_fn)(const decoded_packet_t *packet,
                                       void *user);

typedef struct
{
    receiver_ble_packet_fn on_packet;
    void *user;
} receiver_ble_callbacks_t;

typedef void (*receiver_bredr_packet_fn)(const decoded_packet_t *packet,
                                         const receiver_bredr_piconet_snapshot_t *piconet,
                                         void *user);

typedef struct
{
    receiver_bredr_packet_fn on_packet;
    void *user;
} receiver_bredr_callbacks_t;

typedef void (*receiver_hybrid_bredr_packet_fn)(const decoded_packet_t *packet,
                                                const receiver_bredr_piconet_snapshot_t *piconet,
                                                void *user);

typedef void (*receiver_hybrid_ble_packet_fn)(const decoded_packet_t *packet,
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

int receiver_session_run_ble(receiver_session_t *session,
                             const receiver_ble_config_t *config,
                             const receiver_ble_callbacks_t *callbacks,
                             receiver_ble_stats_t *stats_out);

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
