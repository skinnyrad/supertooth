#ifndef RECEIVER_SESSION_INTERNAL_H
#define RECEIVER_SESSION_INTERNAL_H

#include "receiver_session.h"

#include <complex.h>
#include <pthread.h>

#include <liquid/liquid.h>

#include "hackrf.h"
#include "bredr_piconet.h"
#include "bredr_piconet_store.h"

#define RECEIVER_BTLE_SAMPLE_RATE 2000000u
#define RECEIVER_BTLE_LNA_GAIN 32u
#define RECEIVER_BTLE_VGA_GAIN 32u
#define RECEIVER_BTLE_BUFFER_SIZE 262144u
#define RECEIVER_BTLE_SAMPLES_PER_SYMBOL 2u

#define RECEIVER_BREDR_CHANNEL_BW 1000000.0
#define RECEIVER_BREDR_CHANNEL_0_FREQ 2402000000.0
#define RECEIVER_BREDR_SYMBOL_STEP 2u
#define RECEIVER_BREDR_BUFFER_SIZE 262144u
#define RECEIVER_BREDR_BLOCK_POOL_SIZE 64u
#define RECEIVER_BREDR_CHANNEL_RING_SIZE 8u
#define RECEIVER_BREDR_MAX_CHANNELS 20u
#define RECEIVER_RX_CLK1600_TICKS_PER_SECOND 1600u
#define RECEIVER_CLKN_TICKS_PER_SECOND 3200u
#define RECEIVER_SOURCE_ID_DEFAULT 0u

typedef struct
{
    unsigned int bredr_channel;
    nco_crcf nco;
    firdecim_crcf firdec;
    cpfskdem demod;
    bredr_processor_t proc;
    float complex mixed[RECEIVER_BREDR_BUFFER_SIZE];
    float complex decimated[RECEIVER_BREDR_BUFFER_SIZE];
    unsigned int block_idx_ring[RECEIVER_BREDR_CHANNEL_RING_SIZE];
    unsigned int block_write_idx;
    unsigned int block_read_idx;
    unsigned int block_count;
    unsigned long dropped_blocks;
    pthread_mutex_t queue_mutex;
    pthread_cond_t queue_cv;
    struct receiver_session *session;
} receiver_bredr_channel_ctx_t;

typedef struct
{
    float complex samples[RECEIVER_BREDR_BUFFER_SIZE];
    unsigned int num_samples;
    unsigned long long block_base_sample;
    unsigned int refcount;
} receiver_bredr_block_t;

typedef struct
{
    nco_crcf nco;
    firdecim_crcf firdec;
    cpfskdem demod;
    float complex mixed[RECEIVER_BREDR_BUFFER_SIZE];
    float complex decimated[(unsigned int)(RECEIVER_BREDR_BUFFER_SIZE / 10u) + 1u];
    ble_channel_processor_t ble_proc;
    ble_status_t prev_status;
    long long pkt_start_decimated;
    unsigned int block_idx_ring[RECEIVER_BREDR_CHANNEL_RING_SIZE];
    unsigned int block_write_idx;
    unsigned int block_read_idx;
    unsigned int block_count;
    unsigned long dropped_blocks;
    pthread_mutex_t queue_mutex;
    pthread_cond_t queue_cv;
    struct receiver_session *session;
} receiver_hybrid_ble_ctx_t;

struct receiver_session
{
    volatile int stop_requested;
    int debug;

    cpfskdem demod;
    ble_channel_processor_t ble_proc;
    float complex raw[RECEIVER_BTLE_BUFFER_SIZE / 2u];

    unsigned long long total_samples;
    long long pkt_start_abs;
    ble_status_t prev_status;
    unsigned long packet_count;
    unsigned long truncated_callback_blocks;

    receiver_btle_config_t btle_config;
    receiver_btle_callbacks_t btle_callbacks;

    receiver_bredr_config_t bredr_config;
    receiver_bredr_callbacks_t bredr_callbacks;
    bredr_piconet_store_t bredr_store;
    receiver_bredr_channel_ctx_t bredr_ctx[RECEIVER_BREDR_MAX_CHANNELS];
    receiver_bredr_block_t bredr_block_pool[RECEIVER_BREDR_BLOCK_POOL_SIZE];
    pthread_t *bredr_worker_threads;
    unsigned int bredr_worker_count;
    unsigned int bredr_sample_rate;
    unsigned int bredr_decim_factor;
    unsigned int bredr_raw_samps_per_bit;
    uint64_t bredr_lo_freq_hz;
    unsigned long long bredr_samples_received;
    unsigned int bredr_shutdown_requested;
    unsigned int bredr_pool_write_idx;
    unsigned long bredr_dropped_blocks;
    unsigned long long bredr_total_bits;
    unsigned long bredr_total_packets;
    unsigned long bredr_header_packets;
    unsigned long bredr_id_packets;
    pthread_mutex_t bredr_packet_mutex;

    receiver_hybrid_config_t hybrid_config;
    receiver_hybrid_callbacks_t hybrid_callbacks;
    receiver_hybrid_ble_ctx_t hybrid_ble_ctx;
    pthread_t *hybrid_worker_threads;
    unsigned int hybrid_worker_count;
    unsigned int hybrid_shutdown_requested;
    unsigned long hybrid_total_packets;
    unsigned long hybrid_dropped_blocks;
};

uint32_t receiver_bredr_sample_to_rx_clk_1600(const receiver_session_t *session,
                                              unsigned long long raw_sample_index);
uint32_t receiver_bredr_sample_to_clkn(const receiver_session_t *session,
                                       unsigned long long raw_sample_index);
rx_metadata_t receiver_make_metadata(uint64_t start_sample,
                                     uint32_t center_frequency_hz,
                                     uint16_t channel_index,
                                     float rssi_dbr,
                                     uint8_t confidence);
void receiver_fill_bredr_piconet_snapshot(const bredr_piconet_t *pnet,
                                          receiver_bredr_piconet_snapshot_t *out);

#endif
