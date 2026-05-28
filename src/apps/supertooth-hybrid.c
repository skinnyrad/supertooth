#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>
#include <getopt.h>
#include <inttypes.h>

#include "app_common.h"
#include "ble_display.h"
#include "bredr_display.h"
#include "ble_phy.h"
#include "receiver_session.h"

static unsigned long g_packet_count = 0;
static int g_debug = 0;
static receiver_session_t *g_session = NULL;
static const app_output_mode_option_t s_output_modes[] = {
    {APP_OUTPUT_MODE_FULL, "full"},
    {APP_OUTPUT_MODE_SUMMARY, "summary"},
};

static app_output_mode_t g_output_mode = APP_OUTPUT_MODE_FULL;

static void print_ble_packet_full(unsigned long packet_no,
                                  const ble_event_t *event)
{
    const rx_metadata_t *meta = &event->meta;
    ble_packet_t packet;
    printf("\n------------------ Packet #%lu --------------------\n", packet_no);
    printf("[RX Info]\n");
    printf("Sample Index : %" PRIu64 " (%u Msps master clock)\n",
           meta->start_sample, RECEIVER_HYBRID_SAMPLE_RATE / 1000000u);
    printf("Type         : BLE\n");
    printf("Frequency    : %u MHz (Channel %u)\n",
           (unsigned int)(meta->center_frequency_hz / 1000000u), meta->channel_index);
    printf("RSSI         : %.2f dBr\n\n", meta->rssi_dbr);
    if (ble_decode_frame(&event->frame, meta->channel_index, &packet) == 0)
        ble_print_packet(&packet);
    else
        printf("[BLE Decode Error]\n");
    printf("--------------------------------------------------\n");
}

static void print_ble_packet_summary(unsigned long packet_no,
                                     const ble_event_t *event)
{
    ble_packet_t packet;
    if (ble_decode_frame(&event->frame, event->meta.channel_index, &packet) == 0)
    {
        ble_print_packet_summary_line(packet_no, &packet, &event->meta);
        return;
    }

    printf("pkt=%-6lu type=BLE pdu=%-14s ch=%02u addr=%s len=%-3u crc=%s rssi=%.1f\n",
           packet_no,
           "DECODE_FAIL",
           event->meta.channel_index,
           "--",
           0u,
           "FAIL",
           event->meta.rssi_dbr);
}

static void print_bredr_packet_full(unsigned long packet_no,
                                    const bredr_event_t *event,
                                    const receiver_bredr_piconet_snapshot_t *pnet)
{
    const bredr_frame_t *frame = &event->frame;
    const rx_metadata_t *meta = &event->meta;
    printf("\n------------------ Packet #%lu --------------------\n", packet_no);
    printf("[RX Info]\n");
    printf("Sample Index : %" PRIu64 " (%u Msps master clock)\n",
           meta->start_sample, RECEIVER_HYBRID_SAMPLE_RATE / 1000000u);
    printf("Type         : BR/EDR\n");
    printf("Frequency    : %u MHz (Channel %u)\n",
           (unsigned int)(meta->center_frequency_hz / 1000000u), meta->channel_index);
    printf("RSSI         : %.2f dBr\n\n", meta->rssi_dbr);
    bredr_print_packet_details(frame, pnet, meta, RECEIVER_HYBRID_SAMPLE_RATE);
    printf("--------------------------------------------------\n");
}

static void print_bredr_packet_summary(unsigned long packet_no,
                                       const bredr_event_t *event,
                                       const receiver_bredr_piconet_snapshot_t *pnet)
{
    bredr_print_packet_summary_line(packet_no, &event->frame, pnet, &event->meta);
}

static void print_usage(const char *argv0)
{
    fprintf(stderr, "Usage: %s [-v|--view full|summary] [-d|--debug]\n", argv0);
    fprintf(stderr, "  %-30s Packet view style (default: full)\n", "-v, --view");
    fprintf(stderr, "  %-30s Print block-drop diagnostics\n", "-d, --debug");
}

static void handle_hybrid_bredr_packet(const bredr_event_t *event,
                                       const receiver_bredr_piconet_snapshot_t *pnet,
                                       void *user)
{
    (void)user;
    app_output_lock();
    unsigned long packet_no = ++g_packet_count;
    if (g_output_mode == APP_OUTPUT_MODE_SUMMARY)
        print_bredr_packet_summary(packet_no, event, pnet);
    else
        print_bredr_packet_full(packet_no, event, pnet);
    fflush(stdout);
    app_output_unlock();
}

static void handle_hybrid_ble_packet(const ble_event_t *event,
                                     void *user)
{
    (void)user;
    app_output_lock();
    unsigned long packet_no = ++g_packet_count;
    if (g_output_mode == APP_OUTPUT_MODE_SUMMARY)
        print_ble_packet_summary(packet_no, event);
    else
        print_ble_packet_full(packet_no, event);
    fflush(stdout);
    app_output_unlock();
}

int main(int argc, char *argv[])
{
    static const struct option long_opts[] = {
        {"view", required_argument, NULL, 'v'},
        {"debug", no_argument, NULL, 'd'},
        {"help", no_argument, NULL, 'h'},
        {0, 0, 0, 0}
    };

    int opt;
    while ((opt = getopt_long(argc, argv, "v:dh", long_opts, NULL)) != -1)
    {
        switch (opt)
        {
        case 'v':
            if (app_parse_output_mode(optarg, s_output_modes,
                                      sizeof(s_output_modes) / sizeof(s_output_modes[0]),
                                      &g_output_mode) != 0)
            {
                fprintf(stderr, "Invalid view mode: %s\n", optarg);
                print_usage(argv[0]);
                return EXIT_FAILURE;
            }
            break;
        case 'd':
            g_debug = 1;
            break;
        case 'h':
            print_usage(argv[0]);
            return EXIT_SUCCESS;
        default:
            print_usage(argv[0]);
            return EXIT_FAILURE;
        }
    }

    printf("Supertooth Hybrid: BR/EDR ch0-%u + BLE ch%u\n",
           RECEIVER_BREDR_MAX_CHANNELS - 1u, BLE_CH37_INDEX);
    printf("LO: %.1f MHz, %u BR/EDR channels + 1 BLE channel, %u MHz bandwidth\n",
           (double)RECEIVER_HYBRID_LO_FREQ_HZ / 1e6,
           RECEIVER_BREDR_MAX_CHANNELS,
           RECEIVER_HYBRID_SAMPLE_RATE / 1000000u);
    printf("View mode   : %s\n",
           app_output_mode_name(g_output_mode, s_output_modes,
                                sizeof(s_output_modes) / sizeof(s_output_modes[0])));
    printf("Debug       : %s\n", g_debug ? "enabled" : "disabled");
    printf("Block pool  : %u blocks, per-channel queue: %u\n",
           RECEIVER_BREDR_BLOCK_POOL_SIZE, RECEIVER_BREDR_CHANNEL_RING_SIZE);

    app_install_sigint_handler(&g_session);

    receiver_hybrid_config_t config = {.debug = g_debug};
    receiver_hybrid_callbacks_t callbacks = {
        .on_bredr_packet = handle_hybrid_bredr_packet,
        .on_ble_packet = handle_hybrid_ble_packet,
        .user = NULL,
    };
    receiver_hybrid_stats_t stats;
    g_session = receiver_session_create();
    if (!g_session)
        return EXIT_FAILURE;

    printf("Receiving... Press Ctrl+C to stop.\n");
    int result = receiver_session_run_hybrid(g_session, &config, &callbacks, &stats);

    printf("\n\n=== Session Summary ===\n");
    printf("  Output mode    : %s\n",
           app_output_mode_name(g_output_mode, s_output_modes,
                                sizeof(s_output_modes) / sizeof(s_output_modes[0])));
    printf("  Debug mode     : %s\n", g_debug ? "enabled" : "disabled");
    printf("  Total packets  : %lu\n", stats.total_packets);
    if (g_debug)
    {
        printf("\n=== Debug Summary ===\n");
        printf("  Dropped blocks : %lu\n",
               receiver_session_dispatcher_dropped_blocks(g_session));
    }

    receiver_session_destroy(g_session);
    g_session = NULL;
    return result == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
}
