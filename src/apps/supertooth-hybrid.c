#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>
#include <getopt.h>
#include <inttypes.h>

#include "app_common.h"
#include "ble_display.h"
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
                                  const ble_packet_t *pkt,
                                  const rx_metadata_t *meta)
{
    printf("\n------------------ Packet #%lu --------------------\n", packet_no);
    printf("[RX Info]\n");
    printf("Sample Index : %" PRIu64 " (%u Msps master clock)\n",
           meta->start_sample, RECEIVER_HYBRID_SAMPLE_RATE / 1000000u);
    printf("Type         : BLE\n");
    printf("Frequency    : %u MHz (Channel %u)\n",
           (unsigned int)(meta->center_frequency_hz / 1000000u), meta->channel_index);
    printf("RSSI         : %.2f dBr\n\n", meta->rssi_dbr);
    ble_print_packet(pkt);
    printf("--------------------------------------------------\n");
}

static void print_ble_packet_summary(unsigned long packet_no,
                                     const ble_packet_t *pkt,
                                     const rx_metadata_t *meta)
{
    uint8_t pdu_type = pkt->pdu[0] & 0x0Fu;
    const char *pdu_name = app_ble_pdu_type_name(pdu_type);
    const uint8_t *addr = NULL;
    char addr_buf[18];

    if (app_ble_primary_addr(pkt, &addr))
        app_format_ble_addr(addr_buf, addr);
    else
        snprintf(addr_buf, sizeof(addr_buf), "--");

    printf("pkt=%-6lu type=BLE   pdu=%-14s ch=%02u addr=%s len=%-3u crc=%s rssi=%.1f\n",
           packet_no, pdu_name, meta->channel_index, addr_buf, pkt->pdu[1],
           ble_verify_crc(pkt) ? "PASS" : "FAIL", meta->rssi_dbr);
}

static void print_bredr_packet_full(unsigned long packet_no,
                                    uint32_t lap,
                                    uint32_t clkn,
                                    int ac_errors,
                                    const rx_metadata_t *meta)
{
    printf("\n------------------ Packet #%lu --------------------\n", packet_no);
    printf("[RX Info]\n");
    printf("Sample Index : %" PRIu64 " (%u Msps master clock)\n",
           meta->start_sample, RECEIVER_HYBRID_SAMPLE_RATE / 1000000u);
    printf("Type         : BR/EDR\n");
    printf("Frequency    : %u MHz (Channel %u)\n",
           (unsigned int)(meta->center_frequency_hz / 1000000u), meta->channel_index);
    printf("RSSI         : %.2f dBr\n\n", meta->rssi_dbr);
    printf("[BR/EDR Packet Info]\n");
    printf("LAP          : 0x%06" PRIX32 "\n", lap & 0xFFFFFFu);
    printf("CLKN         : %u\n", clkn);
    printf("AC Errors    : %d\n", ac_errors);
    printf("--------------------------------------------------\n");
}

static void print_bredr_packet_summary(unsigned long packet_no,
                                       uint32_t lap,
                                       uint32_t clkn,
                                       int ac_errors,
                                       const rx_metadata_t *meta)
{
    printf("pkt=%-6lu type=BREDR lap=%06" PRIX32 " ch=%02u ac=%d clkn=%u rssi=%.1f\n",
           packet_no, lap & 0xFFFFFFu, meta->channel_index, ac_errors, clkn, meta->rssi_dbr);
}

static void print_usage(const char *argv0)
{
    fprintf(stderr, "Usage: %s [-v|--view full|summary] [-d|--debug]\n", argv0);
    fprintf(stderr, "  %-30s Packet view style (default: full)\n", "-v, --view");
    fprintf(stderr, "  %-30s Print block-drop diagnostics\n", "-d, --debug");
}

static void handle_hybrid_bredr_packet(uint32_t lap,
                                       uint32_t clkn,
                                       int ac_errors,
                                       const rx_metadata_t *meta,
                                       void *user)
{
    (void)user;
    unsigned long packet_no = ++g_packet_count;
    if (g_output_mode == APP_OUTPUT_MODE_SUMMARY)
        print_bredr_packet_summary(packet_no, lap, clkn, ac_errors, meta);
    else
        print_bredr_packet_full(packet_no, lap, clkn, ac_errors, meta);
    fflush(stdout);
}

static void handle_hybrid_ble_packet(const ble_packet_t *pkt,
                                     const rx_metadata_t *meta,
                                     void *user)
{
    (void)user;
    unsigned long packet_no = ++g_packet_count;
    if (g_output_mode == APP_OUTPUT_MODE_SUMMARY)
        print_ble_packet_summary(packet_no, pkt, meta);
    else
        print_ble_packet_full(packet_no, pkt, meta);
    fflush(stdout);
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

    unsigned long ch_drops_total = 0ul;
    for (unsigned int i = 0; i < stats.bredr_channel_count; i++)
        ch_drops_total += stats.bredr_channel_dropped_blocks[i];

    printf("\n\n=== Session Summary ===\n");
    printf("  Output mode    : %s\n",
           app_output_mode_name(g_output_mode, s_output_modes,
                                sizeof(s_output_modes) / sizeof(s_output_modes[0])));
    printf("  Debug mode     : %s\n", g_debug ? "enabled" : "disabled");
    printf("  Total packets  : %lu\n", stats.total_packets);
    printf("  Dropped blocks : %lu\n", stats.dropped_blocks);

    printf("\n=== Debug Summary ===\n");
    printf("  BR/EDR queue drops (total): %lu\n", ch_drops_total);
    printf("  BR/EDR queue drops (per-channel):\n");
    for (unsigned int i = 0; i < stats.bredr_channel_count; i++)
        printf("    ch=%02u dropped=%lu\n", i, stats.bredr_channel_dropped_blocks[i]);
    printf("  BLE queue drops (total): %lu\n", stats.ble_dropped_blocks);
    printf("  BLE queue drops (per-channel):\n");
    printf("    ch=%02u dropped=%lu\n", BLE_CH37_INDEX, stats.ble_dropped_blocks);

    receiver_session_destroy(g_session);
    g_session = NULL;
    return result == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
}
