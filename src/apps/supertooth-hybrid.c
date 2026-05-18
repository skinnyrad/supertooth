#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>
#include <getopt.h>
#include <signal.h>
#include <math.h>
#include <inttypes.h>
#include <string.h>

#include "ble_phy.h"
#include "receiver_session.h"

#define BREDR_CHANNEL_0_FREQ 2402e6
#define BLE_CH37_INDEX 37

typedef enum
{
    OUTPUT_MODE_FULL = 0,
    OUTPUT_MODE_SUMMARY = 1
} output_mode_t;

static unsigned long g_packet_count = 0;
static int g_debug = 0;
static volatile sig_atomic_t g_stop = 0;
static output_mode_t g_output_mode = OUTPUT_MODE_FULL;
static receiver_session_t *g_session = NULL;

static void handle_sigint(int sig)
{
    (void)sig;
    g_stop = 1;
    if (g_session)
        receiver_session_request_stop(g_session);
}

static const char *ble_pdu_type_name(uint8_t pdu_type)
{
    switch (pdu_type & 0x0Fu)
    {
    case 0x00u: return "ADV_IND";
    case 0x01u: return "ADV_DIRECT_IND";
    case 0x02u: return "ADV_NONCONN_IND";
    case 0x03u: return "SCAN_REQ";
    case 0x04u: return "SCAN_RSP";
    case 0x05u: return "CONNECT_IND";
    case 0x06u: return "ADV_SCAN_IND";
    default: return "RESERVED";
    }
}

static int ble_primary_addr(const ble_packet_t *pkt, const uint8_t **addr_out)
{
    if (!pkt || !addr_out || pkt->pdu[1] < 6u)
        return 0;

    switch (pkt->pdu[0] & 0x0Fu)
    {
    case 0x00u:
    case 0x01u:
    case 0x02u:
    case 0x04u:
    case 0x06u:
    case 0x03u:
    case 0x05u:
        *addr_out = &pkt->pdu[2];
        return 1;
    default:
        return 0;
    }
}

static void format_ble_addr(char out[18], const uint8_t *addr)
{
    snprintf(out, 18, "%02X:%02X:%02X:%02X:%02X:%02X",
             addr[5], addr[4], addr[3], addr[2], addr[1], addr[0]);
}

static void print_ble_packet_full(unsigned long packet_no,
                                  const ble_packet_t *pkt,
                                  const rx_metadata_t *meta)
{
    printf("\n------------------ Packet #%lu --------------------\n", packet_no);
    printf("[RX Info]\n");
    printf("Sample Index : %" PRIu64 " (%u Msps master clock)\n", meta->start_sample, 20u);
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
    const char *pdu_name = ble_pdu_type_name(pdu_type);
    const uint8_t *addr = NULL;
    char addr_buf[18];

    if (ble_primary_addr(pkt, &addr))
        format_ble_addr(addr_buf, addr);
    else
        snprintf(addr_buf, sizeof(addr_buf), "--");

    printf("pkt=%-6lu type=BLE   pdu=%-14s ch=%02u addr=%s len=%-3u crc=%s rssi=%.1f\n",
           packet_no, pdu_name, meta->channel_index, addr_buf, pkt->pdu[1],
           ble_verify_crc(pkt) ? "PASS" : "FAIL", meta->rssi_dbr);
}

static void print_bredr_packet_full(unsigned long packet_no,
                                    unsigned int ctx_index,
                                    uint32_t lap,
                                    uint32_t clkn,
                                    int ac_errors,
                                    const rx_metadata_t *meta)
{
    printf("\n------------------ Packet #%lu --------------------\n", packet_no);
    printf("[RX Info]\n");
    printf("Sample Index : %" PRIu64 " (%u Msps master clock)\n", meta->start_sample, 20u);
    printf("Type         : BR/EDR\n");
    printf("Frequency    : %u MHz (Channel %u)\n",
           (unsigned int)(meta->center_frequency_hz / 1000000u), meta->channel_index);
    printf("RSSI         : %.2f dBr\n\n", meta->rssi_dbr);
    printf("[BR/EDR Packet Info]\n");
    printf("Context      : %u\n", ctx_index);
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

static int parse_output_mode(const char *arg, output_mode_t *out_mode)
{
    if (!arg || !out_mode)
        return -1;
    if (strcmp(arg, "full") == 0)
    {
        *out_mode = OUTPUT_MODE_FULL;
        return 0;
    }
    if (strcmp(arg, "summary") == 0 || strcmp(arg, "ubertooth") == 0)
    {
        *out_mode = OUTPUT_MODE_SUMMARY;
        return 0;
    }
    return -1;
}

static void print_usage(const char *argv0)
{
    fprintf(stderr, "Usage: %s [-v|--view full|summary] [-d|--debug]\n", argv0);
    fprintf(stderr, "  %-30s Packet view style (default: full)\n", "-v, --view");
    fprintf(stderr, "  %-30s Print block-drop diagnostics\n", "-d, --debug");
}

static void handle_hybrid_bredr_packet(unsigned int ctx_index,
                                       uint32_t lap,
                                       uint32_t clkn,
                                       int ac_errors,
                                       const rx_metadata_t *meta,
                                       void *user)
{
    (void)user;
    unsigned long packet_no = ++g_packet_count;
    if (g_output_mode == OUTPUT_MODE_SUMMARY)
        print_bredr_packet_summary(packet_no, lap, clkn, ac_errors, meta);
    else
        print_bredr_packet_full(packet_no, ctx_index, lap, clkn, ac_errors, meta);
    fflush(stdout);
}

static void handle_hybrid_ble_packet(const ble_packet_t *pkt,
                                     const rx_metadata_t *meta,
                                     void *user)
{
    (void)user;
    unsigned long packet_no = ++g_packet_count;
    if (g_output_mode == OUTPUT_MODE_SUMMARY)
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
            if (parse_output_mode(optarg, &g_output_mode) != 0)
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

    printf("Supertooth Hybrid: BR/EDR ch0-19 + BLE ch37\n");
    printf("LO: %.1f MHz, %d BR/EDR channels + 1 BLE channel, 20 MHz bandwidth\n",
           2411500000.0 / 1e6, 20);
    printf("View mode   : %s\n", g_output_mode == OUTPUT_MODE_SUMMARY ? "summary" : "full");
    printf("Debug       : %s\n", g_debug ? "enabled" : "disabled");
    printf("Block pool  : %u blocks, per-channel queue: %u\n", 64u, 8u);

    signal(SIGINT, handle_sigint);

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
    printf("  Output mode    : %s\n", g_output_mode == OUTPUT_MODE_SUMMARY ? "summary" : "full");
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
