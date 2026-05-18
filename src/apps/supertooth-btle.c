#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>
#include <getopt.h>
#include <signal.h>
#include <inttypes.h>
#include <string.h>
#include "ble_phy.h"
#include "receiver_session.h"

// BTLE Advertising Channel 37 constants
#define BTLE_CH37_INDEX 37         // BLE channel index for 2.402 GHz
#define BTLE_CH37_FREQ 2402e6      // Channel 37 frequency (2.402 GHz)
#define BTLE_CH38_INDEX 38         // BLE channel index for 2.426 GHz
#define BTLE_CH38_FREQ 2426e6      // Channel 38 frequency (2.426 GHz)
#define BTLE_CH39_INDEX 39         // BLE channel index for 2.480 GHz
#define BTLE_CH39_FREQ 2480e6      // Channel 39 frequency (2.480 GHz)
#define ADVERTISING_AA 0x8E89BED6U // BTLE advertising access address
#define BTLE_SAMPLE_RATE_HZ 2000000u

static unsigned long g_packet_count = 0;
static int g_debug = 0;
static volatile sig_atomic_t g_stop = 0;
static uint8_t g_ble_channel = BTLE_CH37_INDEX;
static uint64_t g_ble_freq_hz = (uint64_t)BTLE_CH37_FREQ;
static receiver_session_t *g_session = NULL;

typedef enum
{
    OUTPUT_MODE_FULL = 0,
    OUTPUT_MODE_SUMMARY = 1
} output_mode_t;

static output_mode_t g_output_mode = OUTPUT_MODE_FULL;

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
    case 0x00u:
        return "ADV_IND";
    case 0x01u:
        return "ADV_DIRECT_IND";
    case 0x02u:
        return "ADV_NONCONN_IND";
    case 0x03u:
        return "SCAN_REQ";
    case 0x04u:
        return "SCAN_RSP";
    case 0x05u:
        return "CONNECT_IND";
    case 0x06u:
        return "ADV_SCAN_IND";
    default:
        return "RESERVED";
    }
}

static int ble_primary_addr(const ble_packet_t *pkt, const uint8_t **addr_out)
{
    if (!pkt || !addr_out)
        return 0;

    uint8_t pdu_type = pkt->pdu[0] & 0x0Fu;
    uint8_t pay_len = pkt->pdu[1];
    if (pay_len < 6u)
        return 0;

    switch (pdu_type)
    {
    case 0x00u: // ADV_IND
    case 0x01u: // ADV_DIRECT_IND
    case 0x02u: // ADV_NONCONN_IND
    case 0x04u: // SCAN_RSP
    case 0x06u: // ADV_SCAN_IND
    case 0x03u: // SCAN_REQ (ScanA)
    case 0x05u: // CONNECT_IND (InitA)
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

static int parse_ble_channel(const char *arg, uint8_t *out_channel, uint64_t *out_freq_hz)
{
    if (!arg || !out_channel || !out_freq_hz)
        return -1;

    char *endptr = NULL;
    unsigned long ch = strtoul(arg, &endptr, 10);
    if (*arg == '\0' || !endptr || *endptr != '\0')
        return -1;

    switch ((uint8_t)ch)
    {
    case BTLE_CH37_INDEX:
        *out_channel = BTLE_CH37_INDEX;
        *out_freq_hz = (uint64_t)BTLE_CH37_FREQ;
        return 0;
    case BTLE_CH38_INDEX:
        *out_channel = BTLE_CH38_INDEX;
        *out_freq_hz = (uint64_t)BTLE_CH38_FREQ;
        return 0;
    case BTLE_CH39_INDEX:
        *out_channel = BTLE_CH39_INDEX;
        *out_freq_hz = (uint64_t)BTLE_CH39_FREQ;
        return 0;
    default:
        return -1;
    }
}

static void print_usage(const char *argv0)
{
    fprintf(stderr, "Usage: %s [-v|--view full|summary] [-b|--ble-channel 37|38|39] [-d|--debug]\n", argv0);
    fprintf(stderr, "  %-30s Packet view style (default: full)\n", "-v, --view");
    fprintf(stderr, "  %-30s BLE advertising channel (37, 38, or 39; default: 37)\n",
            "-b, --ble-channel 37|38|39");
    fprintf(stderr, "  %-30s Print drop/debug diagnostics\n", "-d, --debug");
}

static void print_ble_packet_full(unsigned long packet_no,
                                  const decoded_packet_t *packet)
{
    const ble_packet_t *pkt = &packet->u.ble;
    const rx_metadata_t *meta = &packet->meta;
    printf("\n------------------ Packet #%lu --------------------\n", packet_no);
    printf("[RX Info]\n");
    printf("Sample Index : %" PRIu64 " (%u Msps master clock)\n",
           meta->start_sample, (unsigned int)(BTLE_SAMPLE_RATE_HZ / 1000000u));
    printf("Type         : BLE\n");
    printf("Frequency    : %u MHz (Channel %u)\n",
           (unsigned int)(meta->center_frequency_hz / 1000000u), meta->channel_index);
    printf("RSSI         : %.2f dBr\n", meta->rssi_dbr);
    printf("\n");
    ble_print_packet(pkt);
    printf("--------------------------------------------------\n");
}

static void print_ble_packet_summary(unsigned long packet_no,
                                     const decoded_packet_t *packet)
{
    const ble_packet_t *pkt = &packet->u.ble;
    const rx_metadata_t *meta = &packet->meta;
    uint8_t pdu_type = pkt->pdu[0] & 0x0Fu;
    const char *pdu_name = ble_pdu_type_name(pdu_type);
    const uint8_t *addr = NULL;
    char addr_buf[18];

    if (ble_primary_addr(pkt, &addr))
        format_ble_addr(addr_buf, addr);
    else
        snprintf(addr_buf, sizeof(addr_buf), "--");

    printf("pkt=%-6lu type=BLE pdu=%-14s ch=%02u addr=%s len=%-3u crc=%s rssi=%.1f\n",
           packet_no,
           pdu_name,
           meta->channel_index,
           addr_buf,
           pkt->pdu[1],
           ble_verify_crc(pkt) ? "PASS" : "FAIL",
           meta->rssi_dbr);
}

static void handle_ble_packet(const decoded_packet_t *packet,
                             void *user)
{
    (void)user;
    unsigned long packet_no = ++g_packet_count;
    if (g_output_mode == OUTPUT_MODE_SUMMARY)
        print_ble_packet_summary(packet_no, packet);
    else
        print_ble_packet_full(packet_no, packet);
    fflush(stdout);
}

// --- Main --------------------------------------------------------------------

int main(int argc, char *argv[])
{
    static const struct option long_opts[] = {
        {"view", required_argument, NULL, 'v'},
        {"ble-channel", required_argument, NULL, 'b'},
        {"debug", no_argument, NULL, 'd'},
        {"help", no_argument, NULL, 'h'},
        {0, 0, 0, 0}
    };
    int opt;
    while ((opt = getopt_long(argc, argv, "v:b:dh", long_opts, NULL)) != -1)
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
        case 'b':
            if (parse_ble_channel(optarg, &g_ble_channel, &g_ble_freq_hz) != 0)
            {
                fprintf(stderr, "Invalid BLE channel: %s (expected 37, 38, or 39)\n", optarg);
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

    printf("BTLE Advertising Packet Detector\n");
    printf("================================\n");
    printf("Channel     : %u (%.3f MHz)\n", g_ble_channel, (double)g_ble_freq_hz / 1e6);
    printf("View mode   : %s\n", g_output_mode == OUTPUT_MODE_SUMMARY ? "summary" : "full");
    printf("Debug       : %s\n", g_debug ? "enabled" : "disabled");
    signal(SIGINT, handle_sigint);

    receiver_btle_config_t config = {
        .ble_channel = g_ble_channel,
        .lo_freq_hz = g_ble_freq_hz,
        .debug = g_debug,
    };
    receiver_btle_callbacks_t callbacks = {
        .on_packet = handle_ble_packet,
        .user = NULL,
    };
    receiver_btle_stats_t stats;
    g_session = receiver_session_create();
    if (!g_session)
    {
        fprintf(stderr, "Failed to create receiver session.\n");
        return EXIT_FAILURE;
    }

    printf("Monitoring BTLE Channel %u (%.3f GHz) for advertising packets...\n",
           g_ble_channel, (double)g_ble_freq_hz / 1e9);
    printf("Press Ctrl+C to exit\n\n");
    int result = receiver_session_run_btle(g_session, &config, &callbacks, &stats);
    receiver_session_destroy(g_session);
    g_session = NULL;
    if (result != 0)
    {
        fprintf(stderr, "BTLE receiver failed.\n");
        return EXIT_FAILURE;
    }

    printf("\n\n=== Session Summary ===\n");
    printf("  Output mode    : %s\n", g_output_mode == OUTPUT_MODE_SUMMARY ? "summary" : "full");
    printf("  Debug mode     : %s\n", g_debug ? "enabled" : "disabled");
    printf("  BLE channel    : %u (%.3f MHz)\n", g_ble_channel, (double)g_ble_freq_hz / 1e6);
    printf("  Total samples  : %" PRIu64 "\n", stats.total_samples);
    printf("  Total packets  : %lu\n", g_packet_count);
    printf("\n=== Debug Summary ===\n");
    printf("  Truncated callback blocks : %lu\n", stats.truncated_callback_blocks);

    return 0;
}
