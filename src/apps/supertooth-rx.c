#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>
#include <signal.h>
#include <string.h>
#include <math.h>
#include <inttypes.h>
#include <getopt.h>
#include "receiver_session.h"
#include <stddef.h>
#define BREDR_CHANNEL_0_FREQ 2402000000.0
#define BREDR_MAX_CHANNEL 79u
#define MAX_BREDR_CHANNELS 20u
#define DEFAULT_BREDR_CHANNELS MAX_BREDR_CHANNELS

/* -------------------------------------------------------------------------
 * Output modes
 * -------------------------------------------------------------------------*/

typedef enum
{
    OUTPUT_MODE_FULL = 0,
    OUTPUT_MODE_SUMMARY = 1,
    OUTPUT_MODE_RSSI = 2
} output_mode_t;

typedef struct
{
    uint8_t lt_addr;
    uint8_t type;
    uint8_t flow;
    uint8_t arqn;
    uint8_t seqn;
    uint8_t hec;
    int     hec_ok;
} bredr_decoded_header_t;

typedef void (*packet_formatter_fn)(unsigned long packet_no,
                                    const decoded_packet_t *packet,
                                    const receiver_bredr_piconet_snapshot_t *pnet);

typedef struct
{
    output_mode_t mode;
    const char *name;
    packet_formatter_fn fn;
} output_mode_spec_t;

static volatile sig_atomic_t g_stop = 0;
static output_mode_t g_output_mode = OUTPUT_MODE_FULL;
static int g_debug = 0;
static int g_lap_filter_enabled = 0;
static uint32_t g_lap_filter = 0u;
static unsigned int g_rssi_averaging_window = 16u;
static unsigned int g_num_bredr_channels = DEFAULT_BREDR_CHANNELS;
static unsigned int g_bottom_bredr_channel = 0u;
static int g_bottom_channel_explicit = 0;

/* Counters. */
static unsigned long g_total_packets = 0UL;
static receiver_session_t *g_session = NULL;

static const char *const s_bredr_type_names[16] = {
    "NULL", "POLL", "FHS", "DM1",
    "DH1", "HV1", "HV2", "HV3",
    "DV", "AUX1", "DM3", "DH3",
    "EV4", "EV5", "DM5", "DH5"
};

/* -------------------------------------------------------------------------
 * Helpers
 * -------------------------------------------------------------------------*/

static void handle_sigint(int sig)
{
    (void)sig;
    g_stop = 1;
    if (g_session)
        receiver_session_request_stop(g_session);
}

static const char *tracking_state_desc(int tracking_state)
{
    if (tracking_state < 0)
        return "CLK1-6 never found";
    if (tracking_state == 0)
        return "CLK1-6 reacquire required";
    if (tracking_state >= 5)
        return "strong lock";
    return "tracking";
}

static int piconet_has_active_track(const receiver_bredr_piconet_snapshot_t *pnet)
{
    return pnet && pnet->uap_found && pnet->clk_known && pnet->tracking_state > 0;
}

static void format_piconet_id(char out[16],
                              const bredr_packet_t *pkt,
                              const receiver_bredr_piconet_snapshot_t *pnet)
{
    uint32_t lap = pkt ? (pkt->lap & 0xFFFFFFu) : 0u;
    if (pnet)
        lap = pnet->lap & 0xFFFFFFu;

    if (pnet && pnet->uap_found)
        snprintf(out, 16, "0x%02X%06" PRIX32, pnet->uap, lap);
    else
        snprintf(out, 16, "0x??%06" PRIX32, lap);
}

static void format_rssi_value(char out[8], int seen, float value)
{
    if (seen)
        snprintf(out, 8, "%6.1f", value);
    else
        snprintf(out, 8, "  --.-");
}

static int piconet_lap_cmp(const void *a, const void *b)
{
    const receiver_bredr_piconet_snapshot_t *pa = *(const receiver_bredr_piconet_snapshot_t *const *)a;
    const receiver_bredr_piconet_snapshot_t *pb = *(const receiver_bredr_piconet_snapshot_t *const *)b;
    uint32_t la = pa ? (pa->lap & 0xFFFFFFu) : 0u;
    uint32_t lb = pb ? (pb->lap & 0xFFFFFFu) : 0u;
    if (la < lb)
        return -1;
    if (la > lb)
        return 1;
    return 0;
}

static void print_piconet_snapshot(const receiver_bredr_piconet_snapshot_t *pnet)
{
    if (!pnet)
        return;

    printf("  LAP: 0x%06" PRIX32, pnet->lap & 0xFFFFFFu);
    if (pnet->uap_found)
        printf("  UAP: 0x%02X", pnet->uap);
    else
        printf("  UAP: ??");

    if (pnet->clk_known)
        printf("  CLK1-6: %02u [state=%d]", pnet->central_clk_1_6, pnet->tracking_state);
    else
        printf("  CLK1-6: ?? [state=%d]", pnet->tracking_state);

    printf("  Packets: %lu\n", pnet->total_packets);
}

static void print_session_piconets(void)
{
    size_t count = receiver_session_bredr_piconet_count(g_session);
    printf("=== BR/EDR Piconet Store (%zu piconet%s) ===\n",
           count, count == 1u ? "" : "s");
    for (size_t i = 0; i < count; i++)
    {
        receiver_bredr_piconet_snapshot_t snapshot;
        if (receiver_session_bredr_piconet_snapshot(g_session, i, &snapshot) == 0)
            print_piconet_snapshot(&snapshot);
    }
}

static unsigned int current_master_clock_mhz(void)
{
    return g_num_bredr_channels == 2u ? 4u : g_num_bredr_channels;
}

static int decode_header_with_clock(const bredr_packet_t *pkt,
                                    uint8_t uap,
                                    uint8_t clk6,
                                    bredr_decoded_header_t *out)
{
    if (!pkt || !pkt->has_header || !out)
        return 0;

    uint8_t bits[18];
    bredr_decode_header_bits(pkt, (uint8_t)(clk6 & 0x3Fu), bits);

    out->lt_addr = (bits[0]) | (uint8_t)(bits[1] << 1) | (uint8_t)(bits[2] << 2);
    out->type    = (bits[3]) | (uint8_t)(bits[4] << 1) | (uint8_t)(bits[5] << 2)
                              | (uint8_t)(bits[6] << 3);
    out->flow    = bits[7];
    out->arqn    = bits[8];
    out->seqn    = bits[9];

    out->hec = 0u;
    for (int i = 0; i < 8; i++)
        out->hec |= (uint8_t)(bits[10 + i] << (7 - i));

    uint16_t hdr_data = (uint16_t)((out->lt_addr & 0x7u)
                                 | ((out->type   & 0xFu) << 3u)
                                 | ((out->flow   & 0x1u) << 7u)
                                 | ((out->arqn   & 0x1u) << 8u)
                                 | ((out->seqn   & 0x1u) << 9u));
    out->hec_ok = (bredr_compute_hec(hdr_data, uap) == out->hec);
    return out->hec_ok;
}

static void print_payload_preview(const bredr_packet_t *pkt)
{
    if (!pkt || pkt->payload_bytes == 0u)
    {
        printf("Payload      : (none)\n");
        return;
    }

    unsigned int show = pkt->payload_bytes < 32u ? pkt->payload_bytes : 32u;
    printf("Payload      : %u bytes", pkt->payload_bytes);
    for (unsigned int i = 0; i < show; i++)
    {
        if (i % 16u == 0u)
            printf("\n               ");
        printf("%02X ", pkt->payload[i]);
    }
    if (pkt->payload_bytes > show)
        printf("...");
    printf("\n");
}

static void print_packet_full(unsigned long packet_no,
                              const decoded_packet_t *packet,
                              const receiver_bredr_piconet_snapshot_t *pnet)
{
    const bredr_packet_t *pkt = &packet->u.bredr;
    const rx_metadata_t *meta = &packet->meta;
    if (!pkt)
        return;

    printf("\n------------------ Packet #%lu --------------------\n", packet_no);
    printf("[RX Info]\n");
    printf("Sample Index : %" PRIu64 " (%u Msps master clock)\n",
           meta->start_sample, current_master_clock_mhz());
    printf("Type         : BR/EDR\n");
    printf("Frequency    : %u MHz (Channel %u)\n",
           (unsigned int)(meta->center_frequency_hz / 1000000u), meta->channel_index);
    printf("RSSI         : %.2f dBr\n", meta->rssi_dbr);

    printf("\n[%s Packet Info]\n",
           pkt->has_header ? "BR/EDR Data" : "BR/EDR Inquiry");
    printf("LAP          : 0x%06" PRIX32 "\n", pkt->lap & 0xFFFFFFu);
    if (pkt->has_header)
        printf("HEADER       : 0x%014" PRIX64 "\n",
               pkt->header_raw & 0x003FFFFFFFFFFFFFull);
    else
        printf("HEADER       : (none — shortened access code)\n");

    if (pkt->has_header)
    {
        bredr_decoded_header_t decoded = {0};
        int decoded_ok = 0;
        if (pnet && pnet->clk_known && pnet->uap_found)
            decoded_ok = decode_header_with_clock(pkt, pnet->uap,
                                                  pnet->central_clk_1_6, &decoded);

        if (decoded_ok)
        {
            printf("\n[Decoded Header Info]\n");
            printf("HEC          : 0x%02X [PASS]\n", decoded.hec);
            printf("TYPE         : %s (%u)\n",
                   s_bredr_type_names[decoded.type & 0x0Fu], decoded.type & 0x0Fu);
            printf("LT_ADDR      : %u\n", decoded.lt_addr & 0x07u);
            printf("FLOW         : %u\n", decoded.flow & 1u);
            printf("ARQN         : %u\n", decoded.arqn & 1u);
            printf("SEQN         : %u\n", decoded.seqn & 1u);
        }

        print_payload_preview(pkt);
    }

    if (pkt->has_header && pnet)
    {
        printf("\n[Piconet Info]\n");
        printf("Packets      : %lu\n", pnet->total_packets);
        if (pnet->uap_found)
            printf("UAP          : 0x%02X\n", pnet->uap);
        else
            printf("UAP          : 0x??\n");
        printf("Tracking     : %d (%s)\n",
               pnet->tracking_state, tracking_state_desc(pnet->tracking_state));
        if (pnet->clk_known)
            printf("CLK1-6       : %u\n", pnet->central_clk_1_6);
        else
            printf("CLK1-6       : ??\n");
    }

    printf("--------------------------------------------------\n");
}

static void print_packet_summary(unsigned long packet_no,
                                 const decoded_packet_t *packet,
                                 const receiver_bredr_piconet_snapshot_t *pnet)
{
    const bredr_packet_t *pkt = &packet->u.bredr;
    const rx_metadata_t *meta = &packet->meta;
    if (!pkt)
        return;

    if (pkt->has_header)
    {
        char uap_buf[8];
        char clk_buf[8];
        if (pnet && pnet->uap_found)
            snprintf(uap_buf, sizeof(uap_buf), "%02X", pnet->uap);
        else
            snprintf(uap_buf, sizeof(uap_buf), "??");
        if (pnet && pnet->clk_known)
            snprintf(clk_buf, sizeof(clk_buf), "%02u", pnet->central_clk_1_6);
        else
            snprintf(clk_buf, sizeof(clk_buf), "??");

        printf("pkt=%-6lu lap=%06" PRIX32 " uap=%s ch=%02u ac=%u clk=%s track=%d rssi=%.1f\n",
               packet_no,
               pkt->lap & 0xFFFFFFu,
               uap_buf,
               meta->channel_index,
               pkt->ac_errors,
               clk_buf,
               pnet ? pnet->tracking_state : -1,
               meta->rssi_dbr);
    }
    else
    {
        printf("pkt=%-6lu lap=%06" PRIX32 " uap=?? ch=%02u ac=%u clk=?? track=%d rssi=%.1f\n",
               packet_no,
               pkt->lap & 0xFFFFFFu,
               meta->channel_index,
               pkt->ac_errors,
               pnet ? pnet->tracking_state : -1,
               meta->rssi_dbr);
    }
}

static void print_packet_rssi(unsigned long packet_no,
                              const decoded_packet_t *packet,
                              const receiver_bredr_piconet_snapshot_t *pnet)
{
    (void)pnet;
    const bredr_packet_t *pkt = &packet->u.bredr;
    const rx_metadata_t *meta = &packet->meta;
    if (!pkt)
        return;

    size_t count = receiver_session_bredr_piconet_count(g_session);
    const receiver_bredr_piconet_snapshot_t **ordered =
        (const receiver_bredr_piconet_snapshot_t **)malloc(sizeof(*ordered) * (count > 0u ? count : 1u));
    if (!ordered)
        return;

    size_t used = 0u;
    for (size_t i = 0; i < count; i++)
    {
        receiver_bredr_piconet_snapshot_t *cur =
            (receiver_bredr_piconet_snapshot_t *)malloc(sizeof(*cur));
        if (!cur)
            continue;
        if (receiver_session_bredr_piconet_snapshot(g_session, i, cur) == 0)
            ordered[used++] = cur;
        else
            free(cur);
    }
    qsort(ordered, used, sizeof(*ordered), piconet_lap_cmp);

    printf("\n================ RSSI Snapshot (Packet #%lu) ================\n", packet_no);
    printf("Sample Index : %" PRIu64 " (%u Msps master clock)\n",
           meta->start_sample, current_master_clock_mhz());
    printf("Piconets     : %zu\n", used);
    printf("--------------------------------------------------------------\n");

    for (size_t i = 0; i < used; i++)
    {
        const receiver_bredr_piconet_snapshot_t *cur = ordered[i];
        char piconet_id[16];
        format_piconet_id(piconet_id, pkt, cur);

        if (!piconet_has_active_track(cur))
        {
            char combined_buf[8];
            format_rssi_value(combined_buf, cur->combined_rssi_seen, cur->combined_rssi);
            printf("Piconet: %-10s | Track: %2d | Combined: %s dBr\n",
                   piconet_id,
                   cur->tracking_state,
                   combined_buf);
            continue;
        }

        char central_buf[8];
        format_rssi_value(central_buf, cur->master_rssi_seen, cur->master_rssi);
        printf("Piconet: %-10s | Track: %2d | Central: %s dBr",
               piconet_id,
               cur->tracking_state,
               central_buf);

        int periph_seen = 0;
        for (int lt = 1; lt <= 7; lt++)
        {
            if (!cur->slave_rssi_seen[lt])
                continue;
            periph_seen = 1;
            char pbuf[8];
            format_rssi_value(pbuf, 1, cur->slave_rssi[lt]);
            printf(" | Periph[%d]: %s dBr", lt, pbuf);
        }
        if (!periph_seen)
            printf(" | Periph: (none yet)");
        printf("\n");
    }

    printf("==============================================================\n");
    for (size_t i = 0; i < used; i++)
        free((void *)ordered[i]);
    free(ordered);
}

static const output_mode_spec_t s_output_modes[] = {
    {OUTPUT_MODE_FULL,      "full",      print_packet_full},
    {OUTPUT_MODE_SUMMARY,   "summary",   print_packet_summary},
    {OUTPUT_MODE_RSSI,      "rssi",      print_packet_rssi},
};

static const output_mode_spec_t *output_mode_spec(output_mode_t mode)
{
    for (size_t i = 0; i < sizeof(s_output_modes) / sizeof(s_output_modes[0]); i++)
    {
        if (s_output_modes[i].mode == mode)
            return &s_output_modes[i];
    }
    return &s_output_modes[0];
}

static int parse_output_mode(const char *arg, output_mode_t *out_mode)
{
    if (!arg || !out_mode)
        return -1;
    for (size_t i = 0; i < sizeof(s_output_modes) / sizeof(s_output_modes[0]); i++)
    {
        if (strcmp(arg, s_output_modes[i].name) == 0)
        {
            *out_mode = s_output_modes[i].mode;
            return 0;
        }
    }
    if (strcmp(arg, "ubertooth") == 0)
    {
        *out_mode = OUTPUT_MODE_SUMMARY;
        return 0;
    }
    return -1;
}

static int parse_lap_filter(const char *arg, uint32_t *out_lap)
{
    if (!arg || !out_lap)
        return -1;

    char *end = NULL;
    unsigned long value = strtoul(arg, &end, 0);
    if (end == arg || *end != '\0' || value > 0xFFFFFFul)
        return -1;

    *out_lap = (uint32_t)value;
    return 0;
}

static int parse_rssi_averaging(const char *arg, unsigned int *out_window)
{
    if (!arg || !out_window)
        return -1;
    if (strcmp(arg, "none") == 0)
    {
        *out_window = 0u;
        return 0;
    }

    char *end = NULL;
    unsigned long value = strtoul(arg, &end, 0);
    if (end == arg || *end != '\0' || value > 1000000ul)
        return -1;

    *out_window = (unsigned int)value;
    return 0;
}

static int parse_channel_count(const char *arg, unsigned int *out_channels)
{
    if (!arg || !out_channels)
        return -1;

    char *end = NULL;
    unsigned long value = strtoul(arg, &end, 0);
    if (end == arg || *end != '\0' ||
        value < 2ul || value > (unsigned long)MAX_BREDR_CHANNELS ||
        (value & 1ul) != 0ul)
        return -1;

    *out_channels = (unsigned int)value;
    return 0;
}

static int parse_bottom_channel(const char *arg, unsigned int *out_bottom_channel)
{
    if (!arg || !out_bottom_channel)
        return -1;

    char *end = NULL;
    unsigned long value = strtoul(arg, &end, 0);
    if (end == arg || *end != '\0' || value > (unsigned long)BREDR_MAX_CHANNEL)
        return -1;

    *out_bottom_channel = (unsigned int)value;
    return 0;
}

static void print_usage(const char *argv0)
{
    fprintf(stderr,
            "Usage: %s [-v|--view full|summary|rssi] [-l|--lap LAP] "
            "[--rssi-averaging N|none] [-c|--channels N] [-b|--bottom-channel CH] [-d|--debug]\n", argv0);
    fprintf(stderr, "  %-30s Packet view style (default: full)\n", "-v, --view");
    fprintf(stderr, "  %-30s Only track/report this LAP (e.g. 0x1FC475)\n", "-l, --lap LAP");
    fprintf(stderr, "  %-30s EMA window for piconet RSSI (default: 16; 0/none disables)\n",
            "--rssi-averaging N|none");
    fprintf(stderr, "  %-30s Number of BR/EDR channels from bottom (even 2-%u, default: %u)\n",
            "-c, --channels N",
            MAX_BREDR_CHANNELS, DEFAULT_BREDR_CHANNELS);
    fprintf(stderr, "  %-30s Lowest BR/EDR channel to process (0-%u, default: 0)\n",
            "-b, --bottom-channel CH",
            BREDR_MAX_CHANNEL);
    fprintf(stderr, "  %-30s Print drop/debug diagnostics\n", "-d, --debug");
}

static void handle_bredr_packet(const decoded_packet_t *packet,
                                const receiver_bredr_piconet_snapshot_t *pnet,
                                void *user)
{
    (void)user;
    const output_mode_spec_t *spec = output_mode_spec(g_output_mode);
    g_total_packets++;
    spec->fn(g_total_packets, packet, pnet);
    fflush(stdout);
}

/* -------------------------------------------------------------------------
 * Main
 * -------------------------------------------------------------------------*/

int main(int argc, char *argv[])
{
    static const struct option long_opts[] = {
        {"view",           required_argument, NULL, 'v'},
        {"lap",            required_argument, NULL, 'l'},
        {"rssi-averaging", required_argument, NULL, 'a'},
        {"channels",       required_argument, NULL, 'c'},
        {"bottom-channel", required_argument, NULL, 'b'},
        {"debug",          no_argument,       NULL, 'd'},
        {"help",           no_argument,       NULL, 'h'},
        {NULL,             0,                 NULL,  0 }
    };

    int opt;
    while ((opt = getopt_long(argc, argv, "v:l:a:c:b:dh", long_opts, NULL)) != -1)
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
            case 'l':
                if (parse_lap_filter(optarg, &g_lap_filter) != 0)
                {
                    fprintf(stderr, "Invalid LAP: %s\n", optarg);
                    print_usage(argv[0]);
                    return EXIT_FAILURE;
                }
                g_lap_filter_enabled = 1;
                break;
            case 'a':
                if (parse_rssi_averaging(optarg, &g_rssi_averaging_window) != 0)
                {
                    fprintf(stderr, "Invalid --rssi-averaging value: %s\n", optarg);
                    print_usage(argv[0]);
                    return EXIT_FAILURE;
                }
                break;
            case 'c':
                if (parse_channel_count(optarg, &g_num_bredr_channels) != 0)
                {
                    fprintf(stderr, "Invalid --channels value: %s (expected even 2-%u)\n",
                            optarg, MAX_BREDR_CHANNELS);
                    print_usage(argv[0]);
                    return EXIT_FAILURE;
                }
                break;
            case 'b':
                if (parse_bottom_channel(optarg, &g_bottom_bredr_channel) != 0)
                {
                    fprintf(stderr, "Invalid --bottom-channel value: %s (expected 0-%u)\n",
                            optarg, BREDR_MAX_CHANNEL);
                    print_usage(argv[0]);
                    return EXIT_FAILURE;
                }
                g_bottom_channel_explicit = 1;
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
    if (optind != argc)
    {
        print_usage(argv[0]);
        return EXIT_FAILURE;
    }

    if (g_bottom_channel_explicit)
    {
        unsigned int max_bottom_channel = BREDR_MAX_CHANNEL - (g_num_bredr_channels - 1u);
        if (g_bottom_bredr_channel > max_bottom_channel)
        {
            fprintf(stderr,
                    "Invalid --bottom-channel %u for --channels %u: out of BR/EDR band (0-%u).\n"
                    "For %u channels, the highest bottom channel would be %u.\n",
                    g_bottom_bredr_channel, g_num_bredr_channels, BREDR_MAX_CHANNEL,
                    g_num_bredr_channels, max_bottom_channel);
            return EXIT_FAILURE;
        }
    }

    const output_mode_spec_t *mode_spec = output_mode_spec(g_output_mode);
    unsigned int sample_rate = (g_num_bredr_channels == 2u) ? 4000000u : g_num_bredr_channels * 1000000u;
    unsigned int decim_factor = sample_rate / 2000000u;
    double lo_mhz = ((BREDR_CHANNEL_0_FREQ + g_bottom_bredr_channel * 1000000.0) -
                    (-(g_num_bredr_channels / 2.0 - 0.5) * 1000000.0)) / 1e6;
    printf("Supertooth RX (BR/EDR)\n");
    printf("======================\n");
    printf("LO          : %.3f MHz\n", lo_mhz);
    printf("Sample rate : %u Msps\n", sample_rate / 1000000u);
    printf("Decimation  : /%u -> %u Msps demod input\n",
           decim_factor, 2u);
    printf("Channels    : %u (%u..%u)\n", g_num_bredr_channels,
           g_bottom_bredr_channel, g_bottom_bredr_channel + g_num_bredr_channels - 1u);
    printf("View mode   : %s\n", mode_spec->name);
    if (g_lap_filter_enabled)
        printf("LAP filter  : %06" PRIX32 "\n", g_lap_filter);
    else
        printf("LAP filter  : (none)\n");
    if (g_rssi_averaging_window == 0u)
        printf("RSSI EMA    : disabled\n");
    else
        printf("RSSI EMA    : window=%u\n", g_rssi_averaging_window);
    printf("Block pool  : %u blocks, per-channel queue: %u\n", 64u, 8u);
    printf("Debug       : %s\n", g_debug ? "enabled" : "disabled");
    printf("Press Ctrl+C to stop.\n\n");
    signal(SIGINT, handle_sigint);
    receiver_bredr_config_t config = {
        .channel_count = g_num_bredr_channels,
        .bottom_channel = g_bottom_bredr_channel,
        .rssi_averaging_window = g_rssi_averaging_window,
        .lap_filter = g_lap_filter,
        .lap_filter_enabled = g_lap_filter_enabled,
        .debug = g_debug,
    };
    receiver_bredr_callbacks_t callbacks = {
        .on_packet = handle_bredr_packet,
        .user = NULL,
    };
    receiver_bredr_stats_t stats;
    g_session = receiver_session_create();
    if (!g_session)
        return EXIT_FAILURE;
    int result = receiver_session_run_bredr(g_session, &config, &callbacks, &stats);

    printf("\n\n=== Session Summary ===\n");
    printf("  Output mode    : %s\n", mode_spec->name);
    if (g_lap_filter_enabled)
        printf("  LAP filter     : %06" PRIX32 "\n", g_lap_filter);
    else
        printf("  LAP filter     : (none)\n");
    if (g_rssi_averaging_window == 0u)
        printf("  RSSI EMA       : disabled\n");
    else
    printf("  RSSI EMA       : window=%u\n", g_rssi_averaging_window);
    printf("  Total bits     : %" PRIu64 "\n", stats.total_bits);
    printf("  Header packets : %lu\n", stats.header_packets);
    printf("  ID packets     : %lu\n", stats.id_packets);
    printf("  Dropped blocks : %lu\n", stats.dropped_blocks);
    if (g_debug)
    {
        unsigned long ch_drops_total = 0ul;
        for (unsigned int i = 0; i < stats.channel_count; i++)
            ch_drops_total += stats.channel_dropped_blocks[i];
        printf("  Channel queue drops (total): %lu\n", ch_drops_total);
        for (unsigned int i = 0; i < stats.channel_count; i++)
        {
            if (stats.channel_dropped_blocks[i] > 0ul)
                printf("    ch=%02u dropped=%lu\n",
                       g_bottom_bredr_channel + i,
                       stats.channel_dropped_blocks[i]);
        }
    }
    printf("\n");
    print_session_piconets();
    receiver_session_destroy(g_session);
    g_session = NULL;

    return result == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
}
