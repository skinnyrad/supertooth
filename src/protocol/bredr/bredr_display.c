#include "bredr_display.h"

#include <inttypes.h>
#include <stdio.h>

#include "bredr_header_codec.h"

typedef struct
{
    uint8_t lt_addr;
    uint8_t type;
    uint8_t flow;
    uint8_t arqn;
    uint8_t seqn;
    uint8_t hec;
    int hec_ok;
} bredr_display_decoded_header_t;

static const char *const s_bredr_type_names[16] = {
    "NULL", "POLL", "FHS", "DM1",
    "DH1", "HV1", "HV2", "HV3",
    "DV", "AUX1", "DM3", "DH3",
    "EV4", "EV5", "DM5", "DH5"
};

static const char *bredr_tracking_state_desc(int tracking_state)
{
    if (tracking_state < 0)
        return "CLK1-6 never found";
    if (tracking_state == 0)
        return "CLK1-6 reacquire required";
    if (tracking_state >= 5)
        return "strong lock";
    return "tracking";
}

static int bredr_piconet_has_active_track(const bredr_piconet_snapshot_t *pnet)
{
    return pnet && pnet->uap_found && pnet->clk_known && pnet->tracking_state > 0;
}

static void bredr_format_piconet_id(char out[16],
                                    const bredr_packet_t *pkt,
                                    const bredr_piconet_snapshot_t *pnet)
{
    uint32_t lap = pkt ? (pkt->lap & 0xFFFFFFu) : 0u;
    if (pnet)
        lap = pnet->lap & 0xFFFFFFu;

    if (pnet && pnet->uap_found)
        snprintf(out, 16, "0x%02X%06" PRIX32, pnet->uap, lap);
    else
        snprintf(out, 16, "0x??%06" PRIX32, lap);
}

static void bredr_format_rssi_value(char out[8], int seen, float value)
{
    if (seen)
        snprintf(out, 8, "%6.1f", value);
    else
        snprintf(out, 8, "  --.-");
}

static int bredr_decode_header_with_clock(const bredr_packet_t *pkt,
                                          uint8_t uap,
                                          uint8_t clk6,
                                          bredr_display_decoded_header_t *out)
{
    if (!pkt || !pkt->has_header || !out)
        return 0;

    uint8_t bits[18];
    bredr_decode_header_bits(pkt, (uint8_t)(clk6 & 0x3Fu), bits);

    out->lt_addr = (bits[0]) | (uint8_t)(bits[1] << 1) | (uint8_t)(bits[2] << 2);
    out->type = (bits[3]) | (uint8_t)(bits[4] << 1) | (uint8_t)(bits[5] << 2)
                            | (uint8_t)(bits[6] << 3);
    out->flow = bits[7];
    out->arqn = bits[8];
    out->seqn = bits[9];

    out->hec = 0u;
    for (int i = 0; i < 8; i++)
        out->hec |= (uint8_t)(bits[10 + i] << (7 - i));

    uint16_t hdr_data = (uint16_t)((out->lt_addr & 0x7u)
                                 | ((out->type & 0xFu) << 3u)
                                 | ((out->flow & 0x1u) << 7u)
                                 | ((out->arqn & 0x1u) << 8u)
                                 | ((out->seqn & 0x1u) << 9u));
    out->hec_ok = (bredr_compute_hec(hdr_data, uap) == out->hec);
    return out->hec_ok;
}

static void bredr_print_payload_preview(const bredr_packet_t *pkt)
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

void bredr_print_packet_details(const bredr_packet_t *pkt,
                                const bredr_piconet_snapshot_t *pnet)
{
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
        bredr_display_decoded_header_t decoded = {0};
        int decoded_ok = 0;
        if (pnet && pnet->clk_known && pnet->uap_found)
            decoded_ok = bredr_decode_header_with_clock(pkt, pnet->uap,
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

        bredr_print_payload_preview(pkt);
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
               pnet->tracking_state, bredr_tracking_state_desc(pnet->tracking_state));
        if (pnet->clk_known)
            printf("CLK1-6       : %u\n", pnet->central_clk_1_6);
        else
            printf("CLK1-6       : ??\n");
    }
}

void bredr_print_packet_summary_line(unsigned long packet_no,
                                     const bredr_packet_t *pkt,
                                     const bredr_piconet_snapshot_t *pnet,
                                     const rx_metadata_t *meta)
{
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

void bredr_print_piconet_snapshot(const bredr_piconet_snapshot_t *pnet)
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

void bredr_print_rssi_snapshot(unsigned long packet_no,
                               const bredr_packet_t *pkt,
                               const rx_metadata_t *meta,
                               const bredr_piconet_snapshot_t *const *piconets,
                               size_t count,
                               unsigned int master_clock_mhz)
{
    printf("\n================ RSSI Snapshot (Packet #%lu) ================\n", packet_no);
    printf("Sample Index : %" PRIu64 " (%u Msps master clock)\n",
           meta->start_sample, master_clock_mhz);
    printf("Piconets     : %zu\n", count);
    printf("--------------------------------------------------------------\n");

    for (size_t i = 0; i < count; i++)
    {
        const bredr_piconet_snapshot_t *cur = piconets[i];
        char piconet_id[16];
        bredr_format_piconet_id(piconet_id, pkt, cur);

        if (!bredr_piconet_has_active_track(cur))
        {
            char combined_buf[8];
            bredr_format_rssi_value(combined_buf, cur->combined_rssi_seen, cur->combined_rssi);
            printf("Piconet: %-10s | Track: %2d | Combined: %s dBr\n",
                   piconet_id,
                   cur->tracking_state,
                   combined_buf);
            continue;
        }

        char central_buf[8];
        bredr_format_rssi_value(central_buf, cur->master_rssi_seen, cur->master_rssi);
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
            bredr_format_rssi_value(pbuf, 1, cur->slave_rssi[lt]);
            printf(" | Periph[%d]: %s dBr", lt, pbuf);
        }
        if (!periph_seen)
            printf(" | Periph: (none yet)");
        printf("\n");
    }

    printf("==============================================================\n");
}
