/**
 * @file bredr_piconet.c
 * @brief BR/EDR piconet tracking implementation.
 *
 * UAP resolution is fully delegated to libbtbb by the calling application.
 * Once the application determines the UAP (and the corresponding CLK1-6 via
 * libbtbb), it calls bredr_piconet_set_uap() to put the piconet into
 * clock-tracking mode.
 *
 * Clock tracking
 * --------------
 * With the UAP known, each subsequent header packet is used to verify the
 * current CLK1-6 estimate: the header is unwhitened with the expected CLK1-6,
 * and the HEC is recomputed with the known UAP.  If the HEC matches, the
 * estimate is updated in-place.  If it does not match, CLK1-6 offsets of
 * ±1 and ±2 are tried; the first match corrects the estimate.  Tracking
 * confidence is managed by tracking_state.
 *
 * Header unwhitening uses bredr_decode_header_bits() from bredr_codec.c/h,
 * which holds the whitening tables (sourced from libbtbb's bluetooth_packet.c).
 */

#include "bredr_piconet.h"
#include "bredr_codec.h"

#include <string.h>
#include <stdlib.h>
#include <math.h>

/* ---------------------------------------------------------------------------
 * RSSI averaging configuration
 * ---------------------------------------------------------------------------*/

static float update_rssi_value(float current, int seen, float sample,
                               unsigned int window, float alpha,
                               float one_minus_alpha)
{
    if (!seen || window == 0u)
        return sample;

    return alpha * sample + one_minus_alpha * current;
}

/* ---------------------------------------------------------------------------
 * Clock tracking
 * ---------------------------------------------------------------------------*/

/**
 * Verify the current CLK1-6 estimate against the incoming frame's header.
 * Tries the expected CLK1-6 (derived from last-success rx_clk_1600 delta),
 * then ±1 and ±2.  Updates central_clk_1_6 and
 * last_successful_rx_clk_1600 on success. Increments tracking_state on
 * success (capped at 5), decrements on failure, and clears clk_known once
 * tracking_state reaches 0.
 */
static int update_clock(bredr_piconet_t *pnet,
                        const bredr_frame_t *frame,
                        uint32_t rx_clk_1600)
{
    uint32_t rx_clk_1600_delta = rx_clk_1600 - pnet->last_successful_rx_clk_1600;
    int expected_clk6 = (int)((pnet->central_clk_1_6 + rx_clk_1600_delta) & 0x3f);

    static const int offsets[] = {0, 1, -1, 2, -2};
    for (int k = 0; k < 5; k++)
    {
        int candidate = ((expected_clk6 + offsets[k]) + 64) % 64;
        if (bredr_hec_ok_for_clk6(frame, pnet->uap, (uint8_t)candidate))
        {
            pnet->central_clk_1_6 = (uint8_t)candidate;
            pnet->last_successful_rx_clk_1600 = rx_clk_1600;
            if (pnet->tracking_state < 5)
                pnet->tracking_state++;
            pnet->clk_known = 1;
            return 1;
        }
    }

    if (pnet->tracking_state > 0)
        pnet->tracking_state--;

    if (pnet->tracking_state == 0)
        pnet->clk_known = 0;

    return 0;
}

/* ---------------------------------------------------------------------------
 * bredr_piconet_t implementation
 * ---------------------------------------------------------------------------*/

void bredr_piconet_init(bredr_piconet_t *pnet, uint32_t lap)
{
    if (!pnet)
        return;

    memset(pnet, 0, sizeof(*pnet));
    pnet->lap = lap & 0xFFFFFFu;
    pnet->tracking_state = -1;

    /* GIAC/LIAC: UAP is the well-known DCI value (0x00). */
    if (pnet->lap == BREDR_LAP_GIAC || pnet->lap == BREDR_LAP_LIAC)
    {
        pnet->uap = BREDR_DCI;
        pnet->uap_found = 1;
    }
}

void bredr_piconet_set_uap(bredr_piconet_t *pnet, uint8_t uap,
                           uint8_t central_clk_1_6,
                           uint32_t last_successful_rx_clk_1600)
{
    if (!pnet)
        return;

    pnet->uap = uap;
    pnet->uap_found = 1;
    pnet->central_clk_1_6 = central_clk_1_6 & 0x3fu;
    pnet->last_successful_rx_clk_1600 = last_successful_rx_clk_1600;
    pnet->clk_known = 1;
    pnet->tracking_state = 1;
}

void bredr_piconet_set_uap_only(bredr_piconet_t *pnet, uint8_t uap)
{
    if (!pnet)
        return;

    pnet->uap = uap;
    pnet->uap_found = 1;
    pnet->clk_known = 0;
    pnet->tracking_state = -1;
}

void bredr_piconet_add_packet(bredr_piconet_t *pnet,
                              const bredr_event_t *event,
                              uint32_t rx_clk_1600,
                              unsigned int rssi_window,
                              float rssi_alpha,
                              float rssi_one_minus_alpha)
{
    if (!pnet || !event)
        return;
    const bredr_frame_t *frame = &event->frame;
    const rx_metadata_t *meta = &event->meta;

    /* Copy into ring buffer. */
    pnet->queue[pnet->queue_head] = *event;
    pnet->queue_head = (pnet->queue_head + 1u) % BREDR_PICONET_QUEUE_SIZE;
    if (pnet->queue_fill < BREDR_PICONET_QUEUE_SIZE)
        pnet->queue_fill++;

    pnet->total_packets++;

    if (pnet->total_packets == 1u)
        pnet->first_seen = rx_clk_1600;
    pnet->last_seen = rx_clk_1600;

    int has_active_track = (pnet->uap_found && pnet->clk_known && pnet->tracking_state > 0);

    /* Before track lock, keep only latest aggregate RSSI. */
    if (!has_active_track && !isnan(meta->rssi_dbr))
    {
        pnet->combined_rssi =
            update_rssi_value(pnet->combined_rssi, pnet->combined_rssi_seen, meta->rssi_dbr,
                              rssi_window, rssi_alpha, rssi_one_minus_alpha);
        pnet->combined_rssi_seen = 1;
    }

    /* Directional RSSI accumulation requires active track + header packet. */
    if (!has_active_track || !frame->has_header)
        return;

    /* Update per-role RSSI only when current frame passes HEC under tracking. */
    int hec_ok = 0;
    if (frame->ac_errors <= 1u)
        hec_ok = update_clock(pnet, frame, rx_clk_1600);
    if (!hec_ok || isnan(meta->rssi_dbr))
        return;

    /* rx_clk_1600 is slot clock (CLKN >> 1), so bit0 == CLK1 parity.
     * Master transmits when CLK1 == 0, slave when CLK1 == 1. */
    if ((rx_clk_1600 & 1u) == 0u)
    {
        pnet->master_rssi =
            update_rssi_value(pnet->master_rssi, pnet->master_rssi_seen, meta->rssi_dbr,
                              rssi_window, rssi_alpha, rssi_one_minus_alpha);
        pnet->master_rssi_seen = 1;
    }
    else
    {
        uint8_t bits[18];
        bredr_decode_header_bits(frame, pnet->central_clk_1_6, bits);
        uint8_t lt = (bits[0]) | (uint8_t)(bits[1] << 1) | (uint8_t)(bits[2] << 2);
        pnet->slave_rssi[lt] =
            update_rssi_value(pnet->slave_rssi[lt], pnet->slave_rssi_seen[lt], meta->rssi_dbr,
                              rssi_window, rssi_alpha, rssi_one_minus_alpha);
        pnet->slave_rssi_seen[lt] = 1;
    }
}
