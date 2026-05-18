/**
 * @file bredr_piconet_store.c
 * @brief Dynamic piconet store with repository-owned BR/EDR recovery plumbing.
 */

#include "bredr_piconet_store.h"
#include "bredr_header_codec.h"
#include "bredr_phy.h"
#include "bredr_recovery.h"

#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <math.h>

/* ---------------------------------------------------------------------------
 * Constants
 * ---------------------------------------------------------------------------*/

/** Reset btbb piconet state if a LAP is idle for too long.
 *  CLKN ticks at 312.5 µs, so 16384 ticks ≈ 5.1 s. */
#define BTBB_LAP_IDLE_RESET_CLKN 16384u

/* ---------------------------------------------------------------------------
 * Internal entry struct
 * ---------------------------------------------------------------------------*/

struct bredr_piconet_store_entry
{
    bredr_piconet_t *pnet;
    bredr_recovery_state_t *recovery;
    uint32_t last_clkn;
    int has_last_clkn;
};

/**
 * Returns 1 if clk6 + uap produce a header with matching HEC, 0 otherwise.
 */
static int packet_hec_ok_for_clk6(const bredr_packet_t *pkt,
                                  uint8_t uap,
                                  uint8_t clk6)
{
    if (!pkt || !pkt->has_header)
        return 0;

    uint8_t bits[18];
    bredr_decode_header_bits(pkt, (uint8_t)(clk6 & 0x3fu), bits);

    uint16_t hdr_data = 0;
    for (int i = 0; i < 10; i++)
        hdr_data |= (uint16_t)(bits[i] << i);

    uint8_t received_hec = 0;
    for (int i = 0; i < 8; i++)
        received_hec |= (uint8_t)(bits[10 + i] << (7 - i));

    return bredr_compute_hec(hdr_data, uap) == received_hec;
}

/* ---------------------------------------------------------------------------
 * Store helpers
 * ---------------------------------------------------------------------------*/

static bredr_piconet_store_entry_t *find_entry(bredr_piconet_store_t *store,
                                               uint32_t lap)
{
    for (size_t i = 0; i < store->count; i++)
    {
        if ((store->entries[i].pnet->lap & 0xFFFFFFu) == lap)
            return &store->entries[i];
    }
    return NULL;
}

static bredr_piconet_store_entry_t *create_entry(bredr_piconet_store_t *store,
                                                 uint32_t lap)
{
    if (store->count >= store->capacity)
    {
        size_t new_cap = store->capacity * 2u;
        bredr_piconet_store_entry_t *resized =
            (bredr_piconet_store_entry_t *)realloc(
                store->entries, new_cap * sizeof(*resized));
        if (!resized)
            return NULL;
        store->entries = resized;
        store->capacity = new_cap;
    }

    bredr_piconet_t *pnet = (bredr_piconet_t *)malloc(sizeof(bredr_piconet_t));
    if (!pnet)
        return NULL;

    bredr_recovery_state_t *recovery = bredr_recovery_state_create(lap);

    bredr_piconet_init(pnet, lap);

    bredr_piconet_store_entry_t *entry = &store->entries[store->count++];
    entry->pnet = pnet;
    entry->recovery = recovery;
    entry->last_clkn = 0u;
    entry->has_last_clkn = 0;
    return entry;
}

/* ---------------------------------------------------------------------------
 * UAP/clock acquisition
 * ---------------------------------------------------------------------------*/

/** Maximum age (rx_clk_1600 ticks) for historical packets used in CLK1-6
 *  narrowing.  625 µs × 8000 ≈ 5 seconds. */
#define CLK1_6_HISTORY_CUTOFF_CLK1600 8000u

/**
 * Narrows a list of CLK1-6 candidates using the piconet's historical packets.
 *
 * Iterates backwards through the ring buffer (most-recent-first, skipping the
 * current packet which was just added), eliminating any candidate whose
 * time-adjusted CLK1-6 does not produce a valid HEC on that historical packet.
 *
 * Stops early when only one candidate remains, no more usable history exists,
 * or a packet older than CLK1_6_HISTORY_CUTOFF_CLK1600 ticks is reached.
 *
 * @param pnet        Piconet whose queue[] to search.
 * @param cur_pkt     The packet that seeded the candidate list.
 * @param uap         UAP to use for HEC checks.
 * @param candidates  In/out: candidate CLK1-6 values; filtered in-place.
 * @param n           Number of candidates on entry.
 * @return            Number of surviving candidates.
 */
static int narrow_clk6_candidates(const bredr_piconet_t *pnet,
                                  const decoded_packet_t *cur_packet,
                                  uint8_t uap,
                                  int candidates[64],
                                  int n)
{
    const bredr_packet_t *cur_pkt = &cur_packet->u.bredr;
    if (n <= 1 || pnet->queue_fill < 2)
        return n;

    uint32_t cur_clk = cur_pkt->rx_clk_1600;

    /* Iterate backwards through the ring buffer, starting at the packet just
     * before cur_pkt (index queue_head - 2 wrapping around). */
    for (unsigned int i = 1; i < pnet->queue_fill; i++)
    {
        unsigned int idx =
            (pnet->queue_head - 1u - i + 2u * BREDR_PICONET_QUEUE_SIZE) % BREDR_PICONET_QUEUE_SIZE;
        const decoded_packet_t *hist_packet = &pnet->queue[idx];
        const bredr_packet_t *hist = &hist_packet->u.bredr;

        /* Stop if this packet is too old. */
        if ((cur_clk - hist->rx_clk_1600) > CLK1_6_HISTORY_CUTOFF_CLK1600)
            break;

        /* Skip packets without a decodable header inside configured AC tolerance. */
        if (!hist->has_header || hist->ac_errors > BREDR_AC_ERRORS_DEFAULT)
            continue;

        /* CLK1-6 advances one tick per rx_clk_1600 slot.  The CLK1-6 at the
         * historical packet is: (c_current - delta) mod 64. */
        uint32_t delta_mod64 = (cur_clk - hist->rx_clk_1600) & 0x3Fu;

        int j = 0;
        for (int k = 0; k < n; k++)
        {
            uint8_t c_at_hist =
                (uint8_t)((candidates[k] - (int)delta_mod64 + 64) & 0x3F);
            if (packet_hec_ok_for_clk6(hist, uap, c_at_hist))
                candidates[j++] = candidates[k];
        }
        n = j;

        if (n <= 1)
            break;
    }

    return n;
}

static void try_uap_acquisition(bredr_piconet_store_entry_t *entry,
                                const decoded_packet_t *packet,
                                uint32_t clkn)
{
    const bredr_packet_t *pkt = &packet->u.bredr;
    bredr_piconet_t *pnet = entry->pnet;

    if (!pnet || !pkt->has_header || pkt->ac_errors > BREDR_AC_ERRORS_DEFAULT)
        return;
    if (pnet->uap_found && pnet->clk_known)
        return;
    if (!entry->recovery)
        return;

    bredr_recovery_result_t result = {0};
    if (bredr_recovery_process_packet(entry->recovery, pkt, (int)packet->meta.channel_index, clkn, &result))
    {
        uint8_t uap = result.uap;
        uint8_t btbb_clk6 = result.clk6_hint;
        if (!pnet->uap_found)
            bredr_piconet_set_uap_only(pnet, uap);

        /* Collect all CLK1-6 values that produce a valid HEC for this packet. */
        int valid_clk[64];
        int valid_n = 0;
        for (int c = 0; c < 64; c++)
        {
            if (packet_hec_ok_for_clk6(pkt, uap, (uint8_t)c))
                valid_clk[valid_n++] = c;
        }

        /* Narrow the candidates using historical packets in the ring buffer. */
        valid_n = narrow_clk6_candidates(pnet, packet, uap, valid_clk, valid_n);

        if (valid_n == 1)
        {
            /* Unambiguous — use directly. */
            bredr_piconet_set_uap(pnet, uap, (uint8_t)valid_clk[0],
                                  pkt->rx_clk_1600);
        }
        else if (valid_n > 1)
        {
            /* Still ambiguous after history scan — fall back to the candidate
             * closest to btbb's clock-offset hint. */
            int best = valid_clk[0];
            int best_dist = 64;
            for (int i = 0; i < valid_n; i++)
            {
                int d = valid_clk[i] - (int)btbb_clk6;
                if (d < 0)
                    d = -d;
                if (d > 32)
                    d = 64 - d;
                if (d < best_dist)
                {
                    best_dist = d;
                    best = valid_clk[i];
                }
            }
            bredr_piconet_set_uap(pnet, uap, (uint8_t)best, pkt->rx_clk_1600);
        }
        /* valid_n == 0: UAP may be wrong — leave state unchanged and let btbb
         * accumulate more packets before trying again. */
    }
}

/* ---------------------------------------------------------------------------
 * Public API
 * ---------------------------------------------------------------------------*/

void bredr_piconet_store_init(bredr_piconet_store_t *store)
{
    if (!store)
        return;

    memset(store, 0, sizeof(*store));
    store->capacity = BREDR_PICONET_STORE_INIT_CAP;
    store->entries = (bredr_piconet_store_entry_t *)calloc(
        store->capacity, sizeof(*store->entries));

    bredr_recovery_global_init(BREDR_AC_ERRORS_DEFAULT);
}

void bredr_piconet_store_free(bredr_piconet_store_t *store)
{
    if (!store)
        return;

    for (size_t i = 0; i < store->count; i++)
    {
        bredr_recovery_state_destroy(store->entries[i].recovery);
        free(store->entries[i].pnet);
    }

    free(store->entries);
    memset(store, 0, sizeof(*store));
}

bredr_piconet_t *bredr_piconet_store_add_packet(bredr_piconet_store_t *store,
                                                const decoded_packet_t *packet,
                                                uint32_t clkn)
{
    if (!store || !packet || packet->protocol != PROTO_BREDR || !store->entries)
        return NULL;
    const bredr_packet_t *pkt = &packet->u.bredr;

    uint32_t lap = pkt->lap & 0xFFFFFFu;

    bredr_piconet_store_entry_t *entry = find_entry(store, lap);
    if (!entry)
        entry = create_entry(store, lap);
    if (!entry)
        return NULL;

    /* Idle reset: if this LAP has been silent long enough, restart btbb state
     * so UAP recovery begins fresh when it reappears.
     *
     * Guard against occasional non-monotonic timestamp regressions by only
     * applying the idle test when clkn advances. */
    if (entry->has_last_clkn && entry->recovery)
    {
        if (clkn >= entry->last_clkn)
        {
            uint32_t idle = clkn - entry->last_clkn;
            if (idle > BTBB_LAP_IDLE_RESET_CLKN)
                bredr_recovery_state_reset(entry->recovery, lap);
        }
    }
    entry->last_clkn = clkn;
    entry->has_last_clkn = 1;

    bredr_piconet_add_packet(entry->pnet, packet);
    try_uap_acquisition(entry, packet, clkn);

    return entry->pnet;
}

void bredr_piconet_store_print(const bredr_piconet_store_t *store)
{
    if (!store)
        return;

    printf("=== BR/EDR Piconet Store (%zu piconet%s) ===\n",
           store->count, store->count == 1u ? "" : "s");

    for (size_t i = 0; i < store->count; i++)
        bredr_piconet_print(store->entries[i].pnet);
}

size_t bredr_piconet_store_count(const bredr_piconet_store_t *store)
{
    return store ? store->count : 0u;
}

const bredr_piconet_t *bredr_piconet_store_get(const bredr_piconet_store_t *store,
                                               size_t index)
{
    if (!store || index >= store->count || !store->entries)
        return NULL;
    return store->entries[index].pnet;
}
