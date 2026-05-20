/**
 * @file bredr_recovery_btbb.c
 * @brief Transitional libbtbb-backed BR/EDR recovery backend.
 */

#include "bredr_recovery_btbb.h"

#include <btbb.h>
#include <stdlib.h>

struct bredr_recovery_btbb_state
{
    btbb_piconet *piconet;
};

static btbb_packet *btbb_packet_from_bredr(const bredr_packet_t *pkt,
                                           int channel,
                                           uint32_t clkn)
{
    if (!pkt || !pkt->has_header)
        return NULL;

    char symbols[BREDR_SYMBOLS_MAX] = {0};
    unsigned int payload_bits = pkt->payload_bytes * 8u;
    const unsigned int max_payload = BREDR_SYMBOLS_MAX > 122u
                                         ? BREDR_SYMBOLS_MAX - 122u
                                         : 0u;
    if (payload_bits > max_payload)
        payload_bits = max_payload;

    uint64_t sw = bredr_gen_syncword(pkt->lap & 0xFFFFFFu);
    for (unsigned int i = 0; i < 64u; i++)
        symbols[i] = (char)((sw >> i) & 1u);

    uint8_t sw_last = (uint8_t)((sw >> 63u) & 1u);
    uint8_t trailer = sw_last ? 0xAu : 0x5u;
    for (unsigned int i = 0; i < 4u; i++)
        symbols[64u + i] = (char)((trailer >> i) & 1u);

    for (unsigned int i = 0; i < 54u; i++)
        symbols[68u + i] = (char)((pkt->header_raw >> i) & 1u);

    for (unsigned int i = 0; i < payload_bits; i++)
        symbols[122u + i] = (char)((pkt->payload[i / 8u] >> (i % 8u)) & 1u);

    btbb_packet *bp = btbb_packet_new();
    if (!bp)
        return NULL;

    btbb_packet_set_data(bp, symbols, (int)(122u + payload_bits), (uint8_t)channel, clkn);
    btbb_packet_set_flag(bp, BTBB_WHITENED, 1);
    return bp;
}

void bredr_recovery_btbb_global_init(uint8_t max_ac_errors)
{
    btbb_init(max_ac_errors);
}

bredr_recovery_btbb_state_t *bredr_recovery_btbb_state_create(uint32_t lap)
{
    bredr_recovery_btbb_state_t *state =
        (bredr_recovery_btbb_state_t *)calloc(1, sizeof(*state));
    if (!state)
        return NULL;

    state->piconet = btbb_piconet_new();
    if (state->piconet)
        btbb_init_piconet(state->piconet, lap & 0xFFFFFFu);
    return state;
}

void bredr_recovery_btbb_state_destroy(bredr_recovery_btbb_state_t *state)
{
    if (!state)
        return;
    if (state->piconet)
        btbb_piconet_unref(state->piconet);
    free(state);
}

void bredr_recovery_btbb_state_reset(bredr_recovery_btbb_state_t *state, uint32_t lap)
{
    if (!state)
        return;
    if (state->piconet)
        btbb_piconet_unref(state->piconet);
    state->piconet = btbb_piconet_new();
    if (state->piconet)
        btbb_init_piconet(state->piconet, lap & 0xFFFFFFu);
}

int bredr_recovery_btbb_process_packet(bredr_recovery_btbb_state_t *state,
                                       const bredr_packet_t *pkt,
                                       int channel,
                                       uint32_t clkn,
                                       uint8_t *uap_out,
                                       uint8_t *clk6_hint_out)
{
    if (!state || !state->piconet || !pkt || !pkt->has_header)
        return 0;

    btbb_packet *bp = btbb_packet_from_bredr(pkt, channel, clkn);
    if (!bp)
        return 0;

    btbb_process_packet(bp, state->piconet);

    int recovered = 0;
    if (btbb_piconet_get_flag(state->piconet, BTBB_UAP_VALID))
    {
        if (uap_out)
            *uap_out = btbb_piconet_get_uap(state->piconet);
        if (clk6_hint_out)
            *clk6_hint_out = (uint8_t)(((uint32_t)btbb_piconet_get_clk_offset(state->piconet)) & 0x3Fu);
        recovered = 1;
    }

    btbb_packet_unref(bp);
    return recovered;
}

static bredr_recovery_backend_state_t *btbb_state_create_adapter(uint32_t lap)
{
    return (bredr_recovery_backend_state_t *)bredr_recovery_btbb_state_create(lap);
}

static void btbb_state_destroy_adapter(bredr_recovery_backend_state_t *state)
{
    bredr_recovery_btbb_state_destroy((bredr_recovery_btbb_state_t *)state);
}

static void btbb_state_reset_adapter(bredr_recovery_backend_state_t *state, uint32_t lap)
{
    bredr_recovery_btbb_state_reset((bredr_recovery_btbb_state_t *)state, lap);
}

static int btbb_process_packet_adapter(bredr_recovery_backend_state_t *state,
                                       const bredr_packet_t *pkt,
                                       int channel,
                                       uint32_t clkn,
                                       bredr_recovery_result_t *out)
{
    uint8_t uap = 0u;
    uint8_t clk6_hint = 0u;

    if (!out)
        return 0;

    if (!bredr_recovery_btbb_process_packet((bredr_recovery_btbb_state_t *)state,
                                            pkt, channel, clkn, &uap, &clk6_hint))
        return 0;

    out->uap = uap;
    out->clk6_hint = clk6_hint;
    return 1;
}

const bredr_recovery_backend_ops_t *bredr_recovery_btbb_backend(void)
{
    static const bredr_recovery_backend_ops_t ops = {
        .global_init = bredr_recovery_btbb_global_init,
        .state_create = btbb_state_create_adapter,
        .state_destroy = btbb_state_destroy_adapter,
        .state_reset = btbb_state_reset_adapter,
        .process_packet = btbb_process_packet_adapter,
    };

    return &ops;
}

const bredr_recovery_backend_ops_t *bredr_recovery_default_backend(void)
{
    return bredr_recovery_btbb_backend();
}
