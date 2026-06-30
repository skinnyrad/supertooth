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

#define BREDR_BTBB_BASE_PACKET_BITS ((BREDR_AC_BITS - 4u) + 54u)
#define BREDR_BTBB_MAX_PACKET_BITS (BREDR_BTBB_BASE_PACKET_BITS + BR_MAX_AIR_PAYLOAD_BITS)

static uint8_t bredr_frame_air_payload_bit(const bredr_frame_t *frame, unsigned int bit_pos)
{
    unsigned int byte_idx = bit_pos / 8u;
    unsigned int bit_idx = bit_pos % 8u;
    return (uint8_t)((frame->air_payload[byte_idx] >> bit_idx) & 1u);
}

static btbb_packet *btbb_packet_from_bredr(const bredr_frame_t *frame,
                                           int channel,
                                           uint32_t clkn)
{
    if (!frame || !frame->has_header)
        return NULL;

    char symbols[BREDR_BTBB_MAX_PACKET_BITS] = {0};
    unsigned int air_payload_bits = frame->air_payload_bits;
    const unsigned int max_payload = BR_MAX_AIR_PAYLOAD_BITS;
    if (air_payload_bits > max_payload)
        air_payload_bits = max_payload;

    uint64_t sw = bredr_gen_syncword(frame->lap & 0xFFFFFFu);
    for (unsigned int i = 0; i < 64u; i++)
        symbols[i] = (char)((sw >> i) & 1u);

    uint8_t sw_last = (uint8_t)((sw >> 63u) & 1u);
    uint8_t trailer = sw_last ? 0xAu : 0x5u;
    for (unsigned int i = 0; i < 4u; i++)
        symbols[64u + i] = (char)((trailer >> i) & 1u);

    for (unsigned int i = 0; i < 54u; i++)
        symbols[68u + i] = (char)((frame->header_raw >> i) & 1u);

    for (unsigned int i = 0; i < air_payload_bits; i++)
        symbols[122u + i] = (char)bredr_frame_air_payload_bit(frame, i);

    btbb_packet *bp = btbb_packet_new();
    if (!bp)
        return NULL;

    btbb_packet_set_data(bp,
                         symbols,
                         (int)(BREDR_BTBB_BASE_PACKET_BITS + air_payload_bits),
                         (uint8_t)channel,
                         clkn);
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
                                       const bredr_frame_t *frame,
                                       int channel,
                                       uint32_t clkn,
                                       uint8_t *uap_out,
                                       uint8_t *clk6_hint_out)
{
    if (!state || !state->piconet || !frame || !frame->has_header)
        return 0;

    btbb_packet *bp = btbb_packet_from_bredr(frame, channel, clkn);
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
                                       const bredr_frame_t *frame,
                                       int channel,
                                       uint32_t clkn,
                                       bredr_recovery_result_t *out)
{
    uint8_t uap = 0u;
    uint8_t clk6_hint = 0u;

    if (!out)
        return 0;

    if (!bredr_recovery_btbb_process_packet((bredr_recovery_btbb_state_t *)state,
                                            frame, channel, clkn, &uap, &clk6_hint))
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
