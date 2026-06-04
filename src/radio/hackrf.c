#include "hackrf.h"

#include <libhackrf/hackrf.h>

#include <complex.h>
#include <stdint.h>
#include <stdlib.h>

typedef struct
{
    hackrf_device *device;
    sample_dispatcher_t *dispatcher;
    int debug_enabled;
    uint64_t samples_received;
} hackrf_radio_t;

static inline float complex hackrf_iq_to_complex(const int8_t *samples,
                                                 unsigned int sample_index)
{
    return samples[2u * sample_index] / 128.0f +
           (samples[2u * sample_index + 1u] / 128.0f) * _Complex_I;
}

static int hackrf_rx_cb(hackrf_transfer *transfer)
{
    hackrf_radio_t *radio = transfer ? (hackrf_radio_t *)transfer->rx_ctx : NULL;
    sample_block_t *block;
    const int8_t *samples;
    unsigned int num_samples;

    if (!radio || !radio->dispatcher)
        return -1;
    if (!transfer->buffer)
        return -1;

    num_samples = (unsigned int)(transfer->valid_length / 2u);
    if (num_samples > SAMPLE_BLOCK_SAMPLE_CAPACITY)
        num_samples = SAMPLE_BLOCK_SAMPLE_CAPACITY;

    block = sample_dispatcher_acquire_block(radio->dispatcher);
    if (!block)
    {
        sample_dispatcher_note_drop(radio->dispatcher, radio->debug_enabled);
        return 0;
    }

    block->num_samples = num_samples;
    block->block_base_sample = radio->samples_received;
    radio->samples_received += num_samples;

    samples = (const int8_t *)transfer->buffer;
    for (unsigned int i = 0; i < num_samples; i++)
        block->samples[i] = hackrf_iq_to_complex(samples, i);

    __atomic_thread_fence(__ATOMIC_RELEASE);
    sample_dispatcher_push_block(radio->dispatcher, block);
    sample_block_release(block);

    return 0;
}

int hackrf_radio_open(void **out_device,
                      sample_dispatcher_t *dispatcher,
                      int debug_enabled)
{
    hackrf_radio_t *radio = NULL;
    int result;

    if (!out_device || !dispatcher)
        return HACKRF_ERROR_INVALID_PARAM;

    *out_device = NULL;
    radio = (hackrf_radio_t *)calloc(1, sizeof(*radio));
    if (!radio)
        return -1;

    radio->dispatcher = dispatcher;
    radio->debug_enabled = debug_enabled;

    result = hackrf_init();
    if (result != HACKRF_SUCCESS)
        goto fail;

    result = hackrf_open(&radio->device);
    if (result != HACKRF_SUCCESS)
    {
        hackrf_exit();
        goto fail;
    }

    *out_device = radio;
    return HACKRF_SUCCESS;

fail:
    free(radio);
    return result;
}

int hackrf_radio_configure(void *device, const radio_stream_config_t *config)
{
    hackrf_radio_t *radio = (hackrf_radio_t *)device;
    int result;

    if (!radio || !radio->device || !config)
        return HACKRF_ERROR_INVALID_PARAM;

    result = hackrf_set_lna_gain(radio->device, config->lna_gain);
    if (result != HACKRF_SUCCESS)
        return result;

    result = hackrf_set_vga_gain(radio->device, config->vga_gain);
    if (result != HACKRF_SUCCESS)
        return result;

    result = hackrf_set_freq(radio->device, config->lo_freq_hz);
    if (result != HACKRF_SUCCESS)
        return result;

    result = hackrf_set_sample_rate(radio->device, config->sample_rate);
    if (result != HACKRF_SUCCESS)
        return result;

    return HACKRF_SUCCESS;
}

int hackrf_radio_start_rx(void *device)
{
    hackrf_radio_t *radio = (hackrf_radio_t *)device;

    if (!radio || !radio->device || !radio->dispatcher)
        return HACKRF_ERROR_INVALID_PARAM;

    radio->samples_received = 0ULL;
    return hackrf_start_rx(radio->device, hackrf_rx_cb, radio);
}

int hackrf_radio_stop_rx(void *device)
{
    hackrf_radio_t *radio = (hackrf_radio_t *)device;
    if (!radio || !radio->device)
        return HACKRF_ERROR_INVALID_PARAM;

    return hackrf_stop_rx(radio->device);
}

void hackrf_radio_close(void *device)
{
    hackrf_radio_t *radio = (hackrf_radio_t *)device;
    if (!radio)
        return;

    if (radio->device)
        hackrf_close(radio->device);
    hackrf_exit();
    free(radio);
}