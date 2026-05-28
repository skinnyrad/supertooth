#include "radio_common.h"

#include "hackrf.h"

#include <stdlib.h>

struct radio_device
{
    radio_device_type_t device_type;
    void *impl;
};

int radio_open(radio_device_t **out_device,
               radio_device_type_t device_type,
               sample_dispatcher_t *dispatcher,
               int debug_enabled)
{
    radio_device_t *device = NULL;
    int result = -1;

    if (!out_device || !dispatcher)
        return -1;

    *out_device = NULL;
    device = (radio_device_t *)calloc(1, sizeof(*device));
    if (!device)
        return -1;

    device->device_type = device_type;

    switch (device_type)
    {
    case RADIO_DEVICE_HACKRF:
        result = hackrf_radio_open(&device->impl, dispatcher, debug_enabled);
        break;
    default:
        result = -1;
        break;
    }

    if (result != RADIO_SUCCESS)
    {
        free(device);
        return result;
    }

    *out_device = device;
    return RADIO_SUCCESS;
}

int radio_configure(radio_device_t *device, const radio_stream_config_t *config)
{
    if (!device)
        return -1;

    switch (device->device_type)
    {
    case RADIO_DEVICE_HACKRF:
        return hackrf_radio_configure(device->impl, config);
    default:
        return -1;
    }
}

int radio_start_rx(radio_device_t *device)
{
    if (!device)
        return -1;

    switch (device->device_type)
    {
    case RADIO_DEVICE_HACKRF:
        return hackrf_radio_start_rx(device->impl);
    default:
        return -1;
    }
}

int radio_stop_rx(radio_device_t *device)
{
    if (!device)
        return -1;

    switch (device->device_type)
    {
    case RADIO_DEVICE_HACKRF:
        return hackrf_radio_stop_rx(device->impl);
    default:
        return -1;
    }
}

void radio_close(radio_device_t *device)
{
    if (!device)
        return;

    switch (device->device_type)
    {
    case RADIO_DEVICE_HACKRF:
        hackrf_radio_close(device->impl);
        break;
    default:
        break;
    }

    free(device);
}
