#ifndef RADIO_COMMON_H
#define RADIO_COMMON_H

#include <stdint.h>

#include "sample_dispatcher.h"

#define RADIO_SUCCESS 0

typedef struct
{
    uint64_t lo_freq_hz;
    uint32_t sample_rate;
    uint32_t lna_gain;
    uint32_t vga_gain;
} radio_stream_config_t;

typedef enum
{
    RADIO_DEVICE_HACKRF = 0,
} radio_device_type_t;

typedef struct radio_device radio_device_t;

int radio_open(radio_device_t **out_device,
               radio_device_type_t device_type,
               sample_dispatcher_t *dispatcher,
               int debug_enabled);
int radio_configure(radio_device_t *device, const radio_stream_config_t *config);
int radio_start_rx(radio_device_t *device);
int radio_stop_rx(radio_device_t *device);
void radio_close(radio_device_t *device);

#endif
