#ifndef HACKRF_WRAPPER_H
#define HACKRF_WRAPPER_H

#include "radio_common.h"

int hackrf_radio_open(void **out_device,
					  sample_dispatcher_t *dispatcher,
					  int debug_enabled);
int hackrf_radio_configure(void *device, const radio_stream_config_t *config);
int hackrf_radio_start_rx(void *device);
int hackrf_radio_stop_rx(void *device);
void hackrf_radio_close(void *device);

#endif // HACKRF_WRAPPER_H