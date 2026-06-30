/**
 * @file dsp/rssi_measurements.h
 * @brief Shared receive-side RSSI measurement helpers.
 */

#ifndef RSSI_MEASUREMENTS_H
#define RSSI_MEASUREMENTS_H

#include <complex.h>
#include <math.h>

#ifdef __cplusplus
extern "C" {
#endif

#define RECEIVER_RSSI_INVALID ((float)NAN)

float receiver_rssi_from_linear_power(float power, float invalid_value);
float receiver_rssi_from_mean_power_range(const float complex *samples,
                                          unsigned int start_index,
                                          unsigned int end_index,
                                          float invalid_value);

#ifdef __cplusplus
}
#endif

#endif /* RSSI_MEASUREMENTS_H */
