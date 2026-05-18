/**
 * @file rssi_measurements.h
 * @brief Shared receive-side RSSI measurement helpers.
 */

#ifndef RSSI_MEASUREMENTS_H
#define RSSI_MEASUREMENTS_H

#include <complex.h>

#ifdef __cplusplus
extern "C" {
#endif

float receiver_rssi_from_linear_power(float power, float invalid_value);
float receiver_rssi_from_mean_power_samples(const float complex *samples,
                                           unsigned int sample_count,
                                           float invalid_value);
float receiver_rssi_from_mean_power_range(const float complex *samples,
                                          unsigned int start_index,
                                          unsigned int end_index,
                                          float invalid_value);

#ifdef __cplusplus
}
#endif

#endif /* RSSI_MEASUREMENTS_H */
