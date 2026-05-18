/**
 * @file bredr_measurements.h
 * @brief BR/EDR receive-side measurement helpers.
 */

#ifndef BREDR_MEASUREMENTS_H
#define BREDR_MEASUREMENTS_H

#include <complex.h>

#ifdef __cplusplus
extern "C" {
#endif

float bredr_compute_rssi_dbr(const float complex *samples, unsigned int sample_count);

#ifdef __cplusplus
}
#endif

#endif /* BREDR_MEASUREMENTS_H */
