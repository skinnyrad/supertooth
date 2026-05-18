/**
 * @file bredr_measurements.c
 * @brief BR/EDR receive-side measurement helpers.
 */

#include "bredr_measurements.h"

#include <complex.h>
#include <math.h>

float bredr_compute_rssi_dbr(const float complex *samples, unsigned int sample_count)
{
    if (!samples || sample_count == 0u)
        return -99.0f;

    float sum_power = 0.0f;
    for (unsigned int i = 0u; i < sample_count; i++)
    {
        float re = crealf(samples[i]);
        float im = cimagf(samples[i]);
        sum_power += re * re + im * im;
    }

    float mean_power = sum_power / (float)sample_count;
    return (mean_power > 0.0f) ? 10.0f * log10f(mean_power) : -99.0f;
}
