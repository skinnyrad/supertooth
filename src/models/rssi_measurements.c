/**
 * @file rssi_measurements.c
 * @brief Shared receive-side RSSI measurement helpers.
 */

#include "rssi_measurements.h"

#include <math.h>

#define RSSI_DBM_LIKE_OFFSET_DB 40.0f

static float receiver_rssi_apply_offset(float rssi_db)
{
    return rssi_db - RSSI_DBM_LIKE_OFFSET_DB;
}

float receiver_rssi_from_linear_power(float power, float invalid_value)
{
    if (power <= 0.0f)
        return invalid_value;

    return receiver_rssi_apply_offset(10.0f * log10f(power));
}

float receiver_rssi_from_mean_power_samples(const float complex *samples,
                                           unsigned int sample_count,
                                           float invalid_value)
{
    if (!samples || sample_count == 0u)
        return invalid_value;

    float sum_power = 0.0f;
    for (unsigned int i = 0u; i < sample_count; i++)
    {
        float re = crealf(samples[i]);
        float im = cimagf(samples[i]);
        sum_power += re * re + im * im;
    }

    return receiver_rssi_from_linear_power(sum_power / (float)sample_count, invalid_value);
}

float receiver_rssi_from_mean_power_range(const float complex *samples,
                                          unsigned int start_index,
                                          unsigned int end_index,
                                          float invalid_value)
{
    if (!samples || start_index >= end_index)
        return invalid_value;

    float sum_power = 0.0f;
    unsigned int sample_count = 0u;
    for (unsigned int i = start_index; i < end_index; i++)
    {
        float re = crealf(samples[i]);
        float im = cimagf(samples[i]);
        sum_power += re * re + im * im;
        sample_count++;
    }

    if (sample_count == 0u)
        return invalid_value;

    return receiver_rssi_from_linear_power(sum_power / (float)sample_count, invalid_value);
}
