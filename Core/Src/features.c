/**
 * @file    features.c
 * @brief   Vibration feature extraction + Z-score normalisation
 */

#include "features.h"
#include <math.h>     /* sqrtf */
#include <string.h>   /* memset */

#define NORM_EPS  1e-6f   /* Variance floor to avoid div-by-zero */

void Features_Extract(const IMU_Sample *window, uint32_t window_len,
                      float *out_feat)
{
    float ax_sum = 0.f, ay_sum = 0.f, az_sum = 0.f;
    float ax_sq  = 0.f, ay_sq  = 0.f, az_sq  = 0.f;
    float peak_res = 0.f;

    for (uint32_t i = 0; i < window_len; i++) {
        float ax = window[i].ax;
        float ay = window[i].ay;
        float az = window[i].az;

        ax_sum += ax;  ax_sq += ax * ax;
        ay_sum += ay;  ay_sq += ay * ay;
        az_sum += az;  az_sq += az * az;

        float res = sqrtf(ax*ax + ay*ay + az*az);
        if (res > peak_res) peak_res = res;
    }

    float n = (float)window_len;

    float ax_mean = ax_sum / n;
    float ay_mean = ay_sum / n;
    float az_mean = az_sum / n;

    // Variance = E[x²] - E[x]²
    float ax_var = (ax_sq / n) - (ax_mean * ax_mean);
    float ay_var = (ay_sq / n) - (ay_mean * ay_mean);
    float az_var = (az_sq / n) - (az_mean * az_mean);

    if (ax_var < 0.f) ax_var = 0.f;
    if (ay_var < 0.f) ay_var = 0.f;
    if (az_var < 0.f) az_var = 0.f;

    float ax_std = sqrtf(ax_var);
    float ay_std = sqrtf(ay_var);
    float az_std = sqrtf(az_var);

    // RMS = sqrt(E[x²])
    float ax_rms = sqrtf(ax_sq / n);
    float ay_rms = sqrtf(ay_sq / n);
    float az_rms = sqrtf(az_sq / n);

    // Pack feature vector
    out_feat[0] = ax_mean;
    out_feat[1] = ax_std;
    out_feat[2] = ax_rms;
    out_feat[3] = ay_mean;
    out_feat[4] = ay_std;
    out_feat[5] = ay_rms;
    out_feat[6] = az_mean;
    out_feat[7] = az_std;
    out_feat[8] = az_rms;
    out_feat[9] = peak_res;
}

void Features_NormInit(NormStats *ns)
{
    memset(ns, 0, sizeof(NormStats));
    for (uint32_t i = 0; i < SNX_FEATURE_DIM; i++) {
        ns->var[i] = 1.0f;   
    }
    ns->is_initialised = 0;
    ns->n_updates = 0;
}

/*
 * Welford-style EMA update:
 *   mean ← (1-α)*mean + α*x
 *   var  ← (1-α)*var  + α*(x - mean_new)²
 *
 * First SNX_FEATURE_DIM calls prime the mean from scratch.
 */
void Features_NormUpdate(NormStats *ns, const float *feat)
{
    float alpha = SNX_NORM_ALPHA;

    if (ns->n_updates == 0) {
        /* First sample: set mean directly */
        for (uint32_t i = 0; i < SNX_FEATURE_DIM; i++) {
            ns->mean[i] = feat[i];
            ns->var[i]  = 1.0f;
        }
    } else {
        for (uint32_t i = 0; i < SNX_FEATURE_DIM; i++) {
            float new_mean = (1.0f - alpha) * ns->mean[i] + alpha * feat[i];
            float diff     = feat[i] - new_mean;
            ns->var[i]     = (1.0f - alpha) * ns->var[i] + alpha * diff * diff;
            ns->mean[i]    = new_mean;
        }
    }

    ns->n_updates++;
    if (ns->n_updates >= 10) {   
        ns->is_initialised = 1;
    }
}

void Features_NormApply(const NormStats *ns, float *feat)
{
    if (!ns->is_initialised) return;

    for (uint32_t i = 0; i < SNX_FEATURE_DIM; i++) {
        float std = sqrtf(ns->var[i] + NORM_EPS);
        feat[i] = (feat[i] - ns->mean[i]) / std;
    }
}

// Layout: [mean×10, var×10, (float)n_updates, (float)is_initialised] = 22 floats

void Features_NormToFlat(const NormStats *ns, float *buf)
{
    for (uint32_t i = 0; i < SNX_FEATURE_DIM; i++) buf[i]                    = ns->mean[i];
    for (uint32_t i = 0; i < SNX_FEATURE_DIM; i++) buf[SNX_FEATURE_DIM + i] = ns->var[i];
    buf[2 * SNX_FEATURE_DIM]     = (float)ns->n_updates;
    buf[2 * SNX_FEATURE_DIM + 1] = (float)ns->is_initialised;
}

void Features_NormFromFlat(NormStats *ns, const float *buf)
{
    for (uint32_t i = 0; i < SNX_FEATURE_DIM; i++) ns->mean[i] = buf[i];
    for (uint32_t i = 0; i < SNX_FEATURE_DIM; i++) ns->var[i]  = buf[SNX_FEATURE_DIM + i];
    ns->n_updates      = (uint32_t)buf[2 * SNX_FEATURE_DIM];
    ns->is_initialised = (uint8_t)buf[2 * SNX_FEATURE_DIM + 1];
}
