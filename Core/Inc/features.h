/**
 * @file    features.h
 * @brief   Feature extraction from MPU-6050 vibration windows
 *
 * Extracts SNX_FEATURE_DIM = 10 statistical features per window:
 *   [0]  ax_mean
 *   [1]  ax_std
 *   [2]  ax_rms
 *   [3]  ay_mean
 *   [4]  ay_std
 *   [5]  ay_rms
 *   [6]  az_mean
 *   [7]  az_std
 *   [8]  az_rms
 *   [9]  resultant_peak   (max √(ax²+ay²+az²) in window)
 *
 * Z-score normalisation is applied in-place using running statistics
 * (exponential moving average of mean and variance).
 */

#ifndef FEATURES_H
#define FEATURES_H

#include <stdint.h>
#include "snx_config.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float ax, ay, az;   
    float gx, gy, gz;   
    uint32_t timestamp; 
} IMU_Sample;

typedef struct {
    float mean[SNX_FEATURE_DIM];
    float var[SNX_FEATURE_DIM];
    uint32_t n_updates;             
    uint8_t  is_initialised;
} NormStats;

/**
 * @brief Extract feature vector from a completed window buffer.
 * @param window     Circular buffer of SNX_WINDOW_SIZE IMU_Sample
 * @param out_feat   Output: float[SNX_FEATURE_DIM]  (raw, not normalised)
 */
void Features_Extract(const IMU_Sample *window, uint32_t window_len,
                      float *out_feat);

/**
 * @brief Initialise normalisation statistics (all zeros/vars=1).
 */
void Features_NormInit(NormStats *ns);

/**
 * @brief Update running mean/var with a new feature vector (EMA).
 *        Use during calibration phase on known-normal data.
 */
void Features_NormUpdate(NormStats *ns, const float *feat);

/**
 * @brief Apply Z-score normalisation in-place: (x - mean) / sqrt(var + eps)
 *        If not initialised, feat is returned unchanged.
 */
void Features_NormApply(const NormStats *ns, float *feat);

/**
 * @brief Serialise NormStats to flat array (for SD save).
 *        buf must hold 2*SNX_FEATURE_DIM + 2 floats.
 */
void Features_NormToFlat(const NormStats *ns, float *buf);

/**
 * @brief Deserialise NormStats from flat array.
 */
void Features_NormFromFlat(NormStats *ns, const float *buf);

#ifdef __cplusplus
}
#endif

#endif /* FEATURES_H */
