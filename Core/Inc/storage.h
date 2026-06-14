/**
 * @file    storage.h
 * @brief   SD card storage layer (FatFS) for SensiNerveX Anomaly Detection
 *
 * Files written to SD root:
 *   raw_imu.csv      — raw ax,ay,az,gx,gy,gz,timestamp (toggleable)
 *   ae_weights.bin   — binary weight dump (header + CRC)
 *   anomaly_log.csv  — timestamped anomaly events with MSE and features
 *   run_stats.csv    — periodic stats (windows, anomalies, threshold)
 *
 * Binary weight file format (ae_weights.bin):
 *   [0x00] uint32  magic     = 0x534E5833 ('SNX3')
 *   [0x04] uint32  version   = (MAJOR<<16)|MINOR
 *   [0x08] uint32  n_weights = AE_WEIGHT_FLAT_SIZE
 *   [0x0C] uint32  crc16     = CRC-16/CCITT over weight bytes
 *   [0x10] float[] weights   = AE_WEIGHT_FLAT_SIZE floats
 *   [...]  float[] norm_mean = SNX_FEATURE_DIM floats
 *   [...]  float[] norm_var  = SNX_FEATURE_DIM floats
 *   [...]  uint32  norm_n    = NormStats.n_updates
 *   [...]  uint32  norm_init = NormStats.is_initialised
 *   [...]  float   threshold = anomaly threshold value
 */

#ifndef STORAGE_H
#define STORAGE_H

#include <stdint.h>
#include "ae_model.h"
#include "features.h"
#include "supervisor.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    SNX_OK    = 0,
    SNX_ERROR = 1,
} SNX_Status;

/**
 * @brief Mount SD card and create/open log files.
 * @return SNX_OK on success.
 */
SNX_Status Storage_Init(void);

/**
 * @brief Flush all open file handles.
 */
void Storage_Sync(void);

/**
 * @brief Unmount SD card gracefully.
 */
void Storage_Deinit(void);

/**
 * @brief Append one raw IMU sample to raw_imu.csv.
 *        Format: timestamp_ms,ax,ay,az,gx,gy,gz\r\n
 */
void Storage_Log_RawIMU(const IMU_Sample *s);

/**
 * @brief Append one anomaly event to anomaly_log.csv.
 *        Format: timestamp_ms,mse,threshold,f0..f9\r\n
 */
void Storage_Log_Anomaly(const AnomalyEvent *ev);

/**
 * @brief Append a stats row to run_stats.csv.
 */
void Storage_Log_Stats(uint32_t windows, uint32_t anomalies,
                       float threshold, float last_mse);

/**
 * @brief Serialise and write ae_weights.bin (weights + norm stats + threshold).
 */
SNX_Status Storage_SaveWeights(const AE_Model *m, const NormStats *ns);

/**
 * @brief Load ae_weights.bin, verify magic + CRC, populate model.
 */
SNX_Status Storage_LoadWeights(AE_Model *m, NormStats *ns);

/**
 * @brief Write threshold to a dedicated section of ae_weights.bin.
 *        (Appended at end of weight file — always re-saved with SaveWeights.)
 */
SNX_Status Storage_SaveThreshold(float threshold);

/**
 * @brief Read threshold from ae_weights.bin.
 */
SNX_Status Storage_LoadThreshold(float *threshold);

#ifdef __cplusplus
}
#endif

#endif /* STORAGE_H */
