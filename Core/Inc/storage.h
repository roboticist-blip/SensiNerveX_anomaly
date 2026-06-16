/**
 * @file    storage.h
 * @brief   SD card storage layer (FatFS) for SensiNerveX Anomaly Detection
 *
 * Existing files (unchanged):
 *   RAWIMU.CSV    — raw ax,ay,az,gx,gy,gz,timestamp (toggleable)
 *   WEIGHTS.BIN   — binary weight dump (header + CRC)
 *   ANOMLOG.CSV   — timestamped anomaly events with MSE and features
 *   RUNSTAT.CSV   — periodic stats (windows, anomalies, threshold)
 *
 * Research logging files added in v3.1 (Objectives 2–6):
 *   EVAL.CSV      — per-window: window_id, timestamp_ms, mse, label, prediction
 *   LATENT.CSV    — per-window: timestamp_ms, z1..z4, label
 *   TRAIN.CSV     — per-epoch:  session_id, epoch, loss
 *   CALIB.CSV     — per-calib-window: window_id, mse
 *   PROFILE.CSV   — per-window: window_id, cycles, time_us
 *
 * Binary weight file format (WEIGHTS.BIN):
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

SNX_Status Storage_Init(void);
void       Storage_Sync(void);
void       Storage_Deinit(void);
void       Storage_Log_RawIMU(const IMU_Sample *s);
void       Storage_Log_Anomaly(const AnomalyEvent *ev);
void       Storage_Log_Stats(uint32_t windows, uint32_t anomalies,
                              float threshold, float last_mse);
SNX_Status Storage_SaveWeights(const AE_Model *m, const NormStats *ns);
SNX_Status Storage_LoadWeights(AE_Model *m, NormStats *ns);
SNX_Status Storage_SaveThreshold(float threshold);
SNX_Status Storage_LoadThreshold(float *threshold);


/**
 * @brief Append one row to EVAL.csv.
 *
 * @param window_id   Sequential window counter (MONITOR scope)
 * @param timestamp   HAL_GetTick() at window completion
 * @param mse         Reconstruction MSE from AE_Forward()
 * @param label       Ground-truth SNX_FaultLabel (0 = NORMAL)
 * @param prediction  1 if mse > anomaly_threshold, else 0
 *
 * Row format: window_id,timestamp_ms,mse,ground_truth_label,prediction
 */
void Storage_Log_Eval(uint32_t window_id, uint32_t timestamp,
                      float mse, uint8_t label, uint8_t prediction);

/**
 * @brief Append one row to LATENT.csv.
 *
 * @param timestamp HAL_GetTick() at window completion
 * @param z         Pointer to AE_Activations.z[AE_Z] (4 floats)
 * @param label     Ground-truth SNX_FaultLabel
 *
 * Row format: timestamp_ms,z1,z2,z3,z4,ground_truth_label
 */
void Storage_Log_Latent(uint32_t timestamp,
                        const float *z, uint8_t label);

/**
 * @brief Append one epoch row to TRAIN.csv.
 *
 * @param session_id  Incremented each time TRAIN state is entered
 * @param epoch       Epoch index (0-based)
 * @param loss        Mean loss for this epoch
 *
 * Row format: session_id,epoch,loss
 */
void Storage_Log_Train(uint32_t session_id, uint32_t epoch, float loss);

/**
 * @brief Append one row to CALIB.csv.
 *
 * @param window_id  Calibration window index (1-based)
 * @param mse        Reconstruction MSE during calibration
 *
 * Row format: window_id,mse
 */
void Storage_Log_Calibration(uint32_t window_id, float mse);

/**
 * @brief Append one row to PROFILE.csv.
 *
 * @param window_id     Sequential window counter
 * @param cycles        DWT->CYCCNT elapsed cycles for AE_Forward()
 * @param time_us       cycles / SNX_CPU_MHZ  (float microseconds)
 *
 * Row format: window_id,cycles,time_us
 */
void Storage_Log_Profile(uint32_t window_id,
                         uint32_t cycles, float time_us);

#ifdef __cplusplus
}
#endif

#endif /* STORAGE_H */
