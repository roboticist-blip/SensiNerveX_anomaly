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
 * Binary weight file format (WEIGHTS.BIN) — UNCHANGED from v3.1:
 *   [0x00] uint32  magic     = 0x534E5833 ('SNX3')
 *   [0x04] uint32  version   = (MAJOR<<16)|MINOR
 *   [0x08] uint32  n_weights = AE_WEIGHT_FLAT_SIZE
 *   [0x0C] uint32  crc16     = CRC-16/CCITT over weight bytes
 *   [0x10] float[] weights   = AE_WEIGHT_FLAT_SIZE floats
 *   [...]  float[] norm_mean = SNX_FEATURE_DIM floats
 *   [...]  float[] norm_var  = SNX_FEATURE_DIM floats
 *   [...]  uint32  norm_n    = NormStats.n_updates
 *   [...]  uint32  norm_init = NormStats.is_initialised
 *   [...]  float   threshold = legacy single-threshold value (unused
 *                               once baselines are active, kept for
 *                               the manual 'C'/CALIBRATE bench path)
 
 *   CYCLE.BIN     — raw IMU_Sample stream captured during RECORD state
 *                   (append-only during capture, random-access read
 *                   during SEGMENT replay)
 *   BASELINE.BIN  — array of SNX_Baseline (centroid + threshold) per
 *                   detected operating state, written by SEGMENT,
 *                   loaded at boot
 *   SNXCFG.BIN    — single byte: operator-configured max_states ceiling
 *                   for this specific machine
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
 * @param prediction  1 if mse > active baseline's threshold, else 0
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
 * @param window_id  Calibration window index (1-based, RESETS per
 *                   plateau when called from SEGMENT's per-plateau
 *                   calibration — see calibrate_one_plateau() in
 *                   supervisor.c)
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

/**
 * @brief Open CYCLE.BIN for writing (truncates any previous recording).
 *        Call once when entering SNX_STATE_RECORD.
 */
SNX_Status Storage_OpenCycleRecording(void);

/**
 * @brief Append one raw IMU sample to CYCLE.BIN.
 *        Called from Supervisor_FeedSample() while in SNX_STATE_RECORD.
 */
void Storage_WriteCycleSample(const IMU_Sample *s);

/**
 * @brief Close CYCLE.BIN. Called on 'D' command, buffer cap, or
 *        SNX_CYCLE_AUTO_TIMEOUT_MS.
 */
void Storage_CloseCycleRecording(void);

/**
 * @brief Random-access read of one sample from CYCLE.BIN by index.
 *        Used during SNX_STATE_SEGMENT replay. Opens/seeks/reads/closes
 *        per call — SEGMENT is a one-time commissioning pass, not a
 *        real-time path, so per-call open/close overhead is acceptable.
 * @return SNX_OK on success, SNX_ERROR if index out of range or I/O fails.
 */
SNX_Status Storage_ReadCycleSample(uint32_t index, IMU_Sample *out);

/**
 * @return Number of samples currently stored in CYCLE.BIN
 *         (file size / sizeof(IMU_Sample)). Returns 0 if not mounted
 *         or file does not exist.
 */
uint32_t Storage_GetCycleSampleCount(void);

/**
 * @brief Save the full baseline array to BASELINE.BIN.
 * @param baselines  Array of `count` SNX_Baseline entries
 * @param count      Number of valid baselines (== g_sv.baseline_count)
 */
SNX_Status Storage_SaveBaselines(const SNX_Baseline *baselines, uint8_t count);

/**
 * @brief Load baselines from BASELINE.BIN.
 * @param baselines  Output array of at least SNX_MAX_STATES_CEILING slots
 * @param count_out  Populated with how many were loaded
 * @return SNX_ERROR if file missing, header invalid, or short read —
 *         caller should treat this as "commissioning required."
 */
SNX_Status Storage_LoadBaselines(SNX_Baseline *baselines, uint8_t *count_out);

/**
 * @brief Persist operator-configured max_states so it survives reboot.
 */
SNX_Status Storage_SaveMaxStates(uint8_t max_states);

/**
 * @brief Load persisted max_states.
 * @return SNX_ERROR if file missing or value out of range — caller
 *         should fall back to a sensible default (SNX_MAX_STATES_CEILING).
 */
SNX_Status Storage_LoadMaxStates(uint8_t *max_states_out);

#ifdef __cplusplus
}
#endif

#endif /* STORAGE_H */