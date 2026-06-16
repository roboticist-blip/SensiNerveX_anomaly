/**
 * @file    supervisor.h
 * @brief   Autonomous supervisor FSM for SensiNerveX Anomaly Detection
 *
 * States:
 *   IDLE        → Waiting for start command or power-on auto-start
 *   COLLECT     → Buffering IMU windows from MPU-6050
 *   CALIBRATE   → Accumulating normal-condition windows, computing
 *                 threshold = mean_mse + k*sigma_mse
 *   MONITOR     → Real-time anomaly detection (inference only)
 *   TRAIN       → Online training pass on buffered normal windows
 *   FALLBACK    → SD error or fatal sensor fault; UART alert only
 *
 * Transitions:
 *   IDLE       → COLLECT   : auto on boot, or 'C' command
 *   COLLECT    → CALIBRATE : window buffer primed (SNX_MIN_NORMAL_SAMPLES)
 *   CALIBRATE  → MONITOR   : SNX_CALIBRATION_WINDOWS collected OR timeout
 *   MONITOR    → TRAIN     : anomaly_streak == 0 && train_batch full
 *                            (train only on windows classified as NORMAL)
 *   TRAIN      → MONITOR   : training pass complete
 *   ANY        → FALLBACK  : fatal error flag set
 *   FALLBACK   → IDLE      : 'R' command (reset)
 */

#ifndef SUPERVISOR_H
#define SUPERVISOR_H

#include <stdint.h>
#include "snx_config.h"
#include "ae_model.h"
#include "features.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    SNX_STATE_IDLE      = 0,
    SNX_STATE_COLLECT   = 1,
    SNX_STATE_CALIBRATE = 2,
    SNX_STATE_MONITOR   = 3,
    SNX_STATE_TRAIN     = 4,
    SNX_STATE_FALLBACK  = 5,
    SNX_STATE_COUNT
} SNX_State;

typedef struct {
    uint32_t timestamp_ms;
    float    mse_score;
    float    threshold;
    float    features[SNX_FEATURE_DIM];
} AnomalyEvent;

typedef struct {
    SNX_State  state;
    SNX_State  prev_state;

    IMU_Sample window_buf[SNX_WINDOW_SIZE];
    uint32_t   window_idx;
    uint8_t    window_ready;

    float      feat[SNX_FEATURE_DIM];
    float      feat_norm[SNX_FEATURE_DIM]; 

    float      calib_mse_sum;
    float      calib_mse_sq_sum;
    uint32_t   calib_count;
    uint32_t   calib_start_tick;

    float      anomaly_threshold;
    uint32_t   anomaly_streak;
    uint32_t   total_anomalies;
    uint32_t   total_windows;

    float      train_buf[SNX_TRAIN_BATCH_SIZE][SNX_FEATURE_DIM];
    uint32_t   train_buf_idx;

    uint32_t   last_sd_sync_tick;
    uint8_t    raw_log_enabled;

    uint32_t   last_print_tick;

    uint8_t    flag_sd_error;
    uint8_t    flag_imu_error;

    uint8_t current_ground_truth;

} SNX_Supervisor;

extern AE_Model      g_ae;
extern NormStats     g_norm;
extern SNX_Supervisor g_sv;

/**
 * @brief Initialise supervisor, model, and normalisation stats.
 *        Call once after HAL_Init, before the main loop.
 */
void Supervisor_Init(void);

/**
 * @brief Main supervisor tick. Call every SNX_TICK_RATE_MS from main loop.
 */
void Supervisor_Tick(void);

/**
 * @brief Feed one IMU sample into the window buffer.
 *        Typically called from a timer ISR or DMA callback at SNX_SAMPLE_RATE_HZ.
 */
void Supervisor_FeedSample(const IMU_Sample *s);

/**
 * @brief Process UART command character. Call from UART RX ISR or polling loop.
 */
void Supervisor_HandleCommand(uint8_t cmd);

/**
 * @brief Force a state transition (for UART commands / testing).
 */
void Supervisor_ForceState(SNX_State new_state);

/**
 * @brief Set anomaly threshold manually (from 'H' UART command).
 */
void Supervisor_SetThreshold(float threshold);

/**
 * @brief Print current status over UART2 (called by 'S' command).
 */
void Supervisor_PrintStatus(void);

#ifdef __cplusplus
}
#endif

#endif /* SUPERVISOR_H */
