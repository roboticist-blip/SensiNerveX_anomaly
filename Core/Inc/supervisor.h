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
 *
 * Research additions (v3.1):
 *   - SNX_FaultLabel enum + current_label field for ground-truth injection
 *   - UART '0'..'5' commands update current_label in real-time
 *   - train_session_id incremented each time TRAIN state is entered
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

/* ── Ground-truth fault labels
 *   Injected via UART commands '0'..'5'.
 *   Stored alongside every EVAL.csv row for offline confusion matrix.
*/
typedef enum {
    SNX_LABEL_NORMAL        = 0,
    SNX_LABEL_IMPACT        = 1,
    SNX_LABEL_IMBALANCE     = 2,
    SNX_LABEL_LOOSE_MOUNT   = 3,
    SNX_LABEL_MISALIGNMENT  = 4,
    SNX_LABEL_BEARING_FAULT = 5
} SNX_FaultLabel;

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
    float      calib_mse[SNX_CALIBRATION_WINDOWS];
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


    SNX_FaultLabel current_label;       /**< Ground-truth label (Obj 1) */
    uint32_t       train_session_id;    /**< Incremented per TRAIN entry (Obj 4) */
    uint32_t       monitor_window_id;   /**< Sequential window counter for EVAL/PROFILE */

    uint8_t current_ground_truth;

} SNX_Supervisor;

extern AE_Model       g_ae;
extern NormStats      g_norm;
extern SNX_Supervisor g_sv;

void Supervisor_Init(void);
void Supervisor_Tick(void);
void Supervisor_FeedSample(const IMU_Sample *s);
void Supervisor_HandleCommand(uint8_t cmd);
void Supervisor_ForceState(SNX_State new_state);
void Supervisor_SetThreshold(float threshold);
void Supervisor_PrintStatus(void);

#ifdef __cplusplus
}
#endif

#endif /* SUPERVISOR_H */
