/**
 * @file    supervisor.c
 * @brief   SensiNerveX v3.2 — Anomaly Detection Supervisor FSM
 *
 * All heavy data structures live in CCM SRAM (64 KB on STM32F405).
 * CCM is connected directly to the CPU data bus — no DMA, fastest access.
 *
 * Changes versus v3.1 (research logging additions) — v3.2 adds
 * Recorded-Cycle Commissioning (multi-baseline calibration):
 *
 *   Problem solved: a machine with multiple legitimate operating states
 *   (idle, spin-up, steady-run, tool-change, shutdown, etc.) was
 *   previously calibrated against a single window of "normal," causing
 *   every other legitimate state to be flagged as anomalous.
 *
 *   New states: SNX_STATE_RECORD, SNX_STATE_SEGMENT
 *     RECORD  — streams raw IMU samples to CYCLE.BIN while the operator
 *               runs the machine through one full representative cycle.
 *               Entered via 'G', exited via 'D' (or auto-timeout/buffer
 *               cap).
 *     SEGMENT — replays CYCLE.BIN from SD, non-causally detects stable
 *               "plateaus" (distinct operating states) using the
 *               resultant_peak feature, then runs the EXISTING
 *               calibration algorithm (AE_Forward/Backward/Update +
 *               median + 10*MAD threshold) once per plateau via
 *               calibrate_one_plateau(). Produces one SNX_Baseline
 *               (feature centroid + threshold) per detected state.
 *
 *   handle_monitor() now performs a nearest-centroid lookup
 *   (find_nearest_baseline()) before comparing reconstruction MSE
 *   against a threshold — the threshold used is the ACTIVE baseline's,
 *   not a single global value. The autoencoder itself is NOT duplicated
 *   per state: one shared g_ae is trained sequentially across all
 *   plateaus during SEGMENT, so it learns a representation expected to
 *   reconstruct every known-normal state well. Only the threshold and
 *   centroid are per-state.
 *
 *   'H' (SNX_CMD_SET_THRESHOLD) is repurposed: it now overrides the
 *   threshold of whichever baseline is CURRENTLY ACTIVE in MONITOR,
 *   rather than a single global g_sv.anomaly_threshold (which
 *   handle_monitor() no longer reads once baselines exist). This keeps
 *   a manual field-tuning escape hatch available per operating state.
 *
 *   The original single-shot manual path ('C' → COLLECT → CALIBRATE)
 *   is KEPT as-is for quick bench recalibration / testing without a
 *   full commissioning recording — it still writes g_sv.anomaly_threshold
 *   and is otherwise byte-for-byte unchanged from v3.1. It is not the
 *   recommended path for real deployments; 'G' (RECORD) is.
 *
 *   A new minimal UART numeric-entry accumulator
 *   (handle_pending_numeric_byte()) was added because no line-buffering
 *   mechanism existed anywhere in the project — DBG_RxPoll() delivers
 *   exactly one byte per call, non-blocking, and SNX_CMD_SET_THRESHOLD's
 *   v3.1 handler was a prompt-only stub that never consumed a reply.
 *   This accumulator now backs both 'H' and the new 'M' (SET_MAX_STATES)
 *   command.
 *
 *   Objective 1 — SNX_FaultLabel + UART commands '0'..'5'
 *   Objective 2 — EVAL.csv
 *   Objective 3 — LATENT.csv
 *   Objective 4 — TRAIN.csv
 *   Objective 5 — CALIB.csv
 *   Objective 6 — PROFILE.csv
 *   Objective 7 — Resource summary
 *
 * The core anomaly detection algorithm, autoencoder, and feature
 * extraction pipeline are NOT modified.
 */

#include "supervisor.h"
#include "storage.h"
#include "imu.h"
#include "debug_uart.h"
#include "snx_config.h"
#include "main.h"
#include "stm32f4xx_hal.h"

#include <string.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

__attribute__((section(".ccmram"))) AE_Model       g_ae;
__attribute__((section(".ccmram"))) NormStats      g_norm;
__attribute__((section(".ccmram"))) SNX_Supervisor g_sv;


static const char *const STATE_NAMES[SNX_STATE_COUNT] = {
    "IDLE", "COLLECT", "CALIBRATE", "MONITOR", "TRAIN", "FALLBACK",
    "RECORD", "SEGMENT"
};

static const char *const LABEL_NAMES[] = {
    "NORMAL", "IMPACT", "IMBALANCE", "LOOSE_MOUNT", "MISALIGNMENT", "BEARING_FAULT"
};

static void transition(SNX_State new_state)
{
    if (g_sv.state == new_state) return;
    DBG_Printf("[FSM] %s → %s\r\n",
               STATE_NAMES[g_sv.state], STATE_NAMES[new_state]);
    g_sv.prev_state = g_sv.state;
    g_sv.state = new_state;
}

static void reset_calibration(void)
{
    g_sv.calib_mse_sum    = 0.0f;
    g_sv.calib_mse_sq_sum = 0.0f;
    g_sv.calib_count      = 0;
    g_sv.calib_start_tick = HAL_GetTick();
}

/*
 * Parameter count (hand-counted from ae_model.h):
 *   enc_W1: AE_H1 * AE_IN  = 8*10 = 80
 *   enc_b1: AE_H1           =      8
 *   enc_W2: AE_Z  * AE_H1  = 4*8  = 32
 *   enc_b2: AE_Z            =      4
 *   dec_W1: AE_H2 * AE_Z   = 8*4  = 32
 *   dec_b1: AE_H2           =      8
 *   dec_W2: AE_OUT* AE_H2  = 10*8 = 80
 *   dec_b2: AE_OUT          =      10
 *   ─────────────────────────────────
 *   Total weights:                  254   (AE_WEIGHT_FLAT_SIZE)
 *   Biases are included in above count per layer pair convention.
 *   Grand total trainable scalars = 254 floats = 1016 bytes
 *
 * RAM layout (CCM section):
 *   sizeof(AE_Model)       includes w + v + act + grad + metadata
 *   sizeof(NormStats)      mean[10] + var[10] + n_updates + is_initialised
 *   sizeof(SNX_Supervisor) window_buf + feat + train_buf + flags +
 *                          baselines[SNX_MAX_STATES_CEILING] (v3.2)
 */
static void print_resource_summary(void)
{
    const uint32_t n_enc_W1 = (uint32_t)(AE_H1 * AE_IN);
    const uint32_t n_enc_b1 = (uint32_t)(AE_H1);
    const uint32_t n_enc_W2 = (uint32_t)(AE_Z  * AE_H1);
    const uint32_t n_enc_b2 = (uint32_t)(AE_Z);
    const uint32_t n_dec_W1 = (uint32_t)(AE_H2 * AE_Z);
    const uint32_t n_dec_b1 = (uint32_t)(AE_H2);
    const uint32_t n_dec_W2 = (uint32_t)(AE_OUT * AE_H2);
    const uint32_t n_dec_b2 = (uint32_t)(AE_OUT);
    const uint32_t n_params  = n_enc_W1 + n_enc_b1 + n_enc_W2 + n_enc_b2
                               + n_dec_W1 + n_dec_b1 + n_dec_W2 + n_dec_b2;

    /*
     * sizeof(AE_Weights) covers one copy of parameters.
     * AE_Model contains: AE_Weights w + AE_Velocity v + AE_Activations act
     *                    + AE_Gradients grad + train_steps + last_mse + is_trained
     * Weight storage on SD: header(16) + weights(n_params*4) + norm(80+80+8) + thr(4)
     */
    const uint32_t model_ram_bytes    = (uint32_t)sizeof(AE_Model);
    const uint32_t weight_bytes       = n_params * 4u;          /* float32 */
    const uint32_t sd_file_bytes      = 16u + weight_bytes
                                        + SNX_FEATURE_DIM * 4u * 2u
                                        + 4u + 4u + 4u;         /* header+norm+thr */
    const uint32_t supervisor_bytes   = (uint32_t)sizeof(SNX_Supervisor);

    DBG_Printf("\r\n=== SensiNerveX v%d.%d Resource Summary ===\r\n",
               SNX_FW_VERSION_MAJOR, SNX_FW_VERSION_MINOR);
    DBG_Printf("  AE architecture    : %u→%u→%u→%u→%u\r\n",
               (unsigned)AE_IN, (unsigned)AE_H1, (unsigned)AE_Z,
               (unsigned)AE_H2, (unsigned)AE_OUT);
    DBG_Printf("  AE param count     : %lu floats\r\n",   (unsigned long)n_params);
    DBG_Printf("  AE weight bytes    : %lu bytes (float32)\r\n", (unsigned long)weight_bytes);
    DBG_Printf("  AE model RAM       : %lu bytes (w+v+act+grad+meta)\r\n",
               (unsigned long)model_ram_bytes);
    DBG_Printf("  SD weights file    : %lu bytes (header+weights+norm+thr)\r\n",
               (unsigned long)sd_file_bytes);
    DBG_Printf("  Supervisor RAM     : %lu bytes\r\n",    (unsigned long)supervisor_bytes);
    DBG_Printf("  NormStats RAM      : %lu bytes\r\n",    (unsigned long)sizeof(NormStats));
    DBG_Printf("  Total CCM usage    : %lu bytes\r\n",
               (unsigned long)(model_ram_bytes + supervisor_bytes + sizeof(NormStats)));
    DBG_Printf("  Train steps (boot) : %lu\r\n",          (unsigned long)g_ae.train_steps);
    DBG_Printf("  Sample rate        : %u Hz\r\n",        (unsigned)SNX_SAMPLE_RATE_HZ);
    DBG_Printf("  Window size/stride : %u / %u samples\r\n",
               (unsigned)SNX_WINDOW_SIZE, (unsigned)SNX_WINDOW_STRIDE);
    DBG_Printf("  Feature dim        : %u\r\n",            (unsigned)SNX_FEATURE_DIM);
    DBG_Printf("  Train batch/epochs : %u / %u\r\n",
               (unsigned)SNX_TRAIN_BATCH_SIZE, (unsigned)SNX_MAX_EPOCHS);
    DBG_Printf("  Max states ceiling : %u (operator-set: %u)\r\n",
               (unsigned)SNX_MAX_STATES_CEILING, (unsigned)g_sv.max_states);
    DBG_Printf("  Baselines loaded   : %u\r\n",            (unsigned)g_sv.baseline_count);
    DBG_Printf("==========================================\r\n\r\n");
}

void Supervisor_Init(void)
{
    memset(&g_sv, 0, sizeof(SNX_Supervisor));
    g_sv.state              = SNX_STATE_IDLE;
    g_sv.raw_log_enabled    = 1;
    g_sv.current_label      = SNX_LABEL_NORMAL; 
    g_sv.train_session_id   = 0;                 
    g_sv.monitor_window_id  = 0;                 
    g_sv.max_states         = SNX_MAX_STATES_CEILING; 

    AE_Init(&g_ae);
    Features_NormInit(&g_norm);

    if (Storage_Init() == SNX_OK) {
        DBG_Printf("[INIT] SD card OK\r\n");

        uint8_t ms = 0;
        if (Storage_LoadMaxStates(&ms) == SNX_OK) {
            g_sv.max_states = ms;
        }

        SNX_Status ws = Storage_LoadWeights(&g_ae, &g_norm);
        if (ws == SNX_OK) {
            DBG_Printf("[INIT] Weights loaded from SD. Steps=%lu\r\n",
                       (unsigned long)g_ae.train_steps);

            if (g_ae.is_trained && g_norm.is_initialised) {
                uint8_t loaded_count = 0;
                if (Storage_LoadBaselines(g_sv.baselines, &loaded_count) == SNX_OK) {
                    g_sv.baseline_count      = loaded_count;
                    g_sv.active_baseline_idx = 0;
                    DBG_Printf("[INIT] Loaded %u baseline(s) — commissioning "
                               "already complete.\r\n", (unsigned)loaded_count);
                    print_resource_summary();
                    g_sv.state = SNX_STATE_MONITOR;
                    return;
                }

                float saved_thresh = 0.0f;
                if (Storage_LoadThreshold(&saved_thresh) == SNX_OK) {
                    g_sv.anomaly_threshold = saved_thresh;
                    DBG_Printf("[INIT] No baselines found, but legacy "
                               "threshold loaded: %.6f. Use 'G' to run "
                               "full commissioning, or 'C' to continue "
                               "with single-baseline mode.\r\n",
                               (double)saved_thresh);
                    print_resource_summary();
                    g_sv.state = SNX_STATE_COLLECT;
                    return;
                }

                DBG_Printf("[INIT] Weights trained but no baselines or "
                           "threshold found. Commissioning required.\r\n");
            }
        } else {
            DBG_Printf("[INIT] No saved weights, starting fresh.\r\n");
        }
    } else {
        DBG_Printf("[INIT] SD card FAIL — logging disabled\r\n");
        g_sv.flag_sd_error = 1;
    }

    print_resource_summary();

    DBG_Printf("[INIT] SensiNerveX v%d.%d Node 0x%02X ready.\r\n",
               SNX_FW_VERSION_MAJOR, SNX_FW_VERSION_MINOR, SNX_NODE_ID);
    DBG_Printf("[INIT] Send 'G' to begin commissioning (recommended), "
               "or 'C' for quick single-baseline calibration.\r\n");
    DBG_Printf("[INIT] Label cmds: '0'=NORMAL '1'=IMPACT '2'=IMBALANCE "
               "'3'=LOOSE_MOUNT '4'=MISALIGN '5'=BEARING\r\n");
    g_sv.state = SNX_STATE_IDLE;
}

void Supervisor_FeedSample(const IMU_Sample *s)
{
    if (g_sv.state == SNX_STATE_IDLE || g_sv.state == SNX_STATE_FALLBACK)
        return;

    if (g_sv.raw_log_enabled && !g_sv.flag_sd_error) {
        Storage_Log_RawIMU(s);
    }

    if (g_sv.state == SNX_STATE_RECORD) {
        if (!g_sv.flag_sd_error &&
            g_sv.cycle_sample_count < SNX_CYCLE_MAX_SAMPLES) {
            Storage_WriteCycleSample(s);
            g_sv.cycle_sample_count++;
        }
        return;
    }

    g_sv.window_buf[g_sv.window_idx++] = *s;

    if (g_sv.window_idx >= SNX_WINDOW_SIZE) {
        g_sv.window_idx  = SNX_WINDOW_STRIDE;
        memmove(&g_sv.window_buf[0],
                &g_sv.window_buf[SNX_WINDOW_STRIDE],
                (SNX_WINDOW_SIZE - SNX_WINDOW_STRIDE) * sizeof(IMU_Sample));
        g_sv.window_ready = 1;
    }
}

static void handle_collect(void)
{
    if (!g_sv.window_ready) return;
    g_sv.window_ready = 0;

    Features_Extract(g_sv.window_buf, SNX_WINDOW_SIZE, g_sv.feat);

    if (g_sv.total_windows >= SNX_MIN_NORMAL_SAMPLES) {
        DBG_Printf("[COLLECT] %lu normal windows buffered. Starting calibration.\r\n",
                   (unsigned long)g_sv.total_windows);
        reset_calibration();
        transition(SNX_STATE_CALIBRATE);
        return;
    }

    Features_NormUpdate(&g_norm, g_sv.feat);
    g_sv.total_windows++;

    DBG_Printf("[COLLECT] window=%lu\r\n", (unsigned long)g_sv.total_windows);
}

static int float_compare(const void *a, const void *b)
{
    float fa = *(const float *)a;
    float fb = *(const float *)b;

    if (fa < fb) return -1;
    if (fa > fb) return 1;
    return 0;
}

static void handle_calibrate(void)
{
    if (!g_sv.window_ready)
        return;

    g_sv.window_ready = 0;

    if ((HAL_GetTick() - g_sv.calib_start_tick) >
        SNX_CALIBRATION_TIMEOUT_MS)
    {
        DBG_Printf("[CALIB] Timeout — using collected stats.\r\n");
        goto compute_threshold;
    }

    Features_Extract(
        g_sv.window_buf,
        SNX_WINDOW_SIZE,
        g_sv.feat);

    memcpy(
        g_sv.feat_norm,
        g_sv.feat,
        sizeof(g_sv.feat));

    Features_NormUpdate(&g_norm, g_sv.feat);
    Features_NormApply(&g_norm, g_sv.feat_norm);

    AE_ZeroGrad(&g_ae);

    AE_Forward(&g_ae, g_sv.feat_norm);
    AE_Backward(&g_ae, g_sv.feat_norm);

    AE_Update(
        &g_ae,
        SNX_LEARNING_RATE,
        SNX_MOMENTUM,
        SNX_L2_LAMBDA,
        1);

    {
        float mse = g_ae.last_mse;

        if (g_sv.calib_count < SNX_CALIBRATION_WINDOWS)
        {
            g_sv.calib_mse[g_sv.calib_count] = mse;
        }

        g_sv.calib_mse_sum += mse;
        g_sv.calib_mse_sq_sum += mse * mse;
        g_sv.calib_count++;

        DBG_Printf(
            "[CALIB] window=%lu mse=%.6f\r\n",
            (unsigned long)g_sv.calib_count,
            (double)mse);

        if (!g_sv.flag_sd_error)
        {
            Storage_Log_Calibration(
                g_sv.calib_count,
                mse);
        }
    }

    if (g_sv.calib_count < SNX_CALIBRATION_WINDOWS)
        return;

compute_threshold:

    if (g_sv.calib_count == 0)
    {
        DBG_Printf(
            "[CALIB] ERROR: zero calibration windows!\r\n");

        transition(SNX_STATE_FALLBACK);
        return;
    }

    float sorted[SNX_CALIBRATION_WINDOWS];
    float mad_buf[SNX_CALIBRATION_WINDOWS];

    memcpy(
        sorted,
        g_sv.calib_mse,
        sizeof(float) * g_sv.calib_count);

    qsort(
        sorted,
        g_sv.calib_count,
        sizeof(float),
        float_compare);

    float median;

    if (g_sv.calib_count & 1)
    {
        median =
            sorted[g_sv.calib_count / 2];
    }
    else
    {
        median =
            0.5f *
            (sorted[g_sv.calib_count / 2 - 1] +
             sorted[g_sv.calib_count / 2]);
    }

    for (uint32_t i = 0;
         i < g_sv.calib_count;
         i++)
    {
        mad_buf[i] =
            fabsf(
                g_sv.calib_mse[i] -
                median);
    }

    qsort(
        mad_buf,
        g_sv.calib_count,
        sizeof(float),
        float_compare);

    float mad;

    if (g_sv.calib_count & 1)
    {
        mad =
            mad_buf[g_sv.calib_count / 2];
    }
    else
    {
        mad =
            0.5f *
            (mad_buf[g_sv.calib_count / 2 - 1] +
             mad_buf[g_sv.calib_count / 2]);
    }

    g_sv.anomaly_threshold =
        median + 10.0f * mad;

    DBG_Printf(
        "[CALIB] Done.\r\n"
        "        median=%.6f\r\n"
        "        MAD=%.6f\r\n"
        "        threshold=%.6f\r\n",
        (double)median,
        (double)mad,
        (double)g_sv.anomaly_threshold);

    if (!g_sv.flag_sd_error)
    {
        Storage_SaveWeights(
            &g_ae,
            &g_norm);

        Storage_SaveThreshold(
            g_sv.anomaly_threshold);
    }

    g_ae.is_trained = 1;

    {
        float centroid[SNX_FEATURE_DIM] = {0};
        memcpy(centroid, g_sv.feat, sizeof(centroid));
        memcpy(g_sv.baselines[0].centroid, centroid, sizeof(centroid));
        g_sv.baselines[0].threshold = g_sv.anomaly_threshold;
        g_sv.baselines[0].valid     = 1;
        g_sv.baseline_count         = 1;
        g_sv.active_baseline_idx    = 0;

        if (!g_sv.flag_sd_error) {
            Storage_SaveBaselines(g_sv.baselines, g_sv.baseline_count);
        }
    }

    transition(SNX_STATE_MONITOR);
}

static void handle_record(void)
{
    if ((HAL_GetTick() - g_sv.record_start_tick) > SNX_CYCLE_AUTO_TIMEOUT_MS) {
        DBG_Printf("[RECORD] Auto-timeout (%lu samples captured). → SEGMENT\r\n",
                   (unsigned long)g_sv.cycle_sample_count);
        Storage_CloseCycleRecording();
        transition(SNX_STATE_SEGMENT);
        return;
    }

    static uint32_t last_progress = 0;
    if ((HAL_GetTick() - last_progress) > 2000u) {
        DBG_Printf("[RECORD] samples=%lu (%.1fs)\r\n",
                   (unsigned long)g_sv.cycle_sample_count,
                   (double)g_sv.cycle_sample_count / (double)SNX_SAMPLE_RATE_HZ);
        last_progress = HAL_GetTick();
    }

    if (g_sv.cycle_sample_count >= SNX_CYCLE_MAX_SAMPLES) {
        DBG_Printf("[RECORD] Buffer cap reached (%lu samples). → SEGMENT\r\n",
                   (unsigned long)g_sv.cycle_sample_count);
        Storage_CloseCycleRecording();
        transition(SNX_STATE_SEGMENT);
    }
}

static void calibrate_one_plateau(uint8_t plateau_id,
                                   uint32_t start_w, uint32_t n_windows_in_plateau)
{
    uint32_t n = n_windows_in_plateau;
    if (n > SNX_CALIBRATION_WINDOWS) n = SNX_CALIBRATION_WINDOWS; /* same cap as manual path */

    static float calib_mse[SNX_CALIBRATION_WINDOWS];
    static float sorted[SNX_CALIBRATION_WINDOWS];
    static float mad_buf[SNX_CALIBRATION_WINDOWS];

    float      centroid[SNX_FEATURE_DIM] = {0};
    IMU_Sample win_buf[SNX_WINDOW_SIZE];
    float      feat[SNX_FEATURE_DIM];
    float      feat_norm[SNX_FEATURE_DIM];

    for (uint32_t i = 0; i < n; i++) {
        uint32_t w    = start_w + i;
        uint32_t base = w * SNX_WINDOW_STRIDE;

        for (uint32_t k = 0; k < SNX_WINDOW_SIZE; k++) {
            if (Storage_ReadCycleSample(base + k, &win_buf[k]) != SNX_OK) {
                if (k > 0) {
                    win_buf[k] = win_buf[k - 1];
                } else {
                    memset(&win_buf[k], 0, sizeof(IMU_Sample));
                }
            }
        }

        Features_Extract(win_buf, SNX_WINDOW_SIZE, feat);
        memcpy(feat_norm, feat, sizeof(feat));
        Features_NormUpdate(&g_norm, feat);
        Features_NormApply(&g_norm, feat_norm);

        for (uint32_t d = 0; d < SNX_FEATURE_DIM; d++) centroid[d] += feat[d];

        AE_ZeroGrad(&g_ae);
        AE_Forward(&g_ae, feat_norm);
        AE_Backward(&g_ae, feat_norm);
        AE_Update(&g_ae, SNX_LEARNING_RATE, SNX_MOMENTUM, SNX_L2_LAMBDA, 1);

        calib_mse[i] = g_ae.last_mse;

        if (!g_sv.flag_sd_error) {
            Storage_Log_Calibration(i + 1, g_ae.last_mse);
        }
    }

    for (uint32_t d = 0; d < SNX_FEATURE_DIM; d++) centroid[d] /= (float)n;

    memcpy(sorted, calib_mse, sizeof(float) * n);
    qsort(sorted, n, sizeof(float), float_compare);

    float median = (n & 1u)
        ? sorted[n / 2]
        : 0.5f * (sorted[n / 2 - 1] + sorted[n / 2]);

    for (uint32_t i = 0; i < n; i++) mad_buf[i] = fabsf(calib_mse[i] - median);
    qsort(mad_buf, n, sizeof(float), float_compare);

    float mad = (n & 1u)
        ? mad_buf[n / 2]
        : 0.5f * (mad_buf[n / 2 - 1] + mad_buf[n / 2]);

    float threshold = median + 10.0f * mad;

    memcpy(g_sv.baselines[plateau_id].centroid, centroid, sizeof(centroid));
    g_sv.baselines[plateau_id].threshold = threshold;
    g_sv.baselines[plateau_id].valid     = 1;

    DBG_Printf("[SEGMENT] Plateau %u (n=%lu): median=%.6f MAD=%.6f threshold=%.6f\r\n",
               (unsigned)plateau_id, (unsigned long)n,
               (double)median, (double)mad, (double)threshold);
}

typedef struct { uint32_t start; uint32_t count; } Plateau;

static void handle_segment(void)
{
    uint32_t n_samples = Storage_GetCycleSampleCount();
    if (n_samples < SNX_WINDOW_SIZE) {
        DBG_Printf("[SEGMENT] ERROR: only %lu samples recorded (need >= %u). "
                   "Re-run commissioning with 'G'.\r\n",
                   (unsigned long)n_samples, (unsigned)SNX_WINDOW_SIZE);
        transition(SNX_STATE_FALLBACK);
        return;
    }

    uint32_t n_windows = (n_samples - SNX_WINDOW_SIZE) / SNX_WINDOW_STRIDE + 1u;
    DBG_Printf("[SEGMENT] Replaying %lu samples -> %lu feature windows\r\n",
               (unsigned long)n_samples, (unsigned long)n_windows);

    static float energy[SNX_CYCLE_MAX_SAMPLES / SNX_WINDOW_STRIDE + 1u];
    IMU_Sample win_buf[SNX_WINDOW_SIZE];

    if (n_windows > (SNX_CYCLE_MAX_SAMPLES / SNX_WINDOW_STRIDE + 1u)) {
        n_windows = SNX_CYCLE_MAX_SAMPLES / SNX_WINDOW_STRIDE + 1u;
    }

    for (uint32_t w = 0; w < n_windows; w++) {
        uint32_t base = w * SNX_WINDOW_STRIDE;
        for (uint32_t k = 0; k < SNX_WINDOW_SIZE; k++) {
            if (Storage_ReadCycleSample(base + k, &win_buf[k]) != SNX_OK) {
                if (k > 0) {
                    win_buf[k] = win_buf[k - 1];
                } else {
                    memset(&win_buf[k], 0, sizeof(IMU_Sample));
                }
            }
        }
        float feat[SNX_FEATURE_DIM];
        Features_Extract(win_buf, SNX_WINDOW_SIZE, feat);
        energy[w] = feat[SNX_ENERGY_FEATURE_IDX];
    }

    static uint8_t is_stable[SNX_CYCLE_MAX_SAMPLES / SNX_WINDOW_STRIDE + 1u];
    uint32_t half = SNX_STABILITY_HOLD_WINDOWS / 2u;

    for (uint32_t w = 0; w < n_windows; w++) {
        uint32_t lo = (w >= half) ? (w - half) : 0u;
        uint32_t hi = (w + half < n_windows) ? (w + half) : (n_windows - 1u);

        float emin = energy[lo], emax = energy[lo];
        for (uint32_t k = lo; k <= hi; k++) {
            if (energy[k] < emin) emin = energy[k];
            if (energy[k] > emax) emax = energy[k];
        }
        is_stable[w] = ((emax - emin) < SNX_STABILITY_EPS) ? 1u : 0u;
    }

    Plateau plateaus[SNX_MAX_STATES_CEILING];
    uint8_t n_plateaus = 0;
    uint32_t run_start = 0;
    uint8_t  in_run = 0;

    for (uint32_t w = 0; w < n_windows && n_plateaus < g_sv.max_states; w++) {
        if (is_stable[w] && !in_run) {
            in_run = 1;
            run_start = w;
        } else if (!is_stable[w] && in_run) {
            in_run = 0;
            if ((w - run_start) >= SNX_STABILITY_HOLD_WINDOWS) {
                plateaus[n_plateaus].start = run_start;
                plateaus[n_plateaus].count = w - run_start;
                n_plateaus++;
            }
        }
    }
    if (in_run && n_plateaus < g_sv.max_states &&
        (n_windows - run_start) >= SNX_STABILITY_HOLD_WINDOWS) {
        plateaus[n_plateaus].start = run_start;
        plateaus[n_plateaus].count = n_windows - run_start;
        n_plateaus++;
    }

    DBG_Printf("[SEGMENT] Found %u plateau(s) (max_states=%u, eps=%.4f, hold=%u)\r\n",
               (unsigned)n_plateaus, (unsigned)g_sv.max_states,
               (double)SNX_STABILITY_EPS, (unsigned)SNX_STABILITY_HOLD_WINDOWS);

    if (n_plateaus == 0) {
        DBG_Printf("[SEGMENT] ERROR: no stable plateau found. Increase "
                   "SNX_STABILITY_EPS, lower SNX_STABILITY_HOLD_WINDOWS, or "
                   "re-record with longer steady periods.\r\n");
        transition(SNX_STATE_FALLBACK);
        return;
    }

    AE_Init(&g_ae);
    Features_NormInit(&g_norm);

    for (uint8_t p = 0; p < n_plateaus; p++) {
        DBG_Printf("[SEGMENT] Calibrating plateau %u/%u (start_window=%lu, n=%lu)\r\n",
                   (unsigned)(p + 1), (unsigned)n_plateaus,
                   (unsigned long)plateaus[p].start, (unsigned long)plateaus[p].count);
        calibrate_one_plateau(p, plateaus[p].start, plateaus[p].count);
    }

    g_sv.baseline_count      = n_plateaus;
    g_sv.active_baseline_idx = 0;
    g_ae.is_trained = 1;

    if (!g_sv.flag_sd_error) {
        Storage_SaveWeights(&g_ae, &g_norm);
        Storage_SaveBaselines(g_sv.baselines, g_sv.baseline_count);
    }

    DBG_Printf("[SEGMENT] Commissioning complete: %u baseline(s) ready. -> MONITOR\r\n",
               (unsigned)g_sv.baseline_count);
    transition(SNX_STATE_MONITOR);
}

static uint8_t find_nearest_baseline(const float *feat)
{
    uint8_t best = g_sv.active_baseline_idx;
    float   best_dist = 3.4e38f;

    for (uint8_t i = 0; i < g_sv.baseline_count; i++) {
        if (!g_sv.baselines[i].valid) continue;
        float dist = 0.0f;
        for (uint32_t d = 0; d < SNX_FEATURE_DIM; d++) {
            float diff = feat[d] - g_sv.baselines[i].centroid[d];
            dist += diff * diff;
        }
        if (dist < best_dist) { best_dist = dist; best = i; }
    }
    return best;
}

static void handle_monitor(void)
{
    if (!g_sv.window_ready) return;
    g_sv.window_ready = 0;

    Features_Extract(g_sv.window_buf, SNX_WINDOW_SIZE, g_sv.feat);

    g_sv.active_baseline_idx = find_nearest_baseline(g_sv.feat);

    memcpy(g_sv.feat_norm, g_sv.feat, sizeof(g_sv.feat));
    Features_NormApply(&g_norm, g_sv.feat_norm);

    uint32_t dwt_start = DWT->CYCCNT;
    float mse = AE_Forward(&g_ae, g_sv.feat_norm);
    uint32_t dwt_end   = DWT->CYCCNT;

    uint32_t elapsed_cycles = dwt_end - dwt_start;
    float    time_us        = (float)elapsed_cycles / (float)SNX_CPU_MHZ;

    g_sv.total_windows++;
    g_sv.monitor_window_id++;

    float active_threshold = g_sv.baselines[g_sv.active_baseline_idx].threshold;
    uint8_t is_anomaly = (mse > active_threshold * 1.10f) ? 1u : 0u;

    if (!g_sv.flag_sd_error) {
        Storage_Log_Eval(g_sv.monitor_window_id,
                         HAL_GetTick(),
                         mse,
                         (uint8_t)g_sv.current_label,
                         is_anomaly);
    }

    if (!g_sv.flag_sd_error) {
        Storage_Log_Latent(HAL_GetTick(),
                           g_ae.act.z,
                           (uint8_t)g_sv.current_label);
    }

    if (!g_sv.flag_sd_error) {
        Storage_Log_Profile(g_sv.monitor_window_id, elapsed_cycles, time_us);
    }

    if (is_anomaly) {
        g_sv.anomaly_streak++;
        g_sv.total_anomalies++;

        DBG_Printf("[MONITOR] ANOMALY! state=%u mse=%.6f thr=%.6f streak=%lu label=%s\r\n",
                   (unsigned)g_sv.active_baseline_idx,
                   (double)mse, (double)active_threshold,
                   (unsigned long)g_sv.anomaly_streak,
                   LABEL_NAMES[g_sv.current_label]);

        if (!g_sv.flag_sd_error) {
            AnomalyEvent ev = {
                .timestamp_ms = HAL_GetTick(),
                .mse_score    = mse,
                .threshold    = active_threshold,
            };
            memcpy(ev.features, g_sv.feat, sizeof(g_sv.feat));
            Storage_Log_Anomaly(&ev);
        }

        if (g_sv.anomaly_streak >= SNX_ANOMALY_STREAK_LIMIT) {
            DBG_Printf("[ALERT] Sustained anomaly! state=%u streak=%lu total=%lu\r\n",
                       (unsigned)g_sv.active_baseline_idx,
                       (unsigned long)g_sv.anomaly_streak,
                       (unsigned long)g_sv.total_anomalies);
        }
    } else {
        g_sv.anomaly_streak = 0;

        DBG_Printf("[MONITOR] normal state=%u mse=%.6f cyc=%lu us=%.1f\r\n",
                   (unsigned)g_sv.active_baseline_idx,
                   (double)mse, (unsigned long)elapsed_cycles, (double)time_us);

        if (g_sv.train_buf_idx < SNX_TRAIN_BATCH_SIZE) {
            memcpy(g_sv.train_buf[g_sv.train_buf_idx],
                   g_sv.feat_norm, sizeof(g_sv.feat_norm));
            g_sv.train_buf_idx++;
        }

        if (g_sv.train_buf_idx >= SNX_TRAIN_BATCH_SIZE) {
            DBG_Printf("[MONITOR] Training batch ready. → TRAIN\r\n");
            transition(SNX_STATE_TRAIN);
            return;
        }
    }
}

static void handle_train(void)
{
    g_sv.train_session_id++;

    DBG_Printf("[TRAIN] Session %lu: %lu windows × %u epochs\r\n",
               (unsigned long)g_sv.train_session_id,
               (unsigned long)g_sv.train_buf_idx,
               (unsigned)SNX_MAX_EPOCHS);

    uint32_t dwt_start = DWT->CYCCNT;

    for (uint32_t epoch = 0; epoch < SNX_MAX_EPOCHS; epoch++) {
        float epoch_loss = 0.0f;
        AE_ZeroGrad(&g_ae);

        for (uint32_t s = 0; s < g_sv.train_buf_idx; s++) {
            AE_Forward(&g_ae, g_sv.train_buf[s]);
            AE_Backward(&g_ae, g_sv.train_buf[s]);
            epoch_loss += g_ae.last_mse;
        }

        AE_Update(&g_ae, SNX_LEARNING_RATE, SNX_MOMENTUM,
                   SNX_L2_LAMBDA, g_sv.train_buf_idx);

        float mean_loss = epoch_loss / (float)g_sv.train_buf_idx;

        DBG_Printf("[TRAIN] epoch=%lu loss=%.6f\r\n",
                   (unsigned long)epoch, (double)mean_loss);

        if (!g_sv.flag_sd_error) {
            Storage_Log_Train(g_sv.train_session_id, epoch, mean_loss);
        }
    }

    uint32_t cycles = DWT->CYCCNT - dwt_start;
    DBG_Printf("[TRAIN] Done. Cycles=%lu (~%lu ms) steps=%lu\r\n",
               (unsigned long)cycles,
               (unsigned long)(cycles / 168000UL),
               (unsigned long)g_ae.train_steps);

    g_sv.train_buf_idx = 0;

    if (!g_sv.flag_sd_error) {
        Storage_SaveWeights(&g_ae, &g_norm);
        DBG_Printf("[TRAIN] Weights saved to SD.\r\n");
    }

    transition(SNX_STATE_MONITOR);
}

static void handle_fallback(void)
{
    static uint32_t last_alert = 0;
    if ((HAL_GetTick() - last_alert) > 5000u) {
        DBG_Printf("[FALLBACK] System fault. SD=%d IMU=%d. Send 'R' to reset.\r\n",
                   g_sv.flag_sd_error, g_sv.flag_imu_error);
        last_alert = HAL_GetTick();
    }
}

void Supervisor_Tick(void)
{
    if (!g_sv.flag_sd_error &&
        (HAL_GetTick() - g_sv.last_sd_sync_tick) >= SNX_SD_SYNC_INTERVAL_MS) {
        Storage_Sync();
        g_sv.last_sd_sync_tick = HAL_GetTick();
    }

    if ((HAL_GetTick() - g_sv.last_print_tick) >= SNX_DEBUG_PRINT_INTERVAL_MS) {
        DBG_Printf("[STATS] state=%s windows=%lu anomalies=%lu baseline=%u/%u label=%s\r\n",
                   STATE_NAMES[g_sv.state],
                   (unsigned long)g_sv.total_windows,
                   (unsigned long)g_sv.total_anomalies,
                   (unsigned)g_sv.active_baseline_idx,
                   (unsigned)g_sv.baseline_count,
                   LABEL_NAMES[g_sv.current_label]);
        g_sv.last_print_tick = HAL_GetTick();
    }

    switch (g_sv.state) {
        case SNX_STATE_IDLE:                                      break;
        case SNX_STATE_COLLECT:    handle_collect();              break;
        case SNX_STATE_CALIBRATE:  handle_calibrate();            break;
        case SNX_STATE_MONITOR:    handle_monitor();              break;
        case SNX_STATE_TRAIN:      handle_train();                break;
        case SNX_STATE_FALLBACK:   handle_fallback();             break;
        case SNX_STATE_RECORD:     handle_record();               break; /* v3.2 */
        case SNX_STATE_SEGMENT:    handle_segment();              break; /* v3.2 */
        default:                   transition(SNX_STATE_FALLBACK); break;
    }
}

static void handle_pending_numeric_byte(uint8_t c)
{
    if (c == '\r' || c == '\n') {
        if (g_sv.numeric_linelen == 0) {
            return; /* bare \r\n — ignore, keep waiting */
        }
        g_sv.numeric_linebuf[g_sv.numeric_linelen] = '\0';

        if (g_sv.pending_numeric_cmd == SNX_CMD_SET_MAX_STATES) {
            long val = strtol(g_sv.numeric_linebuf, NULL, 10);
            if (val >= 1 && val <= (long)SNX_MAX_STATES_CEILING) {
                Supervisor_SetMaxStates((uint8_t)val);
            } else {
                DBG_Printf("[CMD] Invalid max_states '%s' (must be 1-%u)\r\n",
                           g_sv.numeric_linebuf, (unsigned)SNX_MAX_STATES_CEILING);
            }
        } else if (g_sv.pending_numeric_cmd == SNX_CMD_SET_THRESHOLD) {
            float val = strtof(g_sv.numeric_linebuf, NULL);
            if (val > 0.0f) {
                Supervisor_SetThreshold(val);
            } else {
                DBG_Printf("[CMD] Invalid threshold '%s'\r\n", g_sv.numeric_linebuf);
            }
        }

        g_sv.pending_numeric_cmd = 0;
        g_sv.numeric_linelen     = 0;
        return;
    }

    if ((c >= '0' && c <= '9') || c == '.' || c == '-') {
        if (g_sv.numeric_linelen < sizeof(g_sv.numeric_linebuf) - 1u) {
            g_sv.numeric_linebuf[g_sv.numeric_linelen++] = (char)c;
        }
    }
}

void Supervisor_HandleCommand(uint8_t cmd)
{
    if (g_sv.pending_numeric_cmd != 0) {
        handle_pending_numeric_byte(cmd);
        return;
    }

    DBG_Printf("[CMD] Received '%c'\r\n", cmd);

    switch (cmd) {
        case SNX_CMD_LABEL_NORMAL:
            g_sv.current_label = SNX_LABEL_NORMAL;
            DBG_Printf("[LABEL] → NORMAL\r\n");
            return;
        case SNX_CMD_LABEL_IMPACT:
            g_sv.current_label = SNX_LABEL_IMPACT;
            DBG_Printf("[LABEL] → IMPACT\r\n");
            return;
        case SNX_CMD_LABEL_IMBALANCE:
            g_sv.current_label = SNX_LABEL_IMBALANCE;
            DBG_Printf("[LABEL] → IMBALANCE\r\n");
            return;
        case SNX_CMD_LABEL_LOOSE_MOUNT:
            g_sv.current_label = SNX_LABEL_LOOSE_MOUNT;
            DBG_Printf("[LABEL] → LOOSE_MOUNT\r\n");
            return;
        case SNX_CMD_LABEL_MISALIGNMENT:
            g_sv.current_label = SNX_LABEL_MISALIGNMENT;
            DBG_Printf("[LABEL] → MISALIGNMENT\r\n");
            return;
        case SNX_CMD_LABEL_BEARING_FAULT:
            g_sv.current_label = SNX_LABEL_BEARING_FAULT;
            DBG_Printf("[LABEL] → BEARING_FAULT\r\n");
            return;
        default:
            break;
    }

    switch (cmd) {
        case SNX_CMD_CALIBRATE:
            DBG_Printf("[CMD] Starting single-baseline calibration "
                       "(legacy/bench path — 'G' is recommended for "
                       "deployments with multiple operating states)...\r\n");
            AE_Init(&g_ae);
            Features_NormInit(&g_norm);
            g_sv.total_windows   = 0;
            g_sv.total_anomalies = 0;
            g_sv.anomaly_streak  = 0;
            g_sv.train_buf_idx   = 0;
            g_sv.monitor_window_id = 0;
            g_sv.current_label   = SNX_LABEL_NORMAL;
            reset_calibration();
            transition(SNX_STATE_COLLECT);
            break;

        case SNX_CMD_TRAIN:
            if (g_sv.train_buf_idx > 0) {
                transition(SNX_STATE_TRAIN);
            } else {
                DBG_Printf("[CMD] Train batch empty.\r\n");
            }
            break;

        case SNX_CMD_STATUS:
            Supervisor_PrintStatus();
            break;

        case SNX_CMD_RESET_WEIGHTS:
            AE_Init(&g_ae);
            DBG_Printf("[CMD] Weights re-initialised (Xavier).\r\n");
            break;

        case SNX_CMD_SAVE_WEIGHTS:
            if (!g_sv.flag_sd_error) {
                Storage_SaveWeights(&g_ae, &g_norm);
                Storage_SaveThreshold(g_sv.anomaly_threshold);
                Storage_SaveBaselines(g_sv.baselines, g_sv.baseline_count);
                DBG_Printf("[CMD] Weights + threshold + baselines saved.\r\n");
            } else {
                DBG_Printf("[CMD] SD error, cannot save.\r\n");
            }
            break;

        case SNX_CMD_LOAD_WEIGHTS:
            if (!g_sv.flag_sd_error) {
                if (Storage_LoadWeights(&g_ae, &g_norm) == SNX_OK) {
                    float thr = 0.f;
                    if (Storage_LoadThreshold(&thr) == SNX_OK) {
                        g_sv.anomaly_threshold = thr;
                    }
                    uint8_t loaded_count = 0;
                    if (Storage_LoadBaselines(g_sv.baselines, &loaded_count) == SNX_OK) {
                        g_sv.baseline_count = loaded_count;
                    }
                    DBG_Printf("[CMD] Weights loaded. legacy_threshold=%.6f "
                               "baselines=%u\r\n",
                               (double)g_sv.anomaly_threshold,
                               (unsigned)g_sv.baseline_count);
                }
            }
            break;

        case SNX_CMD_SET_THRESHOLD:
            DBG_Printf("[CMD] Send threshold as float string then Enter. "
                       "Applies to active baseline (%u/%u).\r\n",
                       (unsigned)g_sv.active_baseline_idx,
                       (unsigned)g_sv.baseline_count);
            g_sv.pending_numeric_cmd = SNX_CMD_SET_THRESHOLD;
            g_sv.numeric_linelen     = 0;
            break;

        case SNX_CMD_SET_MAX_STATES:
            DBG_Printf("[CMD] Send max_states as integer (1-%u) then Enter.\r\n",
                       (unsigned)SNX_MAX_STATES_CEILING);
            g_sv.pending_numeric_cmd = SNX_CMD_SET_MAX_STATES;
            g_sv.numeric_linelen     = 0;
            break;

        case SNX_CMD_START_RECORD:
            DBG_Printf("[CMD] Commissioning recording started. Run the "
                       "machine through one full operating cycle, then "
                       "send 'D' when done.\r\n");
            if (Storage_OpenCycleRecording() == SNX_OK) {
                g_sv.cycle_sample_count = 0;
                g_sv.record_start_tick  = HAL_GetTick();
                transition(SNX_STATE_RECORD);
            } else {
                DBG_Printf("[CMD] Could not open %s — SD error?\r\n",
                           SNX_SD_CYCLE_FILENAME);
            }
            break;

        case SNX_CMD_DONE_RECORD:
            if (g_sv.state == SNX_STATE_RECORD) {
                DBG_Printf("[CMD] Recording stopped (%lu samples). -> SEGMENT\r\n",
                           (unsigned long)g_sv.cycle_sample_count);
                Storage_CloseCycleRecording();
                transition(SNX_STATE_SEGMENT);
            } else {
                DBG_Printf("[CMD] Not currently recording.\r\n");
            }
            break;

        case SNX_CMD_TOGGLE_RAW_LOG:
            g_sv.raw_log_enabled ^= 1u;
            DBG_Printf("[CMD] Raw IMU log: %s\r\n",
                       g_sv.raw_log_enabled ? "ON" : "OFF");
            break;

        default:
            DBG_Printf("[CMD] Unknown command 0x%02X\r\n", cmd);
            break;
    }
}

void Supervisor_ForceState(SNX_State new_state)
{
    transition(new_state);
}

void Supervisor_SetThreshold(float threshold)
{
    if (g_sv.baseline_count > 0 &&
        g_sv.active_baseline_idx < g_sv.baseline_count) {
        g_sv.baselines[g_sv.active_baseline_idx].threshold = threshold;
        DBG_Printf("[CMD] Baseline %u threshold manually set to %.6f\r\n",
                   (unsigned)g_sv.active_baseline_idx, (double)threshold);
        if (!g_sv.flag_sd_error) {
            Storage_SaveBaselines(g_sv.baselines, g_sv.baseline_count);
        }
    } else {
        g_sv.anomaly_threshold = threshold;
        DBG_Printf("[CMD] Legacy threshold manually set to %.6f "
                   "(no baselines exist yet)\r\n", (double)threshold);
        if (!g_sv.flag_sd_error) {
            Storage_SaveThreshold(threshold);
        }
    }
}
void Supervisor_SetMaxStates(uint8_t n)
{
    if (n < 1) n = 1;
    if (n > SNX_MAX_STATES_CEILING) n = SNX_MAX_STATES_CEILING;
    g_sv.max_states = n;
    DBG_Printf("[CMD] max_states set to %u\r\n", (unsigned)n);
    if (!g_sv.flag_sd_error) {
        Storage_SaveMaxStates(n);
    }
}

void Supervisor_PrintStatus(void)
{
    DBG_Printf("=== SensiNerveX v%d.%d Status (Node 0x%02X) ===\r\n",
               SNX_FW_VERSION_MAJOR, SNX_FW_VERSION_MINOR, SNX_NODE_ID);
    DBG_Printf("  State           : %s\r\n",  STATE_NAMES[g_sv.state]);
    DBG_Printf("  Current label   : %s (%u)\r\n",
               LABEL_NAMES[g_sv.current_label], (unsigned)g_sv.current_label);
    DBG_Printf("  Total windows   : %lu\r\n", (unsigned long)g_sv.total_windows);
    DBG_Printf("  Monitor win id  : %lu\r\n", (unsigned long)g_sv.monitor_window_id);
    DBG_Printf("  Total anomaly   : %lu\r\n", (unsigned long)g_sv.total_anomalies);
    DBG_Printf("  Streak          : %lu\r\n", (unsigned long)g_sv.anomaly_streak);
    DBG_Printf("  Baseline active : %u / %u\r\n",
               (unsigned)g_sv.active_baseline_idx, (unsigned)g_sv.baseline_count);
    if (g_sv.baseline_count > 0) {
        DBG_Printf("  Active threshold: %.6f\r\n",
                   (double)g_sv.baselines[g_sv.active_baseline_idx].threshold);
    }
    DBG_Printf("  Max states      : %u (ceiling %u)\r\n",
               (unsigned)g_sv.max_states, (unsigned)SNX_MAX_STATES_CEILING);
    DBG_Printf("  Legacy threshold: %.6f\r\n", (double)g_sv.anomaly_threshold);
    DBG_Printf("  Last MSE        : %.6f\r\n", (double)g_ae.last_mse);
    DBG_Printf("  Train steps     : %lu\r\n", (unsigned long)g_ae.train_steps);
    DBG_Printf("  Train session   : %lu\r\n", (unsigned long)g_sv.train_session_id);
    DBG_Printf("  Norm init       : %s\r\n",  g_norm.is_initialised ? "YES" : "NO");
    DBG_Printf("  SD error        : %s\r\n",  g_sv.flag_sd_error ? "YES" : "NO");
    DBG_Printf("  IMU error       : %s\r\n",  g_sv.flag_imu_error ? "YES" : "NO");
    DBG_Printf("  Raw log         : %s\r\n",  g_sv.raw_log_enabled ? "ON" : "OFF");
    DBG_Printf("  Calib samples   : %lu\r\n", (unsigned long)g_sv.calib_count);
    DBG_Printf("  Cycle samples   : %lu\r\n", (unsigned long)g_sv.cycle_sample_count);
    DBG_Printf("  Train buf       : %lu / %u\r\n",
               (unsigned long)g_sv.train_buf_idx, (unsigned)SNX_TRAIN_BATCH_SIZE);
    DBG_Printf("  Latent space    : z=[%.4f, %.4f, %.4f, %.4f]\r\n",
               (double)g_ae.act.z[0], (double)g_ae.act.z[1],
               (double)g_ae.act.z[2], (double)g_ae.act.z[3]);
    DBG_Printf("=========================================\r\n");
}