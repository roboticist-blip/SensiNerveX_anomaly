/**
 * @file    supervisor.c
 * @brief   SensiNerveX v3.1 — Anomaly Detection Supervisor FSM
 *
 * All heavy data structures live in CCM SRAM (64 KB on STM32F405).
 * CCM is connected directly to the CPU data bus — no DMA, fastest access.
 *
 * Changes versus v3.0 (research logging additions only):
 *
 *   Objective 1 — SNX_FaultLabel + UART commands '0'..'5'
 *     g_sv.current_label updated immediately on receipt of '0'..'5'.
 *
 *   Objective 2 — EVAL.csv
 *     handle_monitor() calls Storage_Log_Eval() every window.
 *
 *   Objective 3 — LATENT.csv
 *     handle_monitor() calls Storage_Log_Latent() using g_ae.act.z directly
 *     after AE_Forward() — no recomputation.
 *
 *   Objective 4 — TRAIN.csv
 *     handle_train() logs per-epoch loss via Storage_Log_Train().
 *     g_sv.train_session_id incremented at entry to handle_train().
 *
 *   Objective 5 — CALIB.csv
 *     handle_calibrate() calls Storage_Log_Calibration() per window.
 *
 *   Objective 6 — PROFILE.csv
 *     handle_monitor() wraps AE_Forward() with DWT reads; calls
 *     Storage_Log_Profile() with cycles and time_us.
 *
 *   Objective 7 — Resource summary
 *     Supervisor_Init() prints AE parameter count, model RAM size,
 *     weight storage size, supervisor RAM size, train steps after boot.
 *
 * The core anomaly detection algorithm, autoencoder, and feature extraction
 * pipeline are NOT modified.
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
    "IDLE", "COLLECT", "CALIBRATE", "MONITOR", "TRAIN", "FALLBACK"
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
 *   sizeof(SNX_Supervisor) window_buf + feat + train_buf + flags + */
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
    DBG_Printf("==========================================\r\n\r\n");
}

void Supervisor_Init(void)
{
    memset(&g_sv, 0, sizeof(SNX_Supervisor));
    g_sv.state              = SNX_STATE_IDLE;
    g_sv.raw_log_enabled    = 1;
    g_sv.current_label      = SNX_LABEL_NORMAL;  /* Objective 1 */
    g_sv.train_session_id   = 0;                 /* Objective 4 */
    g_sv.monitor_window_id  = 0;                 /* Objectives 2,6 */

    AE_Init(&g_ae);
    Features_NormInit(&g_norm);

    if (Storage_Init() == SNX_OK) {
        DBG_Printf("[INIT] SD card OK\r\n");
        SNX_Status ws = Storage_LoadWeights(&g_ae, &g_norm);
        if (ws == SNX_OK) {
            DBG_Printf("[INIT] Weights loaded from SD. Steps=%lu\r\n",
                       (unsigned long)g_ae.train_steps);
            if (g_ae.is_trained && g_norm.is_initialised) {
                float saved_thresh = 0.0f;
                if (Storage_LoadThreshold(&saved_thresh) == SNX_OK) {
                    g_sv.anomaly_threshold = saved_thresh;
                    DBG_Printf("[INIT] Loaded threshold: %.6f\r\n",
                               (double)saved_thresh);
                    /* Objective 7 — print after weights are loaded */
                    print_resource_summary();
                    g_sv.state = SNX_STATE_COLLECT;
                    return;
                }
            }
        } else {
            DBG_Printf("[INIT] No saved weights, starting fresh.\r\n");
        }
    } else {
        DBG_Printf("[INIT] SD card FAIL — logging disabled\r\n");
        g_sv.flag_sd_error = 1;
    }

    /* Objective 7 — print even on fresh boot */
    print_resource_summary();

    DBG_Printf("[INIT] SensiNerveX v%d.%d Node 0x%02X ready.\r\n",
               SNX_FW_VERSION_MAJOR, SNX_FW_VERSION_MINOR, SNX_NODE_ID);
    DBG_Printf("[INIT] Send 'C' to begin calibration.\r\n");
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

    /* ------------------------------------------
       Robust threshold calculation using
       Median + 6*MAD
       ------------------------------------------ */

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

    transition(SNX_STATE_MONITOR);
}

static void handle_monitor(void)
{
    if (!g_sv.window_ready) return;
    g_sv.window_ready = 0;

    Features_Extract(g_sv.window_buf, SNX_WINDOW_SIZE, g_sv.feat);
    memcpy(g_sv.feat_norm, g_sv.feat, sizeof(g_sv.feat));
    Features_NormApply(&g_norm, g_sv.feat_norm);

    uint32_t dwt_start = DWT->CYCCNT;
    float mse = AE_Forward(&g_ae, g_sv.feat_norm);
    uint32_t dwt_end   = DWT->CYCCNT;

    uint32_t elapsed_cycles = dwt_end - dwt_start;
    float    time_us        = (float)elapsed_cycles / (float)SNX_CPU_MHZ;

    g_sv.total_windows++;
    g_sv.monitor_window_id++;

    uint8_t is_anomaly = (mse > g_sv.anomaly_threshold*1.10f) ? 1u : 0u;

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

        DBG_Printf("[MONITOR] ANOMALY! mse=%.6f thr=%.6f streak=%lu label=%s\r\n",
                   (double)mse, (double)g_sv.anomaly_threshold,
                   (unsigned long)g_sv.anomaly_streak,
                   LABEL_NAMES[g_sv.current_label]);

        if (!g_sv.flag_sd_error) {
            AnomalyEvent ev = {
                .timestamp_ms = HAL_GetTick(),
                .mse_score    = mse,
                .threshold    = g_sv.anomaly_threshold,
            };
            memcpy(ev.features, g_sv.feat, sizeof(g_sv.feat));
            Storage_Log_Anomaly(&ev);
        }

        if (g_sv.anomaly_streak >= SNX_ANOMALY_STREAK_LIMIT) {
            DBG_Printf("[ALERT] Sustained anomaly! Streak=%lu total=%lu\r\n",
                       (unsigned long)g_sv.anomaly_streak,
                       (unsigned long)g_sv.total_anomalies);
        }
    } else {
        g_sv.anomaly_streak = 0;

        DBG_Printf("[MONITOR] normal mse=%.6f cyc=%lu us=%.1f\r\n",
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
        DBG_Printf("[STATS] state=%s windows=%lu anomalies=%lu thr=%.6f label=%s\r\n",
                   STATE_NAMES[g_sv.state],
                   (unsigned long)g_sv.total_windows,
                   (unsigned long)g_sv.total_anomalies,
                   (double)g_sv.anomaly_threshold,
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
        default:                   transition(SNX_STATE_FALLBACK); break;
    }
}

void Supervisor_HandleCommand(uint8_t cmd)
{
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
            DBG_Printf("[CMD] Starting calibration...\r\n");
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
                DBG_Printf("[CMD] Weights + threshold saved.\r\n");
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
                    DBG_Printf("[CMD] Weights loaded. threshold=%.6f\r\n",
                               (double)g_sv.anomaly_threshold);
                }
            }
            break;

        case SNX_CMD_SET_THRESHOLD:
            DBG_Printf("[CMD] Send threshold as float string then Enter.\r\n");
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
    g_sv.anomaly_threshold = threshold;
    DBG_Printf("[CMD] Threshold manually set to %.6f\r\n", (double)threshold);
    if (!g_sv.flag_sd_error) {
        Storage_SaveThreshold(threshold);
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
    DBG_Printf("  Threshold       : %.6f\r\n", (double)g_sv.anomaly_threshold);
    DBG_Printf("  Last MSE        : %.6f\r\n", (double)g_ae.last_mse);
    DBG_Printf("  Train steps     : %lu\r\n", (unsigned long)g_ae.train_steps);
    DBG_Printf("  Train session   : %lu\r\n", (unsigned long)g_sv.train_session_id);
    DBG_Printf("  Norm init       : %s\r\n",  g_norm.is_initialised ? "YES" : "NO");
    DBG_Printf("  SD error        : %s\r\n",  g_sv.flag_sd_error ? "YES" : "NO");
    DBG_Printf("  IMU error       : %s\r\n",  g_sv.flag_imu_error ? "YES" : "NO");
    DBG_Printf("  Raw log         : %s\r\n",  g_sv.raw_log_enabled ? "ON" : "OFF");
    DBG_Printf("  Calib samples   : %lu\r\n", (unsigned long)g_sv.calib_count);
    DBG_Printf("  Train buf       : %lu / %u\r\n",
               (unsigned long)g_sv.train_buf_idx, (unsigned)SNX_TRAIN_BATCH_SIZE);
    DBG_Printf("  Latent space    : z=[%.4f, %.4f, %.4f, %.4f]\r\n",
               (double)g_ae.act.z[0], (double)g_ae.act.z[1],
               (double)g_ae.act.z[2], (double)g_ae.act.z[3]);
    DBG_Printf("=========================================\r\n");
}
