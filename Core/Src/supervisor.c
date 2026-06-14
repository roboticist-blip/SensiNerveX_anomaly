/**
 * @file    supervisor.c
 * @brief   SensiNerveX v3.0 — Anomaly Detection Supervisor FSM
 *
 * All heavy data structures live in CCM SRAM (64 KB on STM32F405).
 * CCM is connected directly to the CPU data bus — no DMA, fastest access.
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

__attribute__((section(".ccmram"))) AE_Model      g_ae;
__attribute__((section(".ccmram"))) NormStats     g_norm;
__attribute__((section(".ccmram"))) SNX_Supervisor g_sv;

static const char *const STATE_NAMES[SNX_STATE_COUNT] = {
    "IDLE", "COLLECT", "CALIBRATE", "MONITOR", "TRAIN", "FALLBACK"
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

void Supervisor_Init(void)
{
    memset(&g_sv, 0, sizeof(SNX_Supervisor));
    g_sv.state          = SNX_STATE_IDLE;
    g_sv.raw_log_enabled = 1;   

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

    g_sv.state = SNX_STATE_COLLECT;
    DBG_Printf("[INIT] SensiNerveX v%d.%d Node 0x%02X ready.\r\n",
               SNX_FW_VERSION_MAJOR, SNX_FW_VERSION_MINOR, SNX_NODE_ID);
    DBG_Printf("[INIT] Send 'C' to begin calibration.\r\n");
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

/*
 * During calibration we run forward passes on NORMAL data to establish the
 * baseline MSE distribution, then set:
 *   threshold = mean_mse + SNX_THRESHOLD_SIGMA_MULT * std_mse
 *
 * We also do light training here so the autoencoder actually learns normal patterns.
 */
static void handle_calibrate(void)
{
    if (!g_sv.window_ready) return;
    g_sv.window_ready = 0;

    if ((HAL_GetTick() - g_sv.calib_start_tick) > SNX_CALIBRATION_TIMEOUT_MS) {
        DBG_Printf("[CALIB] Timeout — using collected stats.\r\n");
        goto compute_threshold;
    }

    Features_Extract(g_sv.window_buf, SNX_WINDOW_SIZE, g_sv.feat);
    memcpy(g_sv.feat_norm, g_sv.feat, sizeof(g_sv.feat));
    Features_NormUpdate(&g_norm, g_sv.feat);
    Features_NormApply(&g_norm, g_sv.feat_norm);

    AE_ZeroGrad(&g_ae);
    AE_Forward(&g_ae, g_sv.feat_norm);
    AE_Backward(&g_ae, g_sv.feat_norm);
    AE_Update(&g_ae, SNX_LEARNING_RATE, SNX_MOMENTUM,
               SNX_L2_LAMBDA, 1);

    float mse = g_ae.last_mse;
    g_sv.calib_mse_sum    += mse;
    g_sv.calib_mse_sq_sum += mse * mse;
    g_sv.calib_count++;

    DBG_Printf("[CALIB] window=%lu mse=%.6f\r\n",
               (unsigned long)g_sv.calib_count, (double)mse);

    if (g_sv.calib_count < SNX_CALIBRATION_WINDOWS) return;

compute_threshold:
    if (g_sv.calib_count == 0) {
        DBG_Printf("[CALIB] ERROR: zero calibration windows!\r\n");
        transition(SNX_STATE_FALLBACK);
        return;
    }

    float mean_mse = g_sv.calib_mse_sum / (float)g_sv.calib_count;
    float var_mse  = (g_sv.calib_mse_sq_sum / (float)g_sv.calib_count)
                     - (mean_mse * mean_mse);
    if (var_mse < 0.f) var_mse = 0.f;
    float std_mse  = sqrtf(var_mse);

    g_sv.anomaly_threshold = mean_mse + SNX_THRESHOLD_SIGMA_MULT * std_mse;

    DBG_Printf("[CALIB] Done. MSE mean=%.6f std=%.6f threshold=%.6f\r\n",
               (double)mean_mse, (double)std_mse,
               (double)g_sv.anomaly_threshold);

    if (!g_sv.flag_sd_error) {
        Storage_SaveWeights(&g_ae, &g_norm);
        Storage_SaveThreshold(g_sv.anomaly_threshold);
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

    float mse = AE_Forward(&g_ae, g_sv.feat_norm);
    g_sv.total_windows++;

    uint8_t is_anomaly = (mse > g_sv.anomaly_threshold) ? 1 : 0;

    if (is_anomaly) {
        g_sv.anomaly_streak++;
        g_sv.total_anomalies++;

        DBG_Printf("[MONITOR] ANOMALY! mse=%.6f thr=%.6f streak=%lu\r\n",
                   (double)mse, (double)g_sv.anomaly_threshold,
                   (unsigned long)g_sv.anomaly_streak);

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
            DBG_Printf("[ALERT] Sustained anomaly detected! "
                       "Streak=%lu total=%lu\r\n",
                       (unsigned long)g_sv.anomaly_streak,
                       (unsigned long)g_sv.total_anomalies);
        }
    } else {
        g_sv.anomaly_streak = 0;

        DBG_Printf("[MONITOR] normal mse=%.6f\r\n", (double)mse);

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
    DBG_Printf("[TRAIN] Online training %lu windows × %u epochs\r\n",
               (unsigned long)g_sv.train_buf_idx, SNX_MAX_EPOCHS);

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

        DBG_Printf("[TRAIN] epoch=%lu loss=%.6f\r\n",
                   (unsigned long)epoch,
                   (double)(epoch_loss / (float)g_sv.train_buf_idx));
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
    if ((HAL_GetTick() - last_alert) > 5000) {
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
        DBG_Printf("[STATS] state=%s windows=%lu anomalies=%lu thr=%.6f\r\n",
                   STATE_NAMES[g_sv.state],
                   (unsigned long)g_sv.total_windows,
                   (unsigned long)g_sv.total_anomalies,
                   (double)g_sv.anomaly_threshold);
        g_sv.last_print_tick = HAL_GetTick();
    }

    switch (g_sv.state) {
        case SNX_STATE_IDLE:                                    break;
        case SNX_STATE_COLLECT:   handle_collect();             break;
        case SNX_STATE_CALIBRATE: handle_calibrate();           break;
        case SNX_STATE_MONITOR:   handle_monitor();             break;
        case SNX_STATE_TRAIN:     handle_train();               break;
        case SNX_STATE_FALLBACK:  handle_fallback();            break;
        default:                  transition(SNX_STATE_FALLBACK); break;
    }
}

void Supervisor_HandleCommand(uint8_t cmd)
{
    DBG_Printf("[CMD] Received '%c'\r\n", cmd);

    switch (cmd) {
        case SNX_CMD_CALIBRATE:
            DBG_Printf("[CMD] Starting calibration...\r\n");
            AE_Init(&g_ae);
            Features_NormInit(&g_norm);
            g_sv.total_windows  = 0;
            g_sv.total_anomalies = 0;
            g_sv.anomaly_streak = 0;
            g_sv.train_buf_idx  = 0;
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
            g_sv.raw_log_enabled ^= 1;
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
    DBG_Printf("  State         : %s\r\n", STATE_NAMES[g_sv.state]);
    DBG_Printf("  Total windows : %lu\r\n", (unsigned long)g_sv.total_windows);
    DBG_Printf("  Total anomaly : %lu\r\n", (unsigned long)g_sv.total_anomalies);
    DBG_Printf("  Streak        : %lu\r\n", (unsigned long)g_sv.anomaly_streak);
    DBG_Printf("  Threshold     : %.6f\r\n", (double)g_sv.anomaly_threshold);
    DBG_Printf("  Last MSE      : %.6f\r\n", (double)g_ae.last_mse);
    DBG_Printf("  Train steps   : %lu\r\n", (unsigned long)g_ae.train_steps);
    DBG_Printf("  Norm init     : %s\r\n", g_norm.is_initialised ? "YES" : "NO");
    DBG_Printf("  SD error      : %s\r\n", g_sv.flag_sd_error ? "YES" : "NO");
    DBG_Printf("  IMU error     : %s\r\n", g_sv.flag_imu_error ? "YES" : "NO");
    DBG_Printf("  Raw log       : %s\r\n", g_sv.raw_log_enabled ? "ON" : "OFF");
    DBG_Printf("  Calib samples : %lu\r\n", (unsigned long)g_sv.calib_count);
    DBG_Printf("  Train buf     : %lu / %u\r\n",
               (unsigned long)g_sv.train_buf_idx, SNX_TRAIN_BATCH_SIZE);
    DBG_Printf("=========================================\r\n");
}
