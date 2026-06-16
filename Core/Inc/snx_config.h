/**
 * @file    snx_config.h
 * @brief   SensiNerveX Anomaly Detection — Global Configuration
 *
 * Target  : WeAct STM32F405RGT6 @ 168 MHz
 * Sensor  : MPU-6050 (I2C1)
 * Storage : TF/SD card (SPI1, FatFS)
 * UART    : UART2 — debug / telemetry only
 *
 * Architecture : Sparse Autoencoder MLP
 *   Encoder  : 10 → 8 → 4
 *   Decoder  : 4  → 8 → 10
 *   Total params = (10×8+8) + (8×4+4) + (4×8+8) + (8×10+10) = 318 parameters
 *
 * Anomaly score = MSE between input features and reconstruction.
 * If score > SNX_ANOMALY_THRESHOLD → anomaly event.
 */

#ifndef SNX_CONFIG_H
#define SNX_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

#define SNX_NODE_ID                 0x01u   /* Unique per board (0x01 / 0x02 / 0x03) */
#define SNX_FW_VERSION_MAJOR        3
#define SNX_FW_VERSION_MINOR        1       /* bumped for research-logging additions  */

#define SNX_SAMPLE_RATE_HZ          200u    /* MPU-6050 ODR                          */
#define SNX_WINDOW_SIZE             50u     /* Samples per feature window            */
#define SNX_WINDOW_STRIDE           25u     /* 50 % overlap                          */
#define SNX_FEATURE_DIM             10u     /* Features extracted per window         */

#define SNX_AE_INPUT_DIM            10u
#define SNX_AE_H1_DIM               8u
#define SNX_AE_LATENT_DIM           4u
#define SNX_AE_H2_DIM               8u
#define SNX_AE_OUTPUT_DIM           10u

#define SNX_LEARNING_RATE           0.005f
#define SNX_MOMENTUM                0.85f
#define SNX_L2_LAMBDA               1e-4f
#define SNX_MAX_EPOCHS              10u
#define SNX_TRAIN_BATCH_SIZE        16u
#define SNX_MIN_NORMAL_SAMPLES      64u

#define SNX_ANOMALY_THRESHOLD       0.025f
#define SNX_ANOMALY_STREAK_LIMIT    5u
#define SNX_CALIBRATION_WINDOWS     100u
#define SNX_THRESHOLD_SIGMA_MULT    3.0f

#define SNX_NORM_ALPHA              0.01f

#define SNX_SD_RAW_FILENAME         "RAWIMU.CSV"
#define SNX_SD_LOG_FILENAME         "ANOMLOG.CSV"
#define SNX_SD_STATS_FILENAME       "RUNSTAT.CSV"
#define SNX_SD_WEIGHTS_FILENAME     "WEIGHTS.BIN"

// Research logging files
#define SNX_SD_EVAL_FILENAME        "EVAL.CSV"
#define SNX_SD_LATENT_FILENAME      "LATENT.CSV"
#define SNX_SD_TRAIN_FILENAME       "TRAIN.CSV"
#define SNX_SD_CALIB_FILENAME       "CALIB.CSV"
#define SNX_SD_PROFILE_FILENAME     "PROFILE.CSV"

#define SNX_SD_SYNC_INTERVAL_MS     5000u
#define SNX_SD_MAGIC                0x534E5833u  /* 'SNX3' */

#define SNX_UART_TX_TIMEOUT_MS      10u
#define SNX_DEBUG_PRINT_INTERVAL_MS 1000u

#define SNX_CALIBRATION_TIMEOUT_MS  60000u
#define SNX_TICK_RATE_MS            10u

#define SNX_CPU_MHZ                 168u    /* STM32F405 @ 168 MHz                   */

#define SNX_CMD_CALIBRATE           'C'
#define SNX_CMD_TRAIN               'T'
#define SNX_CMD_STATUS              'S'
#define SNX_CMD_RESET_WEIGHTS       'R'
#define SNX_CMD_SAVE_WEIGHTS        'W'
#define SNX_CMD_LOAD_WEIGHTS        'L'
#define SNX_CMD_SET_THRESHOLD       'H'
#define SNX_CMD_TOGGLE_RAW_LOG      'N'

/*
 *   '0' → NORMAL          '3' → LOOSE_MOUNT
 *   '1' → IMPACT          '4' → MISALIGNMENT
 *   '2' → IMBALANCE       '5' → BEARING_FAULT
 *
 */
#define SNX_CMD_LABEL_NORMAL        '0'
#define SNX_CMD_LABEL_IMPACT        '1'
#define SNX_CMD_LABEL_IMBALANCE     '2'
#define SNX_CMD_LABEL_LOOSE_MOUNT   '3'
#define SNX_CMD_LABEL_MISALIGNMENT  '4'
#define SNX_CMD_LABEL_BEARING_FAULT '5'

#ifdef __cplusplus
}
#endif

#endif /* SNX_CONFIG_H */
