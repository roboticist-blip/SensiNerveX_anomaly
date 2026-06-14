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
#define SNX_FW_VERSION_MINOR        0

#define SNX_SAMPLE_RATE_HZ          200u    /* MPU-6050 ODR                          */
#define SNX_WINDOW_SIZE             50u     /* Samples per feature window            */
#define SNX_WINDOW_STRIDE           25u     /* 50 % overlap                          */
#define SNX_FEATURE_DIM             10u     /* Features extracted per window         */
                                            /* [ax_mean,ax_std,ax_rms,              */
                                            /*  ay_mean,ay_std,ay_rms,              */
                                            /*  az_mean,az_std,az_rms,              */
                                            /*  resultant_peak]                      */


#define SNX_AE_INPUT_DIM            10u     /* = SNX_FEATURE_DIM                     */
#define SNX_AE_H1_DIM               8u     /* Encoder hidden layer                  */
#define SNX_AE_LATENT_DIM           4u     /* Bottleneck (latent space)             */
#define SNX_AE_H2_DIM               8u     /* Decoder hidden layer                  */
#define SNX_AE_OUTPUT_DIM           10u     /* Reconstruction output                 */

/* Total weight count (for static allocation):
   enc_W1: 10*8=80, enc_b1: 8, enc_W2: 8*4=32, enc_b2: 4
   dec_W1:  4*8=32, dec_b1: 8, dec_W2: 8*10=80, dec_b2: 10
   Total = 254 weights + 30 biases = 284 scalars                                    */

#define SNX_LEARNING_RATE           0.005f
#define SNX_MOMENTUM                0.85f
#define SNX_L2_LAMBDA               1e-4f
#define SNX_MAX_EPOCHS              10u     /* Epochs per online training batch      */
#define SNX_TRAIN_BATCH_SIZE        16u     /* Windows per training batch            */
#define SNX_MIN_NORMAL_SAMPLES      64u     /* Minimum normal windows before train   */


#define SNX_ANOMALY_THRESHOLD       0.025f  /* MSE reconstruction error threshold    */
                                            /* Tune after calibration run            */
#define SNX_ANOMALY_STREAK_LIMIT    5u      /* Consecutive anomalies → alert         */
#define SNX_CALIBRATION_WINDOWS     100u    /* Normal windows used to set threshold  */
#define SNX_THRESHOLD_SIGMA_MULT    3.0f    /* threshold = mean + k*sigma            */


#define SNX_NORM_ALPHA              0.01f   /* EMA coefficient for running mean/var  */


#define SNX_SD_RAW_FILENAME     "RAWIMU.CSV"
#define SNX_SD_LOG_FILENAME     "ANOMLOG.CSV"
#define SNX_SD_STATS_FILENAME   "RUNSTAT.CSV"
#define SNX_SD_WEIGHTS_FILENAME  "WEIGHTS.bin"
#define SNX_SD_SYNC_INTERVAL_MS     5000u   /* Flush to SD every N ms               */
#define SNX_SD_MAGIC                0x534E5833u  /* 'SNX3'                          */


#define SNX_UART_TX_TIMEOUT_MS      10u
#define SNX_DEBUG_PRINT_INTERVAL_MS 1000u   /* Periodic stats print interval         */


#define SNX_CALIBRATION_TIMEOUT_MS  60000u  /* Max time in CALIBRATE state (1 min)  */
#define SNX_TICK_RATE_MS            10u     /* Supervisor FSM tick period            */


#define SNX_CMD_CALIBRATE           'C'     /* (Re)start calibration phase           */
#define SNX_CMD_TRAIN               'T'     /* Force one training pass               */
#define SNX_CMD_STATUS              'S'     /* Print current status                  */
#define SNX_CMD_RESET_WEIGHTS       'R'     /* Xavier re-initialise weights          */
#define SNX_CMD_SAVE_WEIGHTS        'W'     /* Save weights to SD                    */
#define SNX_CMD_LOAD_WEIGHTS        'L'     /* Load weights from SD                  */
#define SNX_CMD_SET_THRESHOLD       'H'     /* Manually set anomaly threshold        */
#define SNX_CMD_TOGGLE_RAW_LOG      'N'     /* Toggle raw IMU CSV logging            */

#ifdef __cplusplus
}
#endif

#endif /* SNX_CONFIG_H */
