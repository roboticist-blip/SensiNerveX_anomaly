/**
 * @file    storage.c
 * @brief   SD card storage via FatFS for SensiNerveX Anomaly Detection
 *
 * Assumes FatFS is configured with:
 *   FF_FS_EXFAT = 0, FF_USE_FASTSEEK = 1, _VOLUMES = 1
 *
 * SPI1 CS is managed by FatFS disk driver (stm32_adafruit_sd.c or similar).
 */

#include "storage.h"
#include "debug_uart.h"
#include "main.h"
#include "stm32f4xx_hal.h"
#include "snx_config.h"
#include "ff.h"          /* FatFS */

#include <stdio.h>
#include <string.h>
#include <stdint.h>

static uint16_t crc16_ccitt(const uint8_t *data, uint32_t len)
{
    uint16_t crc = 0xFFFFu;
    while (len--) {
        crc ^= (uint16_t)(*data++) << 8;
        for (uint8_t i = 0; i < 8; i++) {
            if (crc & 0x8000u)
                crc = (uint16_t)((crc << 1) ^ 0x1021u);
            else
                crc <<= 1;
        }
    }
    return crc;
}

static FATFS  s_fatfs;
static FIL    s_fraw;       /* raw_imu.csv   */
static FIL    s_fanom;      /* anomaly_log.csv */
static FIL    s_fstats;     /* run_stats.csv  */
static uint8_t s_mounted = 0;

static char   s_linebuf[256];

SNX_Status Storage_Init(void)
{
    FRESULT fr;

    fr = f_mount(&s_fatfs, "", 1);
    if (fr != FR_OK) {
        DBG_Printf("[SD] Mount failed: %d\r\n", fr);
        return SNX_ERROR;
    }
    s_mounted = 1;

    fr = f_open(&s_fraw, SNX_SD_RAW_FILENAME,
                FA_OPEN_ALWAYS | FA_WRITE);
    if (fr != FR_OK) {
        DBG_Printf("[SD] Cannot open %s: %d\r\n", SNX_SD_RAW_FILENAME, fr);
        return SNX_ERROR;
    }
    f_lseek(&s_fraw, f_size(&s_fraw));
    if (f_size(&s_fraw) == 0) {
        f_puts("timestamp_ms,ax,ay,az,gx,gy,gz\r\n", &s_fraw);
    }

    fr = f_open(&s_fanom, SNX_SD_LOG_FILENAME,
                FA_OPEN_ALWAYS | FA_WRITE);
    if (fr != FR_OK) {
        DBG_Printf("[SD] Cannot open %s: %d\r\n", SNX_SD_LOG_FILENAME, fr);
        return SNX_ERROR;
    }
    f_lseek(&s_fanom, f_size(&s_fanom));
    if (f_size(&s_fanom) == 0) {
        f_puts("timestamp_ms,mse,threshold,"
               "f0,f1,f2,f3,f4,f5,f6,f7,f8,f9\r\n", &s_fanom);
    }

    fr = f_open(&s_fstats, SNX_SD_STATS_FILENAME,
                FA_OPEN_ALWAYS | FA_WRITE);
    if (fr != FR_OK) {
        DBG_Printf("[SD] Cannot open %s: %d\r\n", SNX_SD_STATS_FILENAME, fr);
        return SNX_ERROR;
    }
    f_lseek(&s_fstats, f_size(&s_fstats));
    if (f_size(&s_fstats) == 0) {
        f_puts("timestamp_ms,windows,anomalies,threshold,last_mse\r\n",
               &s_fstats);
    }

    DBG_Printf("[SD] Storage ready. Files open.\r\n");
    return SNX_OK;
}

void Storage_Sync(void)
{
    if (!s_mounted) return;
    f_sync(&s_fraw);
    f_sync(&s_fanom);
    f_sync(&s_fstats);
}

void Storage_Deinit(void)
{
    if (!s_mounted) return;
    f_close(&s_fraw);
    f_close(&s_fanom);
    f_close(&s_fstats);
    f_mount(NULL, "", 0);
    s_mounted = 0;
}

void Storage_Log_RawIMU(const IMU_Sample *s)
{
    if (!s_mounted) return;
    int n = snprintf(s_linebuf, sizeof(s_linebuf),
                     "%lu,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\r\n",
                     (unsigned long)s->timestamp,
                     (double)s->ax, (double)s->ay, (double)s->az,
                     (double)s->gx, (double)s->gy, (double)s->gz);
    if (n > 0) {
        UINT bw;
        f_write(&s_fraw, s_linebuf, (UINT)n, &bw);
    }
}

void Storage_Log_Anomaly(const AnomalyEvent *ev)
{
    if (!s_mounted) return;
    int n = snprintf(s_linebuf, sizeof(s_linebuf),
                     "%lu,%.6f,%.6f,"
                     "%.4f,%.4f,%.4f,%.4f,%.4f,"
                     "%.4f,%.4f,%.4f,%.4f,%.4f\r\n",
                     (unsigned long)ev->timestamp_ms,
                     (double)ev->mse_score,
                     (double)ev->threshold,
                     (double)ev->features[0], (double)ev->features[1],
                     (double)ev->features[2], (double)ev->features[3],
                     (double)ev->features[4], (double)ev->features[5],
                     (double)ev->features[6], (double)ev->features[7],
                     (double)ev->features[8], (double)ev->features[9]);
    if (n > 0) {
        UINT bw;
        f_write(&s_fanom, s_linebuf, (UINT)n, &bw);
    }
}

void Storage_Log_Stats(uint32_t windows, uint32_t anomalies,
                       float threshold, float last_mse)
{
    if (!s_mounted) return;
    int n = snprintf(s_linebuf, sizeof(s_linebuf),
                     "%lu,%lu,%lu,%.6f,%.6f\r\n",
                     (unsigned long)HAL_GetTick(),
                     (unsigned long)windows,
                     (unsigned long)anomalies,
                     (double)threshold, (double)last_mse);
    if (n > 0) {
        UINT bw;
        f_write(&s_fstats, s_linebuf, (UINT)n, &bw);
    }
}

/*
 * Binary layout (all little-endian):
 *   uint32 magic          4
 *   uint32 version        4
 *   uint32 n_weights      4
 *   uint32 crc16          4
 *   float  weights[254]   1016
 *   float  norm_mean[10]  40
 *   float  norm_var[10]   40
 *   uint32 norm_n         4
 *   uint32 norm_init      4
 *   float  threshold      4
 *   ─────────────────────────
 *   Total                 1120 bytes
 */
SNX_Status Storage_SaveWeights(const AE_Model *m, const NormStats *ns)
{
    if (!s_mounted) return SNX_ERROR;

    FIL f;
    FRESULT fr = f_open(&f, SNX_SD_WEIGHTS_FILENAME, FA_CREATE_ALWAYS | FA_WRITE);
    if (fr != FR_OK) {
        DBG_Printf("[SD] SaveWeights open failed: %d\r\n", fr);
        return SNX_ERROR;
    }

    static float wbuf[AE_WEIGHT_FLAT_SIZE];  
    AE_WeightsToFlat(m, wbuf);

    uint16_t crc = crc16_ccitt((const uint8_t *)wbuf,
                                AE_WEIGHT_FLAT_SIZE * sizeof(float));

    uint32_t magic    = SNX_SD_MAGIC;
    uint32_t version  = ((uint32_t)SNX_FW_VERSION_MAJOR << 16) |
                         (uint32_t)SNX_FW_VERSION_MINOR;
    uint32_t nw       = AE_WEIGHT_FLAT_SIZE;
    uint32_t crc32_wr = (uint32_t)crc;

    UINT bw;
    f_write(&f, &magic,    4, &bw);
    f_write(&f, &version,  4, &bw);
    f_write(&f, &nw,       4, &bw);
    f_write(&f, &crc32_wr, 4, &bw);

    f_write(&f, wbuf, AE_WEIGHT_FLAT_SIZE * sizeof(float), &bw);

    f_write(&f, ns->mean,  SNX_FEATURE_DIM * sizeof(float), &bw);
    f_write(&f, ns->var,   SNX_FEATURE_DIM * sizeof(float), &bw);
    uint32_t nn  = ns->n_updates;
    uint32_t nii = ns->is_initialised;
    f_write(&f, &nn,  4, &bw);
    f_write(&f, &nii, 4, &bw);

    float thr = 0.0f;
    f_write(&f, &thr, sizeof(float), &bw);

    f_close(&f);

    DBG_Printf("[SD] Weights saved (%u floats, CRC=0x%04X)\r\n",
               (unsigned)AE_WEIGHT_FLAT_SIZE, crc);
    return SNX_OK;
}

SNX_Status Storage_LoadWeights(AE_Model *m, NormStats *ns)
{
    if (!s_mounted) return SNX_ERROR;

    FIL f;
    FRESULT fr = f_open(&f, SNX_SD_WEIGHTS_FILENAME, FA_READ);
    if (fr != FR_OK) {
        DBG_Printf("[SD] LoadWeights: file not found\r\n");
        return SNX_ERROR;
    }

    UINT br;
    uint32_t magic, version, nw, crc_stored;

    f_read(&f, &magic,      4, &br);
    f_read(&f, &version,    4, &br);
    f_read(&f, &nw,         4, &br);
    f_read(&f, &crc_stored, 4, &br);

    if (magic != SNX_SD_MAGIC) {
        DBG_Printf("[SD] Bad magic 0x%08lX\r\n", (unsigned long)magic);
        f_close(&f);
        return SNX_ERROR;
    }
    if (nw != AE_WEIGHT_FLAT_SIZE) {
        DBG_Printf("[SD] Weight count mismatch: got %lu expected %u\r\n",
                   (unsigned long)nw, (unsigned)AE_WEIGHT_FLAT_SIZE);
        f_close(&f);
        return SNX_ERROR;
    }

    static float wbuf[AE_WEIGHT_FLAT_SIZE];
    f_read(&f, wbuf, nw * sizeof(float), &br);

    uint16_t crc_calc = crc16_ccitt((const uint8_t *)wbuf,
                                     nw * sizeof(float));
    if ((uint16_t)crc_stored != crc_calc) {
        DBG_Printf("[SD] CRC mismatch! stored=0x%04lX calc=0x%04X\r\n",
                   (unsigned long)crc_stored, crc_calc);
        f_close(&f);
        return SNX_ERROR;
    }

    AE_WeightsFromFlat(m, wbuf);

    f_read(&f, ns->mean,  SNX_FEATURE_DIM * sizeof(float), &br);
    f_read(&f, ns->var,   SNX_FEATURE_DIM * sizeof(float), &br);
    uint32_t nn, nii;
    f_read(&f, &nn,  4, &br);
    f_read(&f, &nii, 4, &br);
    ns->n_updates      = nn;
    ns->is_initialised = (uint8_t)nii;

    f_close(&f);

    m->is_trained = 1;
    DBG_Printf("[SD] Weights loaded OK. CRC=0x%04X steps=%lu\r\n",
               crc_calc, (unsigned long)m->train_steps);
    return SNX_OK;
}

/*
 * The threshold is stored at a fixed byte offset in ae_weights.bin.
 * Offset = 4+4+4+4 + AE_WEIGHT_FLAT_SIZE*4 + SNX_FEATURE_DIM*4*2 + 4+4
 *        = 16 + 1016 + 80 + 8 = 1120 - 4 = byte 1116
 */
#define THRESHOLD_BYTE_OFFSET  (16u + AE_WEIGHT_FLAT_SIZE*4u + \
                                 SNX_FEATURE_DIM*4u*2u + 4u + 4u)

SNX_Status Storage_SaveThreshold(float threshold)
{
    if (!s_mounted) return SNX_ERROR;

    FIL f;
    FRESULT fr = f_open(&f, SNX_SD_WEIGHTS_FILENAME, FA_READ | FA_WRITE);
    if (fr != FR_OK) return SNX_ERROR;

    f_lseek(&f, THRESHOLD_BYTE_OFFSET);
    UINT bw;
    f_write(&f, &threshold, sizeof(float), &bw);
    f_close(&f);

    DBG_Printf("[SD] Threshold saved: %.6f at offset %u\r\n",
               (double)threshold, (unsigned)THRESHOLD_BYTE_OFFSET);
    return SNX_OK;
}

SNX_Status Storage_LoadThreshold(float *threshold)
{
    if (!s_mounted) return SNX_ERROR;

    FIL f;
    FRESULT fr = f_open(&f, SNX_SD_WEIGHTS_FILENAME, FA_READ);
    if (fr != FR_OK) return SNX_ERROR;

    f_lseek(&f, THRESHOLD_BYTE_OFFSET);
    UINT br;
    f_read(&f, threshold, sizeof(float), &br);
    f_close(&f);

    if (br != sizeof(float) || *threshold <= 0.0f) {
        return SNX_ERROR;
    }
    return SNX_OK;
}
