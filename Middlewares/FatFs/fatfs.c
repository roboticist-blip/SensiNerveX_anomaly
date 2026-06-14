/**
 * @file    fatfs.c
 * @brief   FatFS middleware init — registers SDIO disk driver
 *
 * This replaces the CubeMX-generated fatfs.c which defaults to SPI.
 * We register SD_Driver (SDIO-backed) as drive "0:".
 *
 * Call MX_FATFS_Init() once from main() after MX_SDIO_SD_Init().
 * Then call Storage_Init() (storage.c) which calls f_mount().
 */

#include "fatfs.h"
#include "sdio_diskio.h"

char     SDPath[4];   
FATFS    SDFatFS;     

void MX_FATFS_Init(void)
{
    if (FATFS_LinkDriver(&SD_Driver, SDPath) != 0) {
        while (1) {}
    }
}
