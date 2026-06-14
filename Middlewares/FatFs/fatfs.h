/**
 * @file    fatfs.h
 */
#ifndef FATFS_H
#define FATFS_H

#include "ff.h"
#include "ff_gen_drv.h"
#include "sdio_diskio.h"

#ifdef __cplusplus
extern "C" {
#endif

extern char  SDPath[4];
extern FATFS SDFatFS;

void MX_FATFS_Init(void);

#ifdef __cplusplus
}
#endif

#endif /* FATFS_H */
