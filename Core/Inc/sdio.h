/**
 * @file    sdio.h
 * @brief   SDIO peripheral handle and init prototype
 *
 * Pin mapping (WeAct STM32F405RGT6, from schematic TF interface):
 *   PC8  SDIO_D0  PC9  SDIO_D1  PC10 SDIO_D2
 *   PC11 SDIO_D3  PC12 SDIO_CK  PD2  SDIO_CMD
 *
 */

#ifndef SDIO_H
#define SDIO_H

#include "stm32f4xx_hal.h"
#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

extern SD_HandleTypeDef hsd;

void MX_SDIO_SD_Init(void);

#ifdef __cplusplus
}
#endif

#endif /* SDIO_H */
