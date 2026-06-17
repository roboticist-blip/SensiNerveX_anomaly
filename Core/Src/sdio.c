/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    sdio.c
  * @brief   This file provides code for the configuration
  *          of the SDIO instances.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "sdio.h"
#include "debug_uart.h"

SD_HandleTypeDef hsd;

/* DMA handles still declared — needed for IRQ handlers in stm32f4xx_it.c.
 * They are initialised but NOT linked to hsd, so HAL uses polling mode. */
DMA_HandleTypeDef hdma_sdio_rx;
DMA_HandleTypeDef hdma_sdio_tx;

void MX_SDIO_SD_Init(void)
{
    hsd.Instance                 = SDIO;
    hsd.Init.ClockEdge           = SDIO_CLOCK_EDGE_RISING;
    hsd.Init.ClockBypass         = SDIO_CLOCK_BYPASS_DISABLE;
    hsd.Init.ClockPowerSave      = SDIO_CLOCK_POWER_SAVE_DISABLE;
    hsd.Init.BusWide             = SDIO_BUS_WIDE_1B;  /* widened to 4B in disk_initialize */
    hsd.Init.HardwareFlowControl = SDIO_HARDWARE_FLOW_CONTROL_DISABLE;
    hsd.Init.ClockDiv            = 118;               /* 48MHz/(118+2) = 0.4 MHz — safe */

    if (HAL_SD_Init(&hsd) != HAL_OK) {
        DBG_Printf("[SDIO] HAL_SD_Init failed\r\n");
        Error_Handler();
    }
}

void HAL_SD_MspInit(SD_HandleTypeDef *sdHandle)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    if (sdHandle->Instance != SDIO)
        return;

    __HAL_RCC_SDIO_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();

    /*
     * SDIO GPIO — WeAct STM32F405RGT6 onboard TF slot
     *   PC8  SDIO_D0   PC9  SDIO_D1   PC10 SDIO_D2
     *   PC11 SDIO_D3   PC12 SDIO_CK   PD2  SDIO_CMD
     */
    GPIO_InitStruct.Pin       = GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 |
                                GPIO_PIN_11 | GPIO_PIN_12;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_PULLUP;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF12_SDIO;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    GPIO_InitStruct.Pin       = GPIO_PIN_2;
    GPIO_InitStruct.Alternate = GPIO_AF12_SDIO;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    /*
     * DMA streams are initialised so IRQ handlers in stm32f4xx_it.c compile,
     * but __HAL_LINKDMA is intentionally NOT called.
     * Without hdmatx/hdmarx linked, HAL_SD_WriteBlocks/ReadBlocks run in
     * blocking polling mode — the correct pairing for sdio_diskio.c.
     */
    /* ---- DMA2 Stream3 Ch4 = SDIO RX ---- */
    hdma_sdio_rx.Instance                 = DMA2_Stream3;
    hdma_sdio_rx.Init.Channel             = DMA_CHANNEL_4;
    hdma_sdio_rx.Init.Direction           = DMA_PERIPH_TO_MEMORY;
    hdma_sdio_rx.Init.PeriphInc           = DMA_PINC_DISABLE;
    hdma_sdio_rx.Init.MemInc              = DMA_MINC_ENABLE;
    hdma_sdio_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
    hdma_sdio_rx.Init.MemDataAlignment    = DMA_MDATAALIGN_WORD;
    hdma_sdio_rx.Init.Mode                = DMA_PFCTRL;
    hdma_sdio_rx.Init.Priority            = DMA_PRIORITY_VERY_HIGH;
    hdma_sdio_rx.Init.FIFOMode            = DMA_FIFOMODE_ENABLE;
    hdma_sdio_rx.Init.FIFOThreshold       = DMA_FIFO_THRESHOLD_FULL;
    hdma_sdio_rx.Init.MemBurst            = DMA_MBURST_INC4;
    hdma_sdio_rx.Init.PeriphBurst         = DMA_PBURST_INC4;
    if (HAL_DMA_Init(&hdma_sdio_rx) != HAL_OK) {
        DBG_Printf("[SDIO] DMA RX init failed\r\n");
        Error_Handler();
    }
    /* NOT linked: __HAL_LINKDMA(sdHandle, hdmarx, hdma_sdio_rx); */

    /* ---- DMA2 Stream6 Ch4 = SDIO TX ---- */
    hdma_sdio_tx.Instance                 = DMA2_Stream6;
    hdma_sdio_tx.Init.Channel             = DMA_CHANNEL_4;
    hdma_sdio_tx.Init.Direction           = DMA_MEMORY_TO_PERIPH;
    hdma_sdio_tx.Init.PeriphInc           = DMA_PINC_DISABLE;
    hdma_sdio_tx.Init.MemInc              = DMA_MINC_ENABLE;
    hdma_sdio_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
    hdma_sdio_tx.Init.MemDataAlignment    = DMA_MDATAALIGN_WORD;
    hdma_sdio_tx.Init.Mode                = DMA_PFCTRL;
    hdma_sdio_tx.Init.Priority            = DMA_PRIORITY_VERY_HIGH;
    hdma_sdio_tx.Init.FIFOMode            = DMA_FIFOMODE_ENABLE;
    hdma_sdio_tx.Init.FIFOThreshold       = DMA_FIFO_THRESHOLD_FULL;
    hdma_sdio_tx.Init.MemBurst            = DMA_MBURST_INC4;
    hdma_sdio_tx.Init.PeriphBurst         = DMA_PBURST_INC4;
    if (HAL_DMA_Init(&hdma_sdio_tx) != HAL_OK) {
        DBG_Printf("[SDIO] DMA TX init failed\r\n");
        Error_Handler();
    }
    /* NOT linked: __HAL_LINKDMA(sdHandle, hdmatx, hdma_sdio_tx); */

    /* SDIO interrupt — keep at priority 1 */
    HAL_NVIC_SetPriority(SDIO_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(SDIO_IRQn);

    /* DMA IRQs still enabled (IRQ handlers reference the handles) */
    HAL_NVIC_SetPriority(DMA2_Stream3_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(DMA2_Stream3_IRQn);
    HAL_NVIC_SetPriority(DMA2_Stream6_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(DMA2_Stream6_IRQn);
}

void HAL_SD_MspDeInit(SD_HandleTypeDef *sdHandle)
{
    if (sdHandle->Instance != SDIO)
        return;

    __HAL_RCC_SDIO_CLK_DISABLE();
    HAL_GPIO_DeInit(GPIOC, GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 |
                            GPIO_PIN_11 | GPIO_PIN_12);
    HAL_GPIO_DeInit(GPIOD, GPIO_PIN_2);
    HAL_DMA_DeInit(sdHandle->hdmarx);
    HAL_DMA_DeInit(sdHandle->hdmatx);
    HAL_NVIC_DisableIRQ(SDIO_IRQn);
}


