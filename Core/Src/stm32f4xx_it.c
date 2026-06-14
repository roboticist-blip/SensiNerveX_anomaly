/**
 * @file    stm32f4xx_it.c
 * @brief   Interrupt Service Routines
 *
 * Only TIM6_DAC_IRQHandler and SysTick_Handler are needed for this project.
 * CubeMX will generate the rest; this file shows the minimum required.
 */

#include "stm32f4xx_it.h"
#include "main.h"
#include "debug_uart.h"
#include "sdio.h"

extern SD_HandleTypeDef hsd;
extern DMA_HandleTypeDef hdma_sdio_rx;
extern DMA_HandleTypeDef hdma_sdio_tx;
extern TIM_HandleTypeDef htim6;

void SysTick_Handler(void)
{
    HAL_IncTick();
}

void TIM6_DAC_IRQHandler(void)
{
    HAL_TIM_IRQHandler(&htim6);
}


void HardFault_Handler(void)
{
    DBG_Printf("\r\n[HARDFAULT]\r\n");

    while (1)
    {
    }
}

void MemManage_Handler(void)
{
    DBG_Printf("\r\n[MEMMANAGE]\r\n");

    while (1)
    {
    }
}

void BusFault_Handler(void)
{
    DBG_Printf("\r\n[BUSFAULT]\r\n");

    while (1)
    {
    }
}

void UsageFault_Handler(void)
{
    DBG_Printf("\r\n[USAGEFAULT]\r\n");

    while (1)
    {
    }
}

void DMA2_Stream3_IRQHandler(void)
{
    HAL_DMA_IRQHandler(&hdma_sdio_rx);
}

void DMA2_Stream6_IRQHandler(void)
{
    HAL_DMA_IRQHandler(&hdma_sdio_tx);
}

void SDIO_IRQHandler(void)
{
    HAL_SD_IRQHandler(&hsd);
}
