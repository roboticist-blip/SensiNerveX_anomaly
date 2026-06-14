/**
 * @file    main.h
 * @brief   SensiNerveX v3.0 application header
 *
 * Peripheral map (WeAct STM32F405RGT6):
 *   I2C1  PB6/PB7    → MPU-6050
 *   SDIO  PC8-12/PD2 → TF card (4-bit, DMA2)   ← NOT SPI1
 *   UART2 PA2/PA3    → Debug
 *   TIM6             → 200 Hz sampling
 */

#ifndef MAIN_H
#define MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"

extern I2C_HandleTypeDef  hi2c1;    /* MPU-6050         */
extern SD_HandleTypeDef   hsd;      /* SDIO TF card     */
extern UART_HandleTypeDef huart2;   /* Debug UART       */
extern TIM_HandleTypeDef  htim6;    /* 200 Hz timer     */

void MX_GPIO_Init(void);
void MX_I2C1_Init(void);
void MX_SDIO_SD_Init(void);         
void MX_USART2_UART_Init(void);
void MX_TIM6_Init(void);
void MX_FATFS_Init(void);

void Error_Handler(void);

#ifdef __cplusplus
}
#endif

#endif /* MAIN_H */
