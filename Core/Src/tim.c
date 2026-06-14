/**
 * @file    tim.c  (CubeMX-generated skeleton — TIM6 @ 200 Hz)
 *
 * TIM6 is a basic timer on APB1 (84 MHz on STM32F405 @ 168 MHz).
 * APB1 timer clock = 84 MHz × 2 = 168 MHz (when APB1 prescaler > 1).
 *
 * Target: 200 Hz → period = 168,000,000 / 200 = 840,000 counts
 *
 * CubeMX settings (TIM6 → Parameter Settings):
 *   Prescaler     : 839      (divides 168 MHz → 200,000 Hz)
 *   Counter Mode  : Up
 *   Counter Period: 999      (200,000 / (999+1) = 200 Hz)
 *   auto-reload   : Enable
 *
 * NVIC: TIM6_DAC global interrupt → Enable, Priority 1 (below SysTick=0)
 *
 */

#include "tim.h"

TIM_HandleTypeDef htim6;

void MX_TIM6_Init(void)
{
    TIM_MasterConfigTypeDef sMasterConfig = {0};

    htim6.Instance               = TIM6;
    htim6.Init.Prescaler         = 839;
    htim6.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim6.Init.Period            = 999;
    htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;

    if (HAL_TIM_Base_Init(&htim6) != HAL_OK) {
        Error_Handler();
    }

    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;

    if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK) {
        Error_Handler();
    }
}

void HAL_TIM_Base_MspInit(TIM_HandleTypeDef *htim_base)
{
    if (htim_base->Instance == TIM6) {
        __HAL_RCC_TIM6_CLK_ENABLE();
        HAL_NVIC_SetPriority(TIM6_DAC_IRQn, 1, 0);
        HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);
    }
}
