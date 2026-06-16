/**
 * @file    main.c
 * @brief   SensiNerveX v3.0 — Vibration Anomaly Detection
 *          WeAct STM32F405RGT6 @ 168 MHz
 *
 * Peripheral ownership:
 *   I2C1  — MPU-6050          (PB6 SCL, PB7 SDA)        AF4
 *   SDIO  — TF/SD card 4-bit  (PC8-12, PD2)             AF12  + DMA2
 *   UART2 — Debug/telemetry   (PA2 TX,  PA3 RX)          AF7
 *   TIM6  — 200 Hz sampling   (basic timer, no channel)
 *   DWT   — Cycle counter for training profiling
 *
 * Boot sequence:
 *   1. HAL + clock init
 *   2. DWT enable
 *   3. Peripheral inits (I2C1, SDIO, UART2, TIM6)
 *   4. FatFS link (MX_FATFS_Init)
 *   5. Supervisor_Init() → f_mount → weight load → AE init
 *   6. IMU_Init()
 *   7. TIM6 start → 200 Hz sampling flag
 *   8. Main loop: sample → supervisor tick → UART command poll
 */

#include "main.h"
#include "fatfs.h"
#include "sdio.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

#include "supervisor.h"
#include "imu.h"
#include "debug_uart.h"
#include "snx_config.h"

static volatile uint8_t g_sample_flag = 0;
static uint32_t         g_last_tick   = 0;

void SystemClock_Config(void);
static void DWT_Init(void);

int main(void)
{
    HAL_Init();
    SystemClock_Config();

    MX_GPIO_Init();
    MX_USART2_UART_Init();
    MX_I2C1_Init();
    MX_SDIO_SD_Init();
    MX_TIM6_Init();
    MX_FATFS_Init();
    DWT_Init();

    DBG_Printf("\r\n");
    DBG_Printf("========================================\r\n");
    DBG_Printf("  SensiNerveX v%d.%d  Node 0x%02X\r\n",
               SNX_FW_VERSION_MAJOR, SNX_FW_VERSION_MINOR, SNX_NODE_ID);
    DBG_Printf("  Vibration Anomaly Detection\r\n");
    DBG_Printf("  STM32F405RGT6 @ 168 MHz\r\n");
    DBG_Printf("  SD: SDIO 4-bit (PC8-12, PD2)\r\n");
    DBG_Printf("========================================\r\n\r\n");

    if (!IMU_SelfTest()) {
        DBG_Printf("[MAIN] MPU-6050 WHO_AM_I FAIL — check I2C1 (PB8/PB9)\r\n");
    } else {
        DBG_Printf("[MAIN] MPU-6050 detected OK\r\n");
    }

    Supervisor_Init();

    if (IMU_Init() != 0) {
        DBG_Printf("[MAIN] MPU-6050 init FAILED\r\n");
        g_sv.flag_imu_error = 1;
    }

    HAL_TIM_Base_Start_IT(&htim6);
    DBG_Printf("[MAIN] TIM6 started @ %u Hz\r\n", SNX_SAMPLE_RATE_HZ);

    DBG_Printf("[MAIN] UART commands:\r\n");
    DBG_Printf("  C=Calibrate  T=Train  S=Status  R=ResetWeights\r\n");
    DBG_Printf("  W=Save  L=Load  H=SetThreshold  N=ToggleRawLog\r\n\r\n");


    while (1) {
        if (g_sample_flag) {
            g_sample_flag = 0;
            IMU_Sample sample;
            if (IMU_Read(&sample) == 0) {
                Supervisor_FeedSample(&sample);
            } else {
                DBG_Printf("[MAIN] IMU read error\r\n");
                g_sv.flag_imu_error = 1;
            }
        }

        uint8_t cmd = 0;
        if (DBG_RxPoll(&cmd)) {
            DBG_Printf("[RX] 0x%02X '%c'\r\n", cmd, cmd);
            Supervisor_HandleCommand(cmd);
        }

        uint32_t now = HAL_GetTick();
        if ((now - g_last_tick) >= SNX_TICK_RATE_MS) {
            g_last_tick = now;
            Supervisor_Tick();
        }
    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM6) {
        g_sample_flag = 1;
    }
}

static void DWT_Init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;
}

void Error_Handler(void)
{
    __disable_irq();
    DBG_Printf("[FATAL] Error_Handler!\r\n");
    while (1) {}
}
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM = 8;
    RCC_OscInitStruct.PLL.PLLN = 336;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = 7;

    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

    RCC_ClkInitStruct.ClockType =
        RCC_CLOCKTYPE_HCLK |
        RCC_CLOCKTYPE_SYSCLK |
        RCC_CLOCKTYPE_PCLK1 |
        RCC_CLOCKTYPE_PCLK2;

    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
    {
        Error_Handler();
    }
}


#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
    DBG_Printf("[ASSERT] %s line %lu\r\n", file, (unsigned long)line);
}
#endif
