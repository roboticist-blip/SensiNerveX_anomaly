/**
 * @file    debug_uart.c
 * @brief   UART2 debug output — polling TX, polling RX
 */

#include "debug_uart.h"
#include "stm32f4xx_hal.h"
#include "snx_config.h"
#include <stdio.h>
#include <string.h>

extern UART_HandleTypeDef huart2;  /* Defined by CubeMX in usart.c */

#define DBG_BUF_SIZE  256u

void DBG_Printf(const char *fmt, ...)
{
    static char buf[DBG_BUF_SIZE];
    va_list args;
    va_start(args, fmt);
    int n = vsnprintf(buf, DBG_BUF_SIZE, fmt, args);
    va_end(args);

    if (n > 0) {
        HAL_UART_Transmit(&huart2, (uint8_t *)buf,
                          (uint16_t)(n > (int)DBG_BUF_SIZE ?
                                     (int)DBG_BUF_SIZE : n),
                          SNX_UART_TX_TIMEOUT_MS);
    }
}

void DBG_PutChar(uint8_t c)
{
    HAL_UART_Transmit(&huart2, &c, 1, 5);
}

void DBG_Write(const uint8_t *buf, uint16_t len)
{
    HAL_UART_Transmit(&huart2, (uint8_t *)buf, len,
                      SNX_UART_TX_TIMEOUT_MS);
}

/*
 * Directly checks the USART2 SR register for RXNE (bit 5).
 * Returns immediately — does not block.
 */
uint8_t DBG_RxPoll(uint8_t *out)
{
    if (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_RXNE)) {
        *out = (uint8_t)(huart2.Instance->DR & 0xFFu);
        return 1;
    }
    return 0;
}
