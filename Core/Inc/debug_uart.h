/**
 * @file    debug_uart.h
 * @brief   UART2 debug / telemetry output for SensiNerveX
 *
 * UART2 maps to PA2 (TX) / PA3 (RX) on STM32F405RGT6.
 * Baud rate configured in CubeMX (recommended: 115200 or 460800).
 *
 * DBG_Printf() is a thin wrapper around HAL_UART_Transmit with a
 * snprintf intermediate buffer. Safe to call from main context.
 * Do NOT call from ISR — use DBG_PutChar for single-byte ISR use.
 */

#ifndef DEBUG_UART_H
#define DEBUG_UART_H

#include <stdint.h>
#include <stdarg.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Printf-style debug output over UART2.
 *        Blocks until transmission complete (polling, no DMA).
 */
void DBG_Printf(const char *fmt, ...);

/**
 * @brief Transmit a single byte (ISR-safe, non-blocking attempt).
 */
void DBG_PutChar(uint8_t c);

/**
 * @brief Transmit a raw buffer of known length.
 */
void DBG_Write(const uint8_t *buf, uint16_t len);

/**
 * @brief Poll UART2 RX register for a pending byte (non-blocking).
 * @param out  Receives the byte if available.
 * @return     1 if byte received, 0 if RX empty.
 */
uint8_t DBG_RxPoll(uint8_t *out);

#ifdef __cplusplus
}
#endif

#endif /* DEBUG_UART_H */
