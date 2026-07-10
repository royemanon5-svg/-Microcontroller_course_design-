#ifndef __SOFT_UART_H
#define __SOFT_UART_H

#include "stm32f1xx_hal.h"

/**
 * @brief  Initialize software UART.
 * @param  baudrate: baud rate (e.g. 9600, 19200, 38400, 115200)
 * @note   PB3 = TX (output), PB4 = RX (input with pull-up)
 *         JTAG pins are released, SWD (PA13/PA14) remains usable.
 */
void SoftUART_Init(uint32_t baudrate);

/**
 * @brief  Send a single byte (blocking).
 */
void SoftUART_SendByte(uint8_t data);

/**
 * @brief  Send a null-terminated string (blocking).
 */
void SoftUART_SendString(const char *str);

/**
 * @brief  Formatted printf via soft UART.
 * @note   Uses internal 128-byte buffer.
 */
void SoftUART_Printf(const char *fmt, ...);

/**
 * @brief  Blocking receive with timeout.
 * @param  timeout_ms: 0 = wait forever
 * @retval Received byte, or 0 on timeout / false start.
 */
uint8_t SoftUART_ReceiveByte(uint32_t timeout_ms);

#endif
