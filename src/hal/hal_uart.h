/** @file hal_uart.h
 *
 * @brief UART access for uT-Kernel 3.0 on the RP2040 (direct registers).
 *
 * @par
 * COPYRIGHT NOTICE: (c) 2026 Group 29. All rights reserved.
 *
 * Only this module and hal_time touch the kernel and the chip registers.
 * Drivers above it call these functions, so they can be built and tested
 * on a PC with mocks.
 */

#ifndef HAL_UART_H
#define HAL_UART_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/** Identifies one hardware UART instance. */
typedef enum
{
    HAL_UART_0 = 0,
    HAL_UART_1 = 1
} hal_uart_id_t;

/** Handler type for the UART receive interrupt. */
typedef void (*hal_uart_rx_handler_t)(void);

bool hal_uart_init (hal_uart_id_t id, uint32_t baud_rate, uint8_t tx_pin,
                    uint8_t rx_pin);
void hal_uart_write (hal_uart_id_t id, const uint8_t * p_data, size_t length);
bool hal_uart_is_readable (hal_uart_id_t id);
uint8_t hal_uart_read_byte (hal_uart_id_t id);
bool hal_uart_take_overrun (hal_uart_id_t id);
void hal_uart_set_rx_handler (hal_uart_id_t id,
                              hal_uart_rx_handler_t p_handler);

#endif /* HAL_UART_H */

/*** end of file ***/
