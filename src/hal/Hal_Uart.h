#ifndef HAL_UART_H
#define HAL_UART_H

#include <stdint.h>
#include <stdbool.h>

/*
 * \file Hal_Uart.h
 * \brief Hardware Abstraction Layer for RP2040 UART peripherals (UART0, UART1)
 * Compliance: Barr-C:2018
 */

void Hal_Uart_Init(uint8_t port, uint8_t tx_pin, uint8_t rx_pin, uint32_t baudrate);
bool Hal_Uart_DmaInit(uint8_t port, uint8_t * const p_circ_buf, uint32_t buf_size);
uint32_t Hal_Uart_GetDmaWriteIndex(uint32_t buf_size);

#endif /* HAL_UART_H */
