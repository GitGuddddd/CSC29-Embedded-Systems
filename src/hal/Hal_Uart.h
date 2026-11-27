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
/* One RX owner, on the initializing core. The DMA buffer is naturally aligned. */
bool Hal_Uart_DmaInit(uint8_t port, volatile uint8_t * const p_circ_buf, uint32_t buf_size);
typedef struct
{
    uint32_t produced_bytes; /* Monotonic modulo 2^32, not just a ring index. */
    bool fault;
} Hal_Uart_RxSnapshot_t;

bool Hal_Uart_GetRxSnapshot(Hal_Uart_RxSnapshot_t * const p_snapshot);

#ifndef PICO_ON_DEVICE
/* Host transport simulation, excluded from firmware. */
bool Hal_Uart_TestInjectRx(const uint8_t * const p_data, uint32_t len);
void Hal_Uart_TestSetProduced(uint32_t count);
void Hal_Uart_TestSetDmaFault(bool fault);
#endif

#endif /* HAL_UART_H */
