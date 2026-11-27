#ifndef HAL_UART_CFG_H
#define HAL_UART_CFG_H

/* 8N1 at 256 kbaud is at most 25,600 bytes/s: 1024 bytes spans 40 ms. */
#define HAL_UART_DMA_RING_BYTES       (1024U)
#define HAL_UART_DMA_RING_BITS        (10U)
#define HAL_UART_DMA_BLOCK_BYTES      (30U) /* One radar-sized committed DMA block. */
#define HAL_UART_DMA_IRQ_PRIORITY     (0x80U)

#endif /* HAL_UART_CFG_H */
