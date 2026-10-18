#include "Hal_Uart.h"

#ifdef PICO_ON_DEVICE
#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "hardware/dma.h"
#include "hardware/gpio.h"

static int s_dma_chan = -1;
#endif

void Hal_Uart_Init(uint8_t port, uint8_t tx_pin, uint8_t rx_pin, uint32_t baudrate)
{
#ifdef PICO_ON_DEVICE
    uart_inst_t * p_uart = (port == 0U) ? uart0 : uart1;
    uart_init(p_uart, baudrate);
    gpio_set_function(tx_pin, GPIO_FUNC_UART);
    gpio_set_function(rx_pin, GPIO_FUNC_UART);
    uart_set_hw_flow(p_uart, false, false);
    uart_set_format(p_uart, 8, 1, UART_PARITY_NONE);
    uart_set_fifo_enabled(p_uart, true);
#else
    (void)port;
    (void)tx_pin;
    (void)rx_pin;
    (void)baudrate;
#endif
}

bool Hal_Uart_DmaInit(uint8_t port, uint8_t * const p_circ_buf, uint32_t buf_size)
{
#ifdef PICO_ON_DEVICE
    uart_inst_t * p_uart = (port == 0U) ? uart0 : uart1;
    s_dma_chan = dma_claim_unused_channel(false);
    if (s_dma_chan < 0)
    {
        return false;
    }

    dma_channel_config c = dma_channel_get_default_config((uint)s_dma_chan);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_8);
    channel_config_set_dreq(&c, uart_get_dreq(p_uart, false));
    channel_config_set_read_increment(&c, false);
    channel_config_set_write_increment(&c, true);
    channel_config_set_ring(&c, true, 8); /* ring buffer size 256 bytes */

    dma_channel_configure(
        (uint)s_dma_chan,
        &c,
        p_circ_buf,
        &uart_get_hw(p_uart)->dr,
        0xFFFFFFFF,
        true
    );
    return true;
#else
    (void)port;
    (void)p_circ_buf;
    (void)buf_size;
    return true;
#endif
}

uint32_t Hal_Uart_GetDmaWriteIndex(uint32_t buf_size)
{
#ifdef PICO_ON_DEVICE
    if (s_dma_chan < 0)
    {
        return 0U;
    }
    return (uint32_t)(0xFFFFFFFF - dma_channel_hw_addr((uint)s_dma_chan)->transfer_count) % buf_size;
#else
    (void)buf_size;
    return 0U;
#endif
}
