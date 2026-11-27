#include "Hal_Uart.h"
#include "Hal_Uart_Cfg.h"
#include <stddef.h>

#ifdef PICO_ON_DEVICE
#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "hardware/dma.h"
#include "hardware/gpio.h"
#include "hardware/irq.h"
#include "hardware/sync.h"

static int s_dma_chan = -1;
static uint32_t s_owner_core;
static uint32_t s_dma_mask;
static bool s_irq_registered;

/* Cortex-M exception ABI plus explicit compiler ISR annotation (BARR-C 6.5). */
#define HAL_UART_ISR_ATTR __attribute__((interrupt("IRQ")))
static void HAL_UART_ISR_ATTR hal_uart_rx_dma_isr(void);
#else
static uint32_t s_host_produced;
static uint32_t s_host_block_fill;
#endif

/* ISR-owned metadata is read with a very short local IRQ mask on its owning core.
 * No other core accesses these fields. Volatile is not used as a cross-core lock.
 */
static volatile uint8_t * s_rx_buffer;
static volatile uint32_t s_completed_bytes;
static volatile bool s_dma_fault;
static bool s_initialized;

_Static_assert(HAL_UART_DMA_RING_BYTES == (1U << HAL_UART_DMA_RING_BITS), "DMA ring alignment mismatch");
_Static_assert(HAL_UART_DMA_BLOCK_BYTES < HAL_UART_DMA_RING_BYTES, "DMA block must fit in the ring");

void Hal_Uart_Init(uint8_t port, uint8_t tx_pin, uint8_t rx_pin, uint32_t baudrate)
{
    if ((port > 1U) || (baudrate == 0U))
    {
        return;
    }
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

bool Hal_Uart_DmaInit(uint8_t port, volatile uint8_t * const p_circ_buf, uint32_t buf_size)
{
    if ((port > 1U) || (p_circ_buf == NULL) || (buf_size != HAL_UART_DMA_RING_BYTES) ||
        (((uintptr_t)p_circ_buf & (HAL_UART_DMA_RING_BYTES - 1U)) != 0U))
    {
        return false;
    }
#ifdef PICO_ON_DEVICE
    /* Reinitialization is task-only and must stay on the IRQ's original core. */
    if ((__get_current_exception() != 0U) || (s_initialized && (get_core_num() != s_owner_core)))
    {
        return false;
    }
    uart_inst_t * p_uart = (port == 0U) ? uart0 : uart1;
    if (s_dma_chan < 0)
    {
        s_dma_chan = dma_claim_unused_channel(false);
        if (s_dma_chan < 0)
        {
            return false;
        }
    }
    uint channel = (uint)s_dma_chan;
    s_dma_mask = (1U << channel);
    /* RP2040 E13: mask the completion interrupt before aborting/reconfiguring. */
    dma_channel_set_irq1_enabled(channel, false);
    dma_channel_abort(channel);
    dma_hw->ints1 = s_dma_mask;
    s_owner_core = get_core_num();
    s_rx_buffer = p_circ_buf;
    s_completed_bytes = 0U;
    s_dma_fault = false;

    dma_channel_config c = dma_channel_get_default_config(channel);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_8);
    channel_config_set_dreq(&c, uart_get_dreq(p_uart, false));
    channel_config_set_read_increment(&c, false);
    channel_config_set_write_increment(&c, true);
    channel_config_set_ring(&c, true, HAL_UART_DMA_RING_BITS);

    dma_channel_configure(
        channel,
        &c,
        p_circ_buf,
        &uart_get_hw(p_uart)->dr,
        HAL_UART_DMA_BLOCK_BYTES,
        false
    );
    if (!s_irq_registered)
    {
        irq_add_shared_handler(DMA_IRQ_1, hal_uart_rx_dma_isr, PICO_SHARED_IRQ_HANDLER_DEFAULT_ORDER_PRIORITY);
        irq_set_priority(DMA_IRQ_1, HAL_UART_DMA_IRQ_PRIORITY);
        s_irq_registered = true;
    }
    s_initialized = true;
    __dmb();
    dma_channel_set_irq1_enabled(channel, true);
    irq_set_enabled(DMA_IRQ_1, true);
    dma_start_channel_mask(s_dma_mask);
    return true;
#else
    s_rx_buffer = p_circ_buf;
    s_host_produced = 0U;
    s_host_block_fill = 0U;
    s_completed_bytes = 0U;
    s_dma_fault = false;
    s_initialized = true;
    return true;
#endif
}

bool Hal_Uart_GetRxSnapshot(Hal_Uart_RxSnapshot_t * const p_snapshot)
{
    if ((p_snapshot == NULL) || !s_initialized)
    {
        return false;
    }
#ifdef PICO_ON_DEVICE
    if ((__get_current_exception() != 0U) || (get_core_num() != s_owner_core))
    {
        return false;
    }
    /* Snapshot only completion-committed bytes, not a mid-transfer TRANS_COUNT.
     * DMA continues running. No copy, parse, logging or waiting while masked.
     */
    uint32_t saved = save_and_disable_interrupts();
    uint32_t completed = s_completed_bytes;
    bool fault = s_dma_fault || ((dma_hw->ch[(uint)s_dma_chan].ctrl_trig &
                                 DMA_CH0_CTRL_TRIG_AHB_ERROR_BITS) != 0U);
    restore_interrupts(saved);
    __dmb();
    p_snapshot->produced_bytes = completed;
    p_snapshot->fault = fault;
#else
    p_snapshot->produced_bytes = s_completed_bytes;
    p_snapshot->fault = s_dma_fault;
#endif
    return true;
}

#ifdef PICO_ON_DEVICE
/* BARR-C 6.5: private _isr, registered through the SDK's RAM vector/IRQ dispatcher.
 * One IRQ per 30 bytes, normally one complete radar frame, not one IRQ per byte.
 * Ack, commit fully written bytes, re-arm. UART FIFO covers the short re-arm gap.
 * Direct MMIO only: no loops, allocation, atomics/spinlocks, RTOS calls or application work.
 */
static void HAL_UART_ISR_ATTR hal_uart_rx_dma_isr(void)
{
    if ((dma_hw->ints1 & s_dma_mask) != 0U)
    {
        dma_hw->ints1 = s_dma_mask;
        dma_channel_hw_t * const p_channel = &dma_hw->ch[(uint)s_dma_chan];
        if ((p_channel->ctrl_trig & DMA_CH0_CTRL_TRIG_AHB_ERROR_BITS) != 0U)
        {
            s_dma_fault = true;
        }
        else
        {
            s_completed_bytes += HAL_UART_DMA_BLOCK_BYTES;
            /* Preserve the hardware ring write address, including blocks spanning its end. */
            p_channel->al1_transfer_count_trig = HAL_UART_DMA_BLOCK_BYTES;
        }
    }
}
#else
bool Hal_Uart_TestInjectRx(const uint8_t * const p_data, uint32_t len)
{
    if (!s_initialized || (p_data == NULL) || s_dma_fault)
    {
        return false;
    }
    for (uint32_t i = 0U; i < len; ++i)
    {
        s_rx_buffer[s_host_produced % HAL_UART_DMA_RING_BYTES] = p_data[i];
        ++s_host_produced;
        ++s_host_block_fill;
        if (s_host_block_fill == HAL_UART_DMA_BLOCK_BYTES)
        {
            s_completed_bytes += HAL_UART_DMA_BLOCK_BYTES;
            s_host_block_fill = 0U;
        }
    }
    return true;
}

void Hal_Uart_TestSetProduced(uint32_t count)
{
    s_host_produced = count;
    s_completed_bytes = count;
    s_host_block_fill = 0U;
}
void Hal_Uart_TestSetDmaFault(bool fault) { s_dma_fault = fault; }
#endif
