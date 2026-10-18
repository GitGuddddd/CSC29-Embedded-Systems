/** @file ld2450_irq.c
 *
 * @brief UART0 receive interrupt and stream buffer of the LD2450 driver.
 *
 * @par
 * COPYRIGHT NOTICE: (c) 2026 Group 29. All rights reserved.
 *
 * The interrupt only moves bytes from the 32-byte hardware FIFO into a
 * byte stream of the operating-system adapter. All parsing happens in the
 * Glasses_Radar task (NFR7). If the stream is full the bytes are dropped
 * and counted.
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "hal_uart.h"
#include "ld2450.h"
#include "ld2450_cfg.h"
#include "ld2450_irq.h"
#include "osal.h"

// Bytes dropped by the hardware FIFO or the stream buffer. Only the
// interrupt writes it. Tasks read it (a 32-bit read is atomic on the M0+).
static volatile uint32_t g_overflow_count = 0u;

// Diagnosis counters, written only by the interrupt.
static volatile uint32_t g_overrun_count = 0u;
static volatile uint32_t g_dropped_bytes = 0u;
static volatile uint32_t g_byte_count = 0u;
static volatile uint32_t g_isr_count = 0u;
static osal_stream_t g_rx_stream;

static void ld2450_rx_isr (void);

/*!
 * @brief Create the stream buffer, set up UART0 and enable the interrupt.
 *
 * @return true when everything was created.
 */
bool
ld2450_irq_init (void)
{
    bool b_ok = false;

    if (osal_stream_create(&g_rx_stream, LD2450_RX_BUFFER_BYTES))
    {
        b_ok = hal_uart_init(g_ld2450_lcfg.uart_id, LD2450_BAUD_RATE,
                             g_ld2450_lcfg.tx_pin, g_ld2450_lcfg.rx_pin);
    }
    else
    {
        b_ok = false;
    }

    if (b_ok)
    {
        hal_uart_set_rx_handler(g_ld2450_lcfg.uart_id, ld2450_rx_isr);
    }

    return (b_ok);
}

/*!
 * @brief Read received bytes, blocking until some arrive or time runs out.
 *
 * @param[out] p_dst       Destination buffer.
 * @param[in]  max_length  Size of the destination buffer.
 * @param[in]  timeout_ms  Longest time to wait, in milliseconds. 0 = poll.
 *
 * @return Number of bytes copied (0 on timeout).
 */
size_t
ld2450_irq_receive (uint8_t * p_dst, size_t max_length, uint32_t timeout_ms)
{
    return (osal_stream_receive(&g_rx_stream, p_dst, max_length, timeout_ms));
}

/*!
 * @brief Discard everything in the stream buffer without resetting it.
 *
 * NOTE: The interrupt may be writing, so the buffer is read out, not reset.
 */
void
ld2450_irq_flush (void)
{
    uint8_t scratch[LD2450_ISR_CHUNK_BYTES];
    size_t count = 0u;

    do
    {
        count = osal_stream_receive(&g_rx_stream, scratch, sizeof(scratch),
                                    0u);
    } while (count > 0u);
}

/*!
 * @brief Number of bytes lost since start-up (must stay 0, NFR7).
 *
 * @return The count of dropped bytes.
 */
uint32_t
ld2450_irq_get_overflow_count (void)
{
    return (g_overflow_count);
}

/*!
 * @brief Copy the diagnosis counters.
 *
 * @param[out] p_stats  Receives the counters.
 */
void
ld2450_irq_get_stats (ld2450_rx_stats_t * p_stats)
{
    p_stats->overrun_count = g_overrun_count;
    p_stats->dropped_bytes = g_dropped_bytes;
    p_stats->byte_count = g_byte_count;
    p_stats->isr_count = g_isr_count;
}

/*!
 * @brief UART receive interrupt: copy the FIFO into the stream buffer.
 *
 * WARNING: Runs in interrupt context. Call only interrupt-safe adapter
 * functions (osal_stream_send_from_isr).
 */
static void
ld2450_rx_isr (void)
{
    uint8_t chunk[LD2450_ISR_CHUNK_BYTES];
    size_t count = 0u;

    g_isr_count++;

    if (hal_uart_take_overrun(g_ld2450_lcfg.uart_id))
    {
        g_overflow_count++;
        g_overrun_count++;
    }

    do
    {
        count = 0u;

        while ((count < LD2450_ISR_CHUNK_BYTES) &&
               (hal_uart_is_readable(g_ld2450_lcfg.uart_id)))
        {
            chunk[count] = hal_uart_read_byte(g_ld2450_lcfg.uart_id);
            count++;
        }

        if (count > 0u)
        {
            size_t sent = osal_stream_send_from_isr(&g_rx_stream, chunk,
                                                    count);

            // The cast is safe: sent never exceeds count, so the
            // difference is not negative.
            g_overflow_count += (uint32_t) (count - sent);
            g_dropped_bytes += (uint32_t) (count - sent);
            g_byte_count += (uint32_t) count;
        }
    } while (count > 0u);
}

/*** end of file ***/
