/** @file mock_hal.c
 *
 * @brief Host doubles for hal_time, hal_uart and the ld2450 receive path.
 *
 * @par
 * COPYRIGHT NOTICE: (c) 2026 Group 29. All rights reserved.
 *
 * The fake receive path replaces ld2450_irq.c. A read that finds no byte
 * advances the fake clock by the requested timeout, so deadline logic runs
 * without real waiting.
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "hal_time.h"
#include "hal_uart.h"
#include "ld2450_irq.h"
#include "mock_hal.h"

static uint32_t g_now_ms = 0u;
static uint8_t g_rx_buffer[MOCK_RX_CAPACITY];
static size_t g_rx_head = 0u;
static size_t g_rx_tail = 0u;
static bool gb_ack_on_write = false;
static size_t g_tx_count = 0u;

static const uint8_t g_ack_bytes[] =
{
    0xFDu, 0xFCu, 0xFBu, 0xFAu, 0x04u, 0x03u, 0x02u, 0x01u
};

/*!
 * @brief Clear all mock state.
 */
void
mock_reset (void)
{
    g_now_ms = 0u;
    g_rx_head = 0u;
    g_rx_tail = 0u;
    gb_ack_on_write = false;
    g_tx_count = 0u;
}

/*!
 * @brief Set the fake clock.
 *
 * @param[in] now_ms  New time in milliseconds.
 */
void
mock_time_set_ms (uint32_t now_ms)
{
    g_now_ms = now_ms;
}

/*!
 * @brief Move the fake clock forward.
 *
 * @param[in] delta_ms  Milliseconds to add.
 */
void
mock_time_advance_ms (uint32_t delta_ms)
{
    g_now_ms += delta_ms;
}

/*!
 * @brief Queue bytes as if the sensor had sent them.
 *
 * @param[in] p_data  Bytes to queue.
 * @param[in] length  Number of bytes.
 */
void
mock_rx_push (const uint8_t * p_data, size_t length)
{
    for (size_t idx = 0u; idx < length; idx++)
    {
        if (g_rx_head < MOCK_RX_CAPACITY)
        {
            g_rx_buffer[g_rx_head] = p_data[idx];
            g_rx_head++;
        }
    }
}

/*!
 * @brief Bytes still waiting to be read.
 *
 * @return Number of unread bytes.
 */
size_t
mock_rx_pending (void)
{
    return (g_rx_head - g_rx_tail);
}

/*!
 * @brief Make every UART write answer with an acknowledgement frame.
 *
 * @param[in] b_enable  true to acknowledge.
 */
void
mock_rx_set_ack_on_write (bool b_enable)
{
    gb_ack_on_write = b_enable;
}

/*!
 * @brief Number of UART writes seen.
 *
 * @return Count of hal_uart_write calls.
 */
size_t
mock_tx_count (void)
{
    return (g_tx_count);
}

uint32_t
hal_time_now_ms (void)
{
    return (g_now_ms);
}

bool
hal_uart_init (hal_uart_id_t id, uint32_t baud_rate, uint8_t tx_pin,
               uint8_t rx_pin)
{
    // The mock ignores the settings; they are checked on the target.
    (void) id;
    (void) baud_rate;
    (void) tx_pin;
    (void) rx_pin;

    return (true);
}

void
hal_uart_write (hal_uart_id_t id, const uint8_t * p_data, size_t length)
{
    // The mock only counts writes and optionally answers with an ack.
    (void) id;
    (void) p_data;
    (void) length;
    g_tx_count++;

    if (gb_ack_on_write)
    {
        mock_rx_push(g_ack_bytes, sizeof(g_ack_bytes));
    }
}

bool
ld2450_irq_init (void)
{
    return (true);
}

size_t
ld2450_irq_receive (uint8_t * p_dst, size_t max_length, uint32_t timeout_ms)
{
    size_t count = 0u;

    while ((count < max_length) && (g_rx_tail < g_rx_head))
    {
        p_dst[count] = g_rx_buffer[g_rx_tail];
        g_rx_tail++;
        count++;
    }

    if (0u == count)
    {
        // Nothing to read: the wait runs out, so the fake clock moves on.
        g_now_ms += timeout_ms;
    }

    return (count);
}

void
ld2450_irq_flush (void)
{
    g_rx_tail = g_rx_head;
}

uint32_t
ld2450_irq_get_overflow_count (void)
{
    return (0u);
}

void
ld2450_irq_get_stats (ld2450_rx_stats_t * p_stats)
{
    p_stats->overrun_count = 0u;
    p_stats->dropped_bytes = 0u;
    p_stats->byte_count = 0u;
    p_stats->isr_count = 0u;
}

/*** end of file ***/
