#include "Ld2450.h"
#include "Ld2450_Cfg.h"
#include "hal/Hal_Uart.h"
#include "hal/Hal_Uart_Cfg.h"
#include <stddef.h>
#include <string.h>

/*
 * \file Ld2450_Stream.c
 * \brief Task-level DMA stream consumer. The small completion ISR belongs to Hal_Uart.
 */

/* RP2040 DMA ring wrapping is on address bits, not relative to the buffer pointer. */
_Alignas(LD2450_CFG_CIRC_BUF_SIZE) static volatile uint8_t s_circ_buf[LD2450_CFG_CIRC_BUF_SIZE];
static uint32_t s_consumed;
static uint32_t s_overruns;
static uint32_t s_last_poll_ms;
static bool s_have_poll;
static bool s_rx_fault;
static bool s_initialized;

_Static_assert(LD2450_CFG_CIRC_BUF_SIZE == HAL_UART_DMA_RING_BYTES, "Radar/HAL ring mismatch");
_Static_assert(HAL_UART_DMA_BLOCK_BYTES == LD2450_CFG_FRAME_BYTES, "DMA commit block must match a radar frame");
_Static_assert(LD2450_CFG_PARSE_BUDGET_BYTES >= LD2450_CFG_FRAME_BYTES, "Parser budget cannot hold a frame");
#define RX_SAFE_CAPACITY (LD2450_CFG_CIRC_BUF_SIZE - HAL_UART_DMA_BLOCK_BYTES)

bool Ld2450_Init(const Ld2450_Config_t * const p_cfg)
{
    if ((p_cfg == NULL) || (p_cfg->uart_port > 1U))
    {
        return false;
    }
    Hal_Uart_Init(p_cfg->uart_port, p_cfg->tx_pin, p_cfg->rx_pin, LD2450_CFG_BAUDRATE);
    s_initialized = Hal_Uart_DmaInit(p_cfg->uart_port, s_circ_buf, sizeof(s_circ_buf));
    s_consumed = 0U;
    s_overruns = 0U;
    s_have_poll = false;
    s_rx_fault = true; /* Healthy only after a complete frame, not merely DMA setup. */
    return s_initialized;
}

static void Drop_Backlog(uint32_t produced)
{
    s_consumed = produced;
    s_rx_fault = true;
    if (s_overruns != UINT32_MAX)
    {
        ++s_overruns;
    }
}

bool Ld2450_ProcessStream(Radar_Target_t * const p_targets, uint8_t * const p_count, uint32_t now_ms)
{
    if ((p_targets == NULL) || (p_count == NULL))
    {
        return false;
    }
    *p_count = 0U;
    Hal_Uart_RxSnapshot_t rx;
    if (!s_initialized || !Hal_Uart_GetRxSnapshot(&rx) || rx.fault)
    {
        s_rx_fault = true;
        return false;
    }
    bool stale_poll = s_have_poll && ((uint32_t)(now_ms - s_last_poll_ms) > LD2450_CFG_MAX_POLL_GAP_MS);
    s_last_poll_ms = now_ms;
    s_have_poll = true;
    uint32_t available = rx.produced_bytes - s_consumed;
    if (stale_poll || (available > RX_SAFE_CAPACITY))
    {
        /* Reserve a block for writes still in flight. Committed indices alone cannot
         * detect the active DMA block beginning to overwrite the oldest unread bytes.
         */
        Drop_Backlog(rx.produced_bytes);
        return false;
    }

    bool found = false;
    uint32_t examined = 0U;
    while ((available >= LD2450_CFG_FRAME_BYTES) &&
           (examined <= (LD2450_CFG_PARSE_BUDGET_BYTES - LD2450_CFG_FRAME_BYTES)))
    {
        uint8_t frame[LD2450_CFG_FRAME_BYTES];
        for (uint32_t i = 0U; i < sizeof(frame); ++i)
        {
            frame[i] = s_circ_buf[(s_consumed + i) % LD2450_CFG_CIRC_BUF_SIZE];
        }
        Hal_Uart_RxSnapshot_t after;
        bool snapshot_ok = Hal_Uart_GetRxSnapshot(&after);
        if (!snapshot_ok || after.fault ||
            ((uint32_t)(after.produced_bytes - s_consumed) > RX_SAFE_CAPACITY))
        {
            if (snapshot_ok)
            {
                Drop_Backlog(after.produced_bytes);
            }
            s_rx_fault = true;
            *p_count = 0U;
            return false; /* DMA may have overwritten this frame during the copy. */
        }

        Radar_Target_t targets[LD2450_CFG_MAX_TARGETS];
        uint8_t count = 0U;
        uint32_t consumed = 1U;
        if (Ld2450_ParseFrame(frame, sizeof(frame), targets, &count))
        {
            memcpy(p_targets, targets, (size_t)count * sizeof(targets[0]));
            *p_count = count;
            found = true;
            s_rx_fault = false;
            consumed = LD2450_CFG_FRAME_BYTES;
        }
        s_consumed += consumed;
        examined += consumed;
        available -= consumed;
    }
    return found;
}

bool Ld2450_HasRxFault(void)
{
    return !s_initialized || s_rx_fault;
}

uint32_t Ld2450_GetOverrunCount(void)
{
    return s_overruns;
}
