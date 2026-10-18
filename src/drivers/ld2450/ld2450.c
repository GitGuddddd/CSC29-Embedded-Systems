/** @file ld2450.c
 *
 * @brief HLK-LD2450 mmWave radar driver: configuration and frame parsing.
 *
 * @par
 * COPYRIGHT NOTICE: (c) 2026 Group 29. All rights reserved.
 *
 * The driver never blocks without a deadline. It reads bytes that the
 * receive interrupt put into a stream buffer (see ld2450_irq.c).
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "hal_time.h"
#include "hal_uart.h"
#include "ld2450.h"
#include "ld2450_cfg.h"
#include "ld2450_irq.h"
#include "rte_type.h"

#define LD2450_HEADER_LEN          (4u)
#define LD2450_FOOTER_LEN          (2u)
#define LD2450_ACK_MARKER_LEN      (4u)
#define LD2450_TARGET_BLOCK_LEN    (8u)

// Byte offsets inside one 8-byte target block.
#define LD2450_OFFSET_X            (0u)
#define LD2450_OFFSET_Y            (2u)
#define LD2450_OFFSET_SPEED        (4u)
#define LD2450_OFFSET_RESOLUTION   (6u)

// Sign-magnitude fields: bit 15 set means positive.
#define LD2450_SIGN_BIT            (0x8000u)
#define LD2450_MAGNITUDE_MASK      (0x7FFFu)
#define LD2450_BYTE_SHIFT          (8u)

// Largest time difference that still counts as "in the future".
#define LD2450_HALF_RANGE_MS       (0x7FFFFFFFu)

static const uint8_t g_frame_header[LD2450_HEADER_LEN] =
{
    0xAAu, 0xFFu, 0x03u, 0x00u
};

static const uint8_t g_frame_footer[LD2450_FOOTER_LEN] = { 0x55u, 0xCCu };

static const uint8_t g_ack_header[LD2450_ACK_MARKER_LEN] =
{
    0xFDu, 0xFCu, 0xFBu, 0xFAu
};

static const uint8_t g_ack_footer[LD2450_ACK_MARKER_LEN] =
{
    0x04u, 0x03u, 0x02u, 0x01u
};

static uint32_t g_footer_mismatch_count = 0u;

static uint32_t ld2450_remaining_ms (uint32_t deadline_ms);
static bool ld2450_wait_for_pattern (const uint8_t * p_pattern, size_t length,
                                     uint32_t deadline_ms);
static bool ld2450_read_exact (uint8_t * p_dst, size_t length,
                               uint32_t deadline_ms);
static bool ld2450_send_command (const ld2450_command_t * p_command);
static ld2450_frame_status_t ld2450_read_body (rte_radar_target_t * p_targets);

/*!
 * @brief Create the receive path (stream buffer, UART0, interrupt).
 *
 * @return true when the receive path is ready.
 */
bool
ld2450_init (void)
{
    g_footer_mismatch_count = 0u;

    return (ld2450_irq_init());
}

/*!
 * @brief Put the sensor into multi-target mode (three config commands).
 *
 * @return true when the sensor acknowledged every command.
 */
bool
ld2450_configure (void)
{
    bool b_ok = ld2450_send_command(&g_ld2450_lcfg.enable_config);

    if (b_ok)
    {
        b_ok = ld2450_send_command(&g_ld2450_lcfg.set_multi_target);
    }

    if (b_ok)
    {
        b_ok = ld2450_send_command(&g_ld2450_lcfg.end_config);
    }

    return (b_ok);
}

/*!
 * @brief Receive and decode one frame, resynchronising on the header.
 *
 * A short read or a bad footer is not an error for the caller: it simply
 * calls this function again (FR7: discard the frame and resync).
 *
 * @param[out] p_targets   Receives the three target slots on success.
 * @param[in]  deadline_ms Absolute time (hal_time_now_ms) to give up at.
 *
 * @return LD2450_FRAME_OK, or the reason no frame was produced.
 */
ld2450_frame_status_t
ld2450_receive_frame (rte_radar_target_t * p_targets, uint32_t deadline_ms)
{
    ld2450_frame_status_t status = LD2450_FRAME_TIMEOUT;

    if (ld2450_wait_for_pattern(g_frame_header, LD2450_HEADER_LEN,
                                deadline_ms))
    {
        status = ld2450_read_body(p_targets);
    }

    return (status);
}

/*!
 * @brief Decode one LD2450 sign-magnitude 16-bit value.
 *
 * @param[in] low   Low byte (sent first).
 * @param[in] high  High byte. Bit 7 set means positive.
 *
 * @return The signed value, in the unit of the field (mm or cm/s).
 */
int16_t
ld2450_decode_signed (uint8_t low, uint8_t high)
{
    // The casts widen unsigned bytes to uint32_t, which cannot lose bits.
    uint32_t raw = ((uint32_t) high << LD2450_BYTE_SHIFT) | (uint32_t) low;

    // The casts narrow values of at most 15 bits (magnitude) or a negated
    // 15-bit magnitude, so both always fit in int32_t and int16_t.
    int32_t magnitude = (int32_t) (raw & LD2450_MAGNITUDE_MASK);
    int32_t value = (0u != (raw & LD2450_SIGN_BIT)) ? magnitude : -magnitude;

    return ((int16_t) value);
}

/*!
 * @brief Decode the 26 bytes that follow the frame header.
 *
 * @param[in]  p_body     The 26 body bytes.
 * @param[out] p_targets  Three target slots. Untouched on a bad footer.
 *
 * @return true when the footer was 55 CC and the targets were decoded.
 */
bool
ld2450_decode_body (const uint8_t * p_body, rte_radar_target_t * p_targets)
{
    const size_t footer_at = LD2450_BODY_LEN - LD2450_FOOTER_LEN;
    bool b_ok = ((g_frame_footer[0] == p_body[footer_at]) &&
                 (g_frame_footer[1] == p_body[footer_at + 1u]));

    if (b_ok)
    {
        for (size_t idx = 0u; idx < RTE_RADAR_NUM_TARGETS; idx++)
        {
            const uint8_t * p_block = &p_body[idx * LD2450_TARGET_BLOCK_LEN];
            rte_radar_target_t * p_out = &p_targets[idx];

            p_out->x_mm = ld2450_decode_signed(p_block[LD2450_OFFSET_X],
                                               p_block[LD2450_OFFSET_X + 1u]);
            p_out->y_mm = ld2450_decode_signed(p_block[LD2450_OFFSET_Y],
                                               p_block[LD2450_OFFSET_Y + 1u]);
            p_out->speed_cms = ld2450_decode_signed(
                p_block[LD2450_OFFSET_SPEED],
                p_block[LD2450_OFFSET_SPEED + 1u]);

            // The cast narrows a 16-bit value built from two bytes.
            p_out->resolution_mm = (uint16_t) (
                ((uint32_t) p_block[LD2450_OFFSET_RESOLUTION + 1u]
                 << LD2450_BYTE_SHIFT) |
                (uint32_t) p_block[LD2450_OFFSET_RESOLUTION]);

            // An all-zero block is an empty slot.
            p_out->b_valid = !((0 == p_out->x_mm) && (0 == p_out->y_mm) &&
                               (0 == p_out->speed_cms));
        }
    }

    return (b_ok);
}

/*!
 * @brief Bytes lost in the receive path since start-up (must stay 0).
 *
 * @return The count of dropped bytes.
 */
uint32_t
ld2450_get_overflow_count (void)
{
    return (ld2450_irq_get_overflow_count());
}

/*!
 * @brief Copy the receive-path statistics (for the debug log).
 *
 * @param[out] p_stats  Receives the counters.
 */
void
ld2450_get_rx_stats (ld2450_rx_stats_t * p_stats)
{
    ld2450_irq_get_stats(p_stats);
}

/*!
 * @brief Frames discarded because the footer was not 55 CC.
 *
 * @return The count of discarded frames.
 */
uint32_t
ld2450_get_bad_footer_count (void)
{
    return (g_footer_mismatch_count);
}

/*!
 * @brief Milliseconds left until an absolute deadline (0 if passed).
 *
 * @param[in] deadline_ms  Deadline on the hal_time_now_ms time base.
 *
 * @return Remaining time in milliseconds.
 */
static uint32_t
ld2450_remaining_ms (uint32_t deadline_ms)
{
    // Unsigned subtraction wraps correctly when the 32-bit clock rolls over.
    uint32_t diff_ms = deadline_ms - hal_time_now_ms();

    return ((diff_ms > LD2450_HALF_RANGE_MS) ? 0u : diff_ms);
}

/*!
 * @brief Scan the byte stream for a fixed pattern (resyncs mid-frame).
 *
 * @param[in] p_pattern    Bytes to find.
 * @param[in] length       Length of the pattern.
 * @param[in] deadline_ms  Absolute time to give up at.
 *
 * @return true when the whole pattern was seen before the deadline.
 */
static bool
ld2450_wait_for_pattern (const uint8_t * p_pattern, size_t length,
                         uint32_t deadline_ms)
{
    size_t matched = 0u;
    bool b_found = false;
    bool b_timed_out = false;

    while ((!b_found) && (!b_timed_out))
    {
        uint8_t byte = 0u;

        if (0u == ld2450_irq_receive(&byte, 1u,
                                     ld2450_remaining_ms(deadline_ms)))
        {
            b_timed_out = true;
        }
        else
        {
            // NOTE: Restarting at 1 on the first pattern byte is correct for
            // the patterns used here, none of which repeat their first byte.
            if (byte == p_pattern[matched])
            {
                matched++;
            }
            else if (byte == p_pattern[0])
            {
                matched = 1u;
            }
            else
            {
                matched = 0u;
            }

            b_found = (matched >= length);
        }
    }

    return (b_found);
}

/*!
 * @brief Read exactly length bytes or give up at the deadline.
 *
 * @param[out] p_dst       Destination buffer.
 * @param[in]  length      Number of bytes wanted.
 * @param[in]  deadline_ms Absolute time to give up at.
 *
 * @return true when all bytes were read.
 */
static bool
ld2450_read_exact (uint8_t * p_dst, size_t length, uint32_t deadline_ms)
{
    size_t received = 0u;
    bool b_ok = true;

    while ((received < length) && b_ok)
    {
        size_t count = ld2450_irq_receive(&p_dst[received], length - received,
                                          ld2450_remaining_ms(deadline_ms));

        if (0u == count)
        {
            b_ok = false;
        }
        else
        {
            received += count;
        }
    }

    return (b_ok);
}

/*!
 * @brief Send one config command and wait for its acknowledgement frame.
 *
 * @param[in] p_command  Command bytes to send.
 *
 * @return true when the ack header and footer were seen in time.
 */
static bool
ld2450_send_command (const ld2450_command_t * p_command)
{
    bool b_ok = false;

    ld2450_irq_flush();
    hal_uart_write(g_ld2450_lcfg.uart_id, p_command->p_data,
                   p_command->length);

    b_ok = ld2450_wait_for_pattern(g_ack_header, LD2450_ACK_MARKER_LEN,
                                   hal_time_now_ms() + LD2450_ACK_TIMEOUT_MS);

    if (b_ok)
    {
        b_ok = ld2450_wait_for_pattern(
            g_ack_footer, LD2450_ACK_MARKER_LEN,
            hal_time_now_ms() + LD2450_ACK_TIMEOUT_MS);
    }

    return (b_ok);
}

/*!
 * @brief Read the 26 bytes after a found header and decode them.
 *
 * @param[out] p_targets  Receives the three target slots on success.
 *
 * @return LD2450_FRAME_OK, LD2450_FRAME_SHORT or LD2450_FRAME_BAD_FOOTER.
 */
static ld2450_frame_status_t
ld2450_read_body (rte_radar_target_t * p_targets)
{
    uint8_t body[LD2450_BODY_LEN];
    ld2450_frame_status_t status = LD2450_FRAME_SHORT;

    if (ld2450_read_exact(body, LD2450_BODY_LEN,
                          hal_time_now_ms() + LD2450_BODY_TIMEOUT_MS))
    {
        if (ld2450_decode_body(body, p_targets))
        {
            status = LD2450_FRAME_OK;
        }
        else
        {
            g_footer_mismatch_count++;
            status = LD2450_FRAME_BAD_FOOTER;
        }
    }

    return (status);
}

/*** end of file ***/
