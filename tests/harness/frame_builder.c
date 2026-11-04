/** @file frame_builder.c
 *
 * @brief Builds LD2450 frames for the host tests.
 *
 * @par
 * COPYRIGHT NOTICE: (c) 2026 Group 29. All rights reserved.
 */

#include <stddef.h>
#include <stdint.h>

#include "frame_builder.h"

#define FB_NUM_TARGETS    (3u)
#define FB_BLOCK_LEN      (8u)
#define FB_HEADER_LEN     (4u)
#define FB_SIGN_BIT       (0x8000u)

/*!
 * @brief Encode a value as LD2450 sign-magnitude, little endian.
 *
 * @param[in]  value   Value in the range -32767 to 32767.
 * @param[out] p_low   Low byte.
 * @param[out] p_high  High byte.
 */
void
frame_builder_encode_signed (int16_t value, uint8_t * p_low, uint8_t * p_high)
{
    int32_t wide = value;
    uint32_t magnitude = (uint32_t) ((wide < 0) ? -wide : wide);
    uint32_t raw = magnitude | ((wide < 0) ? 0u : FB_SIGN_BIT);

    // Test code: the masks keep each byte within 8 bits.
    *p_low = (uint8_t) (raw & 0xFFu);
    *p_high = (uint8_t) ((raw >> 8u) & 0xFFu);
}

/*!
 * @brief Build a complete 30-byte frame.
 *
 * @param[out] p_out      Receives FRAME_BUILDER_FRAME_LEN bytes.
 * @param[in]  p_targets  Three targets.
 */
void
frame_builder_build (uint8_t * p_out, const frame_builder_target_t * p_targets)
{
    static const uint8_t header[FB_HEADER_LEN] = { 0xAAu, 0xFFu, 0x03u, 0x00u };
    size_t pos = 0u;

    for (size_t idx = 0u; idx < FB_HEADER_LEN; idx++)
    {
        p_out[pos] = header[idx];
        pos++;
    }

    for (size_t idx = 0u; idx < FB_NUM_TARGETS; idx++)
    {
        frame_builder_encode_signed(p_targets[idx].x_mm, &p_out[pos],
                                    &p_out[pos + 1u]);
        frame_builder_encode_signed(p_targets[idx].y_mm, &p_out[pos + 2u],
                                    &p_out[pos + 3u]);
        frame_builder_encode_signed(p_targets[idx].speed_cms, &p_out[pos + 4u],
                                    &p_out[pos + 5u]);

        // Resolution 360 mm, like the real sensor reports.
        p_out[pos + 6u] = 0x68u;
        p_out[pos + 7u] = 0x01u;
        pos += FB_BLOCK_LEN;
    }

    p_out[pos] = 0x55u;
    p_out[pos + 1u] = 0xCCu;
}

/*!
 * @brief Build a frame with one target in slot 0 and two empty slots.
 *
 * @param[out] p_out      Receives FRAME_BUILDER_FRAME_LEN bytes.
 * @param[in]  x_mm       X position.
 * @param[in]  y_mm       Y position.
 * @param[in]  speed_cms  Speed in cm/s.
 */
void
frame_builder_build_one (uint8_t * p_out, int16_t x_mm, int16_t y_mm,
                         int16_t speed_cms)
{
    frame_builder_target_t targets[FB_NUM_TARGETS] = { { 0, 0, 0 },
                                                       { 0, 0, 0 },
                                                       { 0, 0, 0 } };

    targets[0].x_mm = x_mm;
    targets[0].y_mm = y_mm;
    targets[0].speed_cms = speed_cms;
    frame_builder_build(p_out, targets);
}

/*** end of file ***/
