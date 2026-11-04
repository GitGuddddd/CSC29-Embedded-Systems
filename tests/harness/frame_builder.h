/** @file frame_builder.h
 *
 * @brief Builds LD2450 frames for the host tests.
 *
 * @par
 * COPYRIGHT NOTICE: (c) 2026 Group 29. All rights reserved.
 */

#ifndef FRAME_BUILDER_H
#define FRAME_BUILDER_H

#include <stddef.h>
#include <stdint.h>

/** Length of a complete frame: header 4 + body 26. */
#define FRAME_BUILDER_FRAME_LEN    (30u)

/** One target as the test wants to send it. All zero = empty slot. */
typedef struct
{
    int16_t x_mm;
    int16_t y_mm;
    int16_t speed_cms;
} frame_builder_target_t;

void frame_builder_encode_signed (int16_t value, uint8_t * p_low,
                                  uint8_t * p_high);
void frame_builder_build (uint8_t * p_out,
                          const frame_builder_target_t * p_targets);
void frame_builder_build_one (uint8_t * p_out, int16_t x_mm, int16_t y_mm,
                              int16_t speed_cms);

#endif /* FRAME_BUILDER_H */

/*** end of file ***/
