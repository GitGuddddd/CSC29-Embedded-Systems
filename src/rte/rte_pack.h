/** @file rte_pack.h
 *
 * @brief Bit-level packing of the radar queue message (minimum-bit change).
 *
 * @par
 * COPYRIGHT NOTICE: (c) 2026 Group 29. All rights reserved.
 *
 * Layout: docs/radar_message_layout.md. Bit n is bit (n % 8) of byte (n / 8),
 * least significant bit first.
 */

#ifndef RTE_PACK_H
#define RTE_PACK_H

#include <stdint.h>

#include "rte_type.h"

/** Distance step in the message: 2 mm. */
#define RTE_PACK_DISTANCE_STEP_MM    (2u)

/** Distance field width and its largest value (saturation), in 2 mm units. */
#define RTE_PACK_DISTANCE_BITS       (12u)
#define RTE_PACK_DISTANCE_MAX        (4095u)

/**
 * Speed field width (cm/s, two's complement). Settled by the speed capture
 * (assumption A7). Change this one constant to change the width.
 */
#define RTE_PACK_SPEED_BITS          (9u)

/** Largest speed magnitude that is sent; larger values are clamped. */
#define RTE_PACK_SPEED_MAX_CMS       ((1 << (RTE_PACK_SPEED_BITS - 1u)) - 1)

void rte_pack_put_bits (uint8_t * p_buf, uint32_t offset, uint32_t width,
                   uint32_t value);
uint32_t rte_pack_get_bits (const uint8_t * p_buf, uint32_t offset,
                            uint32_t width);

void rte_pack_radar (const rte_radar_data_t * p_data, rte_radar_msg_t * p_msg);
void rte_pack_unpack_radar (const rte_radar_msg_t * p_msg,
                       rte_radar_data_t * p_data);

#endif /* RTE_PACK_H */

/*** end of file ***/
