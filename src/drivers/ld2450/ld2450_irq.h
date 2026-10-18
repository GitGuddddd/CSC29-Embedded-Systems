/** @file ld2450_irq.h
 *
 * @brief Private interface between ld2450.c and the receive interrupt.
 *
 * @par
 * COPYRIGHT NOTICE: (c) 2026 Group 29. All rights reserved.
 *
 * NOTE: Private to the LD2450 driver. Do not include from other modules.
 */

#ifndef LD2450_IRQ_H
#define LD2450_IRQ_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "ld2450.h"

bool ld2450_irq_init (void);
size_t ld2450_irq_receive (uint8_t * p_dst, size_t max_length,
                           uint32_t timeout_ms);
void ld2450_irq_flush (void);
uint32_t ld2450_irq_get_overflow_count (void);
void ld2450_irq_get_stats (ld2450_rx_stats_t * p_stats);

#endif /* LD2450_IRQ_H */

/*** end of file ***/
