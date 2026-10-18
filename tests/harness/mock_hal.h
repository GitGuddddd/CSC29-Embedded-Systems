/** @file mock_hal.h
 *
 * @brief Test control of the mocked HAL, ld2450 receive path and RTE.
 *
 * @par
 * COPYRIGHT NOTICE: (c) 2026 Group 29. All rights reserved.
 */

#ifndef MOCK_HAL_H
#define MOCK_HAL_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "rte_type.h"

#define MOCK_RX_CAPACITY      (4096u)
#define MOCK_LOG_CAPACITY     (256u)

// Fake clock (hal_time) and byte source (ld2450_irq).
void mock_reset (void);
void mock_rte_reset (void);
void mock_time_set_ms (uint32_t now_ms);
void mock_time_advance_ms (uint32_t delta_ms);
void mock_rx_push (const uint8_t * p_data, size_t length);
size_t mock_rx_pending (void);
void mock_rx_set_ack_on_write (bool b_enable);
size_t mock_tx_count (void);

// Messages captured by the mock RTE.
size_t mock_audio_count (void);
rte_audio_msg_t mock_audio_get (size_t index);
size_t mock_ble_count (void);
rte_ble_obstacle_t mock_ble_get (size_t index);
size_t mock_radar_frame_count (void);
size_t mock_radar_fault_count (void);
rte_radar_msg_t mock_radar_get (size_t index);

#endif /* MOCK_HAL_H */

/*** end of file ***/
