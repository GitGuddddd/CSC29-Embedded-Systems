/** @file rte.h
 *
 * @brief Runtime environment of the glasses perception components.
 *
 * @par
 * COPYRIGHT NOTICE: (c) 2026 Group 29. All rights reserved.
 *
 * The application components (radar_monitor, threat_assessment) call only
 * the rte_write_* functions below. The RTE maps them to RTOS queues and hosts
 * the two tasks. The Audio Alert and BLE owners call the rte_read_*
 * functions. Data flow (report, statechart 6.6):
 *
 *   UART0 RX interrupt -> stream buffer -> Glasses_Radar (radar_monitor)
 *     -> radar queue -> Glasses_Threat (threat_assessment)
 *     -> audio queue (OBSTACLE, CLEAR, RADAR_FAULT, RADAR_OK) and BLE queue
 *
 * The Glasses BLE task calls rte_write_grip_state() with HELD or RELEASED
 * from each cane packet, and with UNKNOWN when no cane packet has arrived
 * for 500 ms. It joins the radar queue so Glasses_Threat blocks in one place.
 */

#ifndef RTE_H
#define RTE_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "rte_type.h"

bool rte_init (void);
bool rte_start (void);

void rte_write_radar_frame (const rte_radar_target_t * p_targets,
                            uint32_t time_ms);
void rte_write_radar_fault (void);
void rte_write_grip_state (rte_grip_state_t grip);
void rte_write_audio_msg (rte_audio_msg_type_t type, uint16_t d_mm,
                          uint8_t cue);
void rte_write_ble_obstacle (bool b_active, uint16_t d_mm, uint16_t ttc_ms);

// Bench-debug only (OPEN-19, FEATURE_DEBUG_LOG_MIO): which target the MIO
// selection picked this frame, and why. No-op unless that flag is 1. Never
// read back by anything, unlike the rte_write_* functions above - this is
// a print, not a queued message.
void rte_write_mio_debug (bool b_present, size_t track_slot, uint32_t d_mm,
                          uint32_t risk_mm, bool b_collision);

// Bench-debug only (OPEN-20, same FEATURE_DEBUG_LOG_MIO flag as above): a
// plain-language line for the new FR9 direction cue - which sides are
// occupied this frame (independent of b_active: occupancy is still useful
// to see even while no alert is active) and the resulting "go left"/"stay
// middle"/"go right"/"STOP" decision, or "no alert" when b_active is false
// (cue is not meaningful then) - so the cue byte does not have to be
// decoded by hand on the bench.
void rte_write_dir_debug (bool b_left_blocked, bool b_middle_blocked,
                          bool b_right_blocked, bool b_active, uint8_t cue);

bool rte_read_audio_msg (rte_audio_msg_t * p_msg, uint32_t timeout_ms);
bool rte_read_ble_obstacle (rte_ble_obstacle_t * p_msg, uint32_t timeout_ms);

#endif /* RTE_H */

/*** end of file ***/
