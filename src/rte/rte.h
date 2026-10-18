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
 */

#ifndef RTE_H
#define RTE_H

#include <stdbool.h>
#include <stdint.h>

#include "rte_type.h"

bool rte_init (void);
bool rte_start (void);

void rte_write_radar_frame (const rte_radar_target_t * p_targets,
                            uint32_t time_ms);
void rte_write_radar_fault (void);
void rte_write_audio_msg (rte_audio_msg_type_t type, uint16_t d_mm);
void rte_write_ble_obstacle (bool b_active, uint16_t d_mm, uint16_t ttc_ms);

bool rte_read_audio_msg (rte_audio_msg_t * p_msg, uint32_t timeout_ms);
bool rte_read_ble_obstacle (rte_ble_obstacle_t * p_msg, uint32_t timeout_ms);

#endif /* RTE_H */

/*** end of file ***/
