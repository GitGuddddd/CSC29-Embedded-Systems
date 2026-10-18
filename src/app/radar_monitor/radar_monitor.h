/** @file radar_monitor.h
 *
 * @brief Glasses_Radar supervision: INIT, RUN and FAULT (statechart 6.6).
 *
 * @par
 * COPYRIGHT NOTICE: (c) 2026 Group 29. All rights reserved.
 *
 * Reference: Group 29 Week 6 design review, statechart 6.6 (top), FR7, FR11.
 *   INIT   send the multi-target config; after 3 failed tries go to FAULT.
 *   RUN    wait for frames; no valid frame for 500 ms goes to FAULT.
 *   FAULT  RADAR_FAULT was sent. Keep listening for 2 s: if valid frames
 *          resume (the sensor often pauses by itself, bench finding), send
 *          RADAR_OK and go back to RUN without a new config. If nothing
 *          arrives, retry the config every 2 s.
 *
 * The component does no I/O itself. The task asks it what to do next
 * (radar_monitor_get_action), does that, and reports the result back.
 */

#ifndef RADAR_MONITOR_H
#define RADAR_MONITOR_H

#include <stdbool.h>
#include <stdint.h>

#include "rte_type.h"

/** State of the supervision state machine. */
typedef enum
{
    RADAR_MONITOR_STATE_INIT  = 0,
    RADAR_MONITOR_STATE_RUN   = 1,
    RADAR_MONITOR_STATE_FAULT = 2
} radar_monitor_state_t;

/** What the task has to do next. */
typedef enum
{
    RADAR_MONITOR_ACTION_CONFIGURE  = 0,  /**< Try the sensor config. */
    RADAR_MONITOR_ACTION_RECEIVE    = 1,  /**< Wait for a frame. */
    RADAR_MONITOR_ACTION_WAIT_RETRY = 2   /**< Sleep, then report done. */
} radar_monitor_action_t;

/** Supervision state. */
typedef struct
{
    radar_monitor_state_t state;
    bool                  b_delay_pending;
    bool                  b_fault_reported;
    bool                  b_listening;
    uint32_t              failed_tries;
    uint32_t              deadline_ms;
} radar_monitor_t;

void radar_monitor_init (radar_monitor_t * p_monitor);
radar_monitor_action_t radar_monitor_get_action (
    const radar_monitor_t * p_monitor);
uint32_t radar_monitor_get_deadline_ms (const radar_monitor_t * p_monitor);

void radar_monitor_on_config_result (radar_monitor_t * p_monitor, bool b_ok,
                                     uint32_t now_ms);
void radar_monitor_on_retry_done (radar_monitor_t * p_monitor);
void radar_monitor_on_frame (radar_monitor_t * p_monitor,
                             const rte_radar_target_t * p_targets,
                             uint32_t now_ms);
void radar_monitor_on_timeout (radar_monitor_t * p_monitor, uint32_t now_ms);

#endif /* RADAR_MONITOR_H */

/*** end of file ***/
