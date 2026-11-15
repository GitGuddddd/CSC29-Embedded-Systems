/** @file threat_assessment.h
 *
 * @brief Glasses obstacle decision: per-lane occupancy, hysteresis.
 *
 * @par
 * COPYRIGHT NOTICE: (c) 2026 Group 29. All rights reserved.
 *
 * Reference: Group 29 Week 6 design review, statechart 6.6 (bottom).
 *   FR8   each of the 3 lanes (left/middle/right, from the target's
 *         RTE_SIDE_* classification in radar_monitor.c) becomes BLOCKED
 *         once a valid target inside it is within THREAT_ASSESSMENT_
 *         START_MM, independent of the other lanes; the alert is active
 *         whenever any lane is BLOCKED (OPEN-22, 13 Nov; was single-target
 *         MIO selection, OPEN-16 through OPEN-21).
 *   FR9   while active, OBSTACLE(d_mm, cue) goes to Audio Alert on every
 *         frame. cue's direction bits mean "which way is open" (CENTER/
 *         LEFT/RIGHT/STOP), from the 8-row truth table over the three
 *         lanes' BLOCKED state (threat_assessment_direction_cue(), OPEN-20,
 *         unchanged by OPEN-22). d_mm is the nearest distance among the
 *         currently blocked lanes.
 *   FR10  OBSTACLE(CLEAR) once every lane clears (a lane clears once every
 *         valid target in it is beyond THREAT_ASSESSMENT_STOP_MM, or it
 *         reads empty, for THREAT_ASSESSMENT_LANE_HOLD_MS in a row), or on
 *         RADAR_FAULT.
 *   FR13  Restored (OPEN-23, 13 Nov), then reported per-lane rather than
 *         for a single picked lane (OPEN-25, 14 Nov): lane_ttc_ms[side]
 *         is the TTC for that lane,
 *         computed from its own live, per-frame speed_cms this frame
 *         (threat_assessment_closing_cms(), threat_assessment_ttc_ms()) -
 *         never from frame-to-frame position tracking, which is the
 *         specific hazard OPEN-17 removed (see docs/deviations.md OPEN-17,
 *         OPEN-23, OPEN-25). A lane's ttc_ms stays RTE_TTC_NONE_MS
 *         whenever that lane is not BLOCKED, has no live reading this
 *         frame (it is only BLOCKED courtesy of its THREAT_ASSESSMENT_
 *         LANE_HOLD_MS grace window), or is not approaching. OPEN-25:
 *         the single-lane scoping OPEN-23 used (whichever lane drives
 *         d_mm) was dropped because the nearest lane by distance is not
 *         always the one closing in soonest - a farther, faster-closing
 *         lane's TTC would otherwise never reach BLE at all.
 *   FR6   grip state from the cane: HELD and UNKNOWN keep the alert active,
 *         only RELEASED silences it (and ends an active alert with one
 *         OBSTACLE(CLEAR)). The state starts as UNKNOWN. Unchanged by
 *         OPEN-22.
 *
 * OPEN-22 (13 Nov): removed entirely - MIO (most important object) target
 * selection and its tie-break (OPEN-16), 2D collision-course prediction
 * (OPEN-17), persistent frame-to-frame track IDs (OPEN-18), the [MIO]
 * bench-debug line (OPEN-19), and the multi-frame departing-target range/
 * speed trend filters and the away-jump gate (all pre-OPEN-16). Every
 * target is now evaluated into its lane every frame; nothing is excluded
 * for moving away, and nothing is selected as "the" obstacle. See
 * docs/deviations.md OPEN-22 for the full reasoning.
 */

#ifndef THREAT_ASSESSMENT_H
#define THREAT_ASSESSMENT_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "rte_type.h"

/** One lane's (left/middle/right) BLOCKED state and hold timer (OPEN-22). */
typedef struct
{
    bool     b_blocked;       /**< Debounced state; drives the FR9 cue. */
    uint32_t last_d_mm;       /**< Nearest distance last seen in this lane;
                                    reported while b_blocked, including
                                    during the hold grace period. */
    uint32_t hold_pending_ms; /**< Time this lane first read clear/empty
                                    while still blocked, or UINT32_MAX when
                                    no clear is pending. */
} threat_assessment_lane_t;

/** State of Glasses_Threat: CLEAR (b_active false) or OBSTACLE_ACTIVE. */
typedef struct
{
    bool     b_active;
    rte_grip_state_t grip;   /**< Last grip state; kept across a fault. */

    // OPEN-22: one lane per side, independent hysteresis and hold timer.
    // Replaces OPEN-20's bare b_left_blocked/b_middle_blocked/
    // b_right_blocked bools (no hold timer, no per-lane last-known
    // distance) now that the per-lane hold has to outlive the frame in
    // which a target stops being seen.
    threat_assessment_lane_t left;
    threat_assessment_lane_t middle;
    threat_assessment_lane_t right;
} threat_assessment_t;

/** What the caller must send after one threat_assessment_update(). */
typedef struct
{
    bool     b_post_obstacle;  /**< Send OBSTACLE(d_mm, cue) (every frame). */
    bool     b_post_clear;     /**< Send OBSTACLE(CLEAR) (once per exit). */
    bool     b_active;         /**< State after this update. */
    uint16_t d_mm;             /**< Distance for OBSTACLE(d_mm): nearest
                                     among the currently blocked lanes.
                                     FR9 audio only - BLE uses lane_d_mm
                                     below instead (OPEN-25). */
    uint8_t  cue;              /**< FR9 cue byte (direction, OPEN-20); 0
                                     unless b_post_obstacle. */
    uint16_t lane_d_mm[3];     /**< Per lane (RTE_SIDE_* index) distance
                                     for BLE; RTE_LANE_NONE_MM for a lane
                                     that is not BLOCKED (OPEN-25). */
    uint16_t lane_ttc_ms[3];   /**< Per lane TTC for BLE, from that lane's
                                     own live speed_cms this frame;
                                     RTE_TTC_NONE_MS if that lane is not
                                     BLOCKED, has no live reading this
                                     frame, or isn't approaching (OPEN-23,
                                     OPEN-25, FR13 - see
                                     docs/deviations.md). */
} threat_assessment_out_t;

/** Link-time configuration (defined in threat_assessment_lcfg.c). */
typedef struct
{
    /** Assumption A7: negative speed means approaching the sensor. Used by
     *  threat_assessment_closing_cms(), which OPEN-23 calls again (via
     *  threat_assessment_scan_lanes()) for the restored, scoped TTC - see
     *  docs/deviations.md OPEN-23. */
    bool b_negative_speed_is_approaching;
} threat_assessment_lcfg_t;

extern const threat_assessment_lcfg_t g_threat_assessment_lcfg;

void threat_assessment_init (threat_assessment_t * p_state);
bool threat_assessment_reset (threat_assessment_t * p_state);
void threat_assessment_update (threat_assessment_t * p_state,
                               const rte_radar_target_t * p_targets,
                               uint32_t now_ms,
                               threat_assessment_out_t * p_out);

void threat_assessment_process_frame (threat_assessment_t * p_state,
                                      const rte_radar_target_t * p_targets,
                                      uint32_t now_ms);
void threat_assessment_process_fault (threat_assessment_t * p_state);

bool threat_assessment_set_grip (threat_assessment_t * p_state,
                                 rte_grip_state_t grip);
void threat_assessment_process_grip (threat_assessment_t * p_state,
                                     rte_grip_state_t grip);

uint16_t threat_assessment_distance_mm (const rte_radar_target_t * p_target);
int32_t threat_assessment_closing_cms (const rte_radar_target_t * p_target);
uint16_t threat_assessment_ttc_ms (uint32_t d_mm, int32_t closing_cms);

#endif /* THREAT_ASSESSMENT_H */

/*** end of file ***/
