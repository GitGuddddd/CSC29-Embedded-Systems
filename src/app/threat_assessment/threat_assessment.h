/** @file threat_assessment.h
 *
 * @brief Glasses obstacle decision: nearest target, hysteresis, TTC.
 *
 * @par
 * COPYRIGHT NOTICE: (c) 2026 Group 29. All rights reserved.
 *
 * Reference: Group 29 Week 6 design review, statechart 6.6 (bottom).
 *   FR8   nearest valid non-departing target within 2000 mm is an obstacle;
 *         the alert starts below 2000 mm and stops above 2100 mm (NFR8).
 *   FR9   while active, OBSTACLE(d_mm, cue) goes to Audio Alert on every
 *         frame. OPEN-20 (11 Nov): cue's direction bits now mean "which way
 *         is open" (CENTER/LEFT/RIGHT), from per-side corridor occupancy
 *         independent of which target is the MIO; STOP now means all three
 *         sides (left/middle/right) are occupied within THREAT_ASSESSMENT_
 *         START_MM/_STOP_MM, not a near-distance sub-cue on the single MIO
 *         target (see threat_assessment_direction_cue()).
 *   FR10  OBSTACLE(CLEAR) when the target is beyond 2100 mm, none was seen
 *         for 500 ms, the range trend showed the target leaving for 2 frames,
 *         or the radar is in RADAR_FAULT.
 *   A target is departing when its sensor speed or its range trend over
 *   3 frames is above +10 cm/s.
 *   FR13  closing speed and TTC = distance / closing speed go to BLE.
 *   FR6   grip state from the cane: HELD and UNKNOWN keep the alert active,
 *         only RELEASED silences it (and ends an active alert with one
 *         OBSTACLE(CLEAR)). The state starts as UNKNOWN.
 */

#ifndef THREAT_ASSESSMENT_H
#define THREAT_ASSESSMENT_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "rte_type.h"

/** Number of past frames kept for the range trend (keep in step with cfg). */
#define THREAT_ASSESSMENT_HISTORY_LEN    (4u)

/** State of Glasses_Threat: CLEAR (b_active false) or OBSTACLE_ACTIVE. */
typedef struct
{
    bool     b_active;
    bool     b_seen;
    uint32_t last_seen_ms;
    uint32_t last_d_mm;
    uint16_t last_ttc_ms;
    uint32_t hist_d_mm[THREAT_ASSESSMENT_HISTORY_LEN];  /**< [0] = newest. */
    uint32_t hist_ms[THREAT_ASSESSMENT_HISTORY_LEN];
    uint8_t  hist_count;
    uint8_t  depart_frames;  /**< Frames in a row with only departing. */
    bool     b_jump_pending; /**< A far jump was ignored once (jump gate). */
    rte_grip_state_t grip;   /**< Last grip state; kept across a fault. */
    // OPEN-20: b_stop_cue/stop_cue_on_ms no longer drive the FR9 cue byte
    // (that STOP bit now means "all three sides blocked", see
    // threat_assessment_direction_cue()); still computed in case a near-
    // distance STOP sub-cue is wanted again, but currently dormant output.
    bool     b_stop_cue;     /**< Near-distance STOP sub-state (dormant). */
    uint32_t stop_cue_on_ms; /**< Time b_stop_cue last became true. */
    rte_cue_side_t last_side; /**< Side of the target last alerted on
                                    (OPEN-16 tie-break only, not the cue). */

    // Collision risk assessment (OPEN-17) and persistent track IDs
    // (OPEN-18): up to RTE_RADAR_NUM_TARGETS tracks, matched to each
    // frame's targets by position (not by slot index, which the LD2450
    // does not promise is stable) via greedy assignment in
    // threat_assessment_assign_tracks(). A track survives a short run of
    // frames with no match (track_miss_frames) before being dropped, so a
    // briefly-lost target keeps its motion history instead of restarting.
    int16_t  track_x_mm[RTE_RADAR_NUM_TARGETS];
    int16_t  track_y_mm[RTE_RADAR_NUM_TARGETS];
    uint32_t track_ms[RTE_RADAR_NUM_TARGETS];   /**< Last time matched. */
    uint8_t  track_miss_frames[RTE_RADAR_NUM_TARGETS]; /**< Since last match. */
    bool     track_valid[RTE_RADAR_NUM_TARGETS];       /**< Not yet dropped. */

    // OPEN-20: per-side corridor occupancy, independent of which target is
    // the MIO. Hysteresis mirrors FR8/NFR8 (THREAT_ASSESSMENT_START_MM /
    // _STOP_MM): a side becomes blocked once some target on it is within
    // START_MM, and clears once none is within STOP_MM (or none is on that
    // side at all this frame). Drives the FR9 cue's new direction meaning
    // (threat_assessment_build_cue()), replacing the old side-of-threat
    // encoding.
    bool     b_left_blocked;
    bool     b_middle_blocked;
    bool     b_right_blocked;
} threat_assessment_t;

/** What the caller must send after one threat_assessment_update(). */
typedef struct
{
    bool     b_post_obstacle;  /**< Send OBSTACLE(d_mm, cue) (every frame). */
    bool     b_post_clear;     /**< Send OBSTACLE(CLEAR) (once per exit). */
    bool     b_active;         /**< State after this update. */
    uint16_t d_mm;             /**< Distance for OBSTACLE(d_mm). */
    uint16_t ttc_ms;           /**< RTE_TTC_NONE_MS if not approaching. */
    uint8_t  cue;              /**< FR9 cue byte (direction, OPEN-20); 0
                                     unless b_post_obstacle. */

    // OPEN-19: bench-debug only (see rte_write_mio_debug()). Never sent to
    // Audio Alert or BLE; a fresh selection was not made this frame (jump-
    // gate hold, or no target) leaves these at their default/false values.
    bool     b_mio_present;    /**< A target was selected this frame. */
    size_t   mio_track_slot;   /**< Track slot id, or RTE_RADAR_NUM_TARGETS
                                     if none (new/lost this frame). */
    uint32_t mio_risk_mm;      /**< Risk-adjusted selection distance. */
    bool     b_mio_collision;  /**< Collision course predicted this frame. */
} threat_assessment_out_t;

/** Link-time configuration (defined in threat_assessment_lcfg.c). */
typedef struct
{
    /** Assumption A7: negative speed means approaching the sensor. */
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
bool threat_assessment_is_departing (const rte_radar_target_t * p_target);
uint16_t threat_assessment_ttc_ms (uint32_t d_mm, int32_t closing_cms);

#endif /* THREAT_ASSESSMENT_H */

/*** end of file ***/
