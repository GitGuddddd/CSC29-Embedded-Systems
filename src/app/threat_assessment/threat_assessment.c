/** @file threat_assessment.c
 *
 * @brief Glasses obstacle decision: nearest target, hysteresis, TTC.
 *
 * @par
 * COPYRIGHT NOTICE: (c) 2026 Group 29. All rights reserved.
 *
 * Pure algorithm: no Pico SDK and no RTOS. Output goes through the RTE.
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "rte.h"
#include "rte_type.h"
#include "threat_assessment.h"
#include "threat_assessment_cfg.h"

/** Per-target-slot motion data for this frame, built once by
 *  threat_assessment_assign_tracks() before selection runs (OPEN-18). */
typedef struct
{
    bool     b_has_track; /**< false: no persistent track matched (new or
                                lost target; no velocity available). */
    size_t   track_slot;  /**< Matched track's slot id (OPEN-19, debug
                                only), or RTE_RADAR_NUM_TARGETS. */
    int16_t  prev_x_mm;   /**< The matched track's position before this
                                frame (only meaningful if b_has_track). */
    int16_t  prev_y_mm;
    uint32_t prev_ms;
} threat_assessment_motion_t;

static bool threat_assessment_find_nearest (
    const threat_assessment_t * p_state,
    const rte_radar_target_t * p_targets,
    const threat_assessment_motion_t * p_motion, bool b_include_departing,
    size_t skip_index, size_t speed_exempt_index, bool b_mio_tiebreak,
    uint32_t now_ms, size_t * p_index, uint32_t * p_d_mm);
static bool threat_assessment_candidate_wins (
    const rte_radar_target_t * p_candidate, uint32_t cand_d_mm,
    const threat_assessment_motion_t * p_cand_motion,
    const rte_radar_target_t * p_best, uint32_t best_d_mm,
    const threat_assessment_motion_t * p_best_motion,
    const threat_assessment_t * p_state, uint32_t now_ms);
static bool threat_assessment_predicts_collision (
    const rte_radar_target_t * p_target,
    const threat_assessment_motion_t * p_motion, uint32_t now_ms);
static uint32_t threat_assessment_effective_risk_mm (
    const rte_radar_target_t * p_target, uint32_t d_mm,
    const threat_assessment_motion_t * p_motion, uint32_t now_ms);
static void threat_assessment_assign_tracks (
    threat_assessment_t * p_state, const rte_radar_target_t * p_targets,
    uint32_t now_ms, threat_assessment_motion_t * p_motion);
static bool threat_assessment_range_confirms_speed (
    const threat_assessment_t * p_state, uint32_t d_mm, uint32_t now_ms);
static bool threat_assessment_jump_gate (
    threat_assessment_t * p_state, uint32_t d_mm, uint32_t now_ms);
static bool threat_assessment_trend_away (
    const threat_assessment_t * p_state, uint32_t d_mm, uint32_t now_ms);
static void threat_assessment_push_history (
    threat_assessment_t * p_state, uint32_t d_mm, uint32_t now_ms);
static void threat_assessment_on_target (
    threat_assessment_t * p_state, const rte_radar_target_t * p_target,
    uint32_t d_mm, uint32_t now_ms, threat_assessment_out_t * p_out);
static void threat_assessment_on_no_target (
    threat_assessment_t * p_state, uint32_t now_ms,
    threat_assessment_out_t * p_out);
static bool threat_assessment_far_target_must_wait (
    const threat_assessment_t * p_state, bool b_any_valid, size_t any_index,
    uint32_t any_d_mm, size_t nearest_index, uint32_t nearest_d_mm);
static void threat_assessment_set_stop_cue (
    threat_assessment_t * p_state, uint32_t d_mm, uint32_t now_ms);
static uint8_t threat_assessment_build_cue (
    const threat_assessment_t * p_state);
static void threat_assessment_hold_side (
    bool * p_blocked, uint32_t nearest_d_mm);
static void threat_assessment_scan_sides (
    threat_assessment_t * p_state, const rte_radar_target_t * p_targets);
static uint8_t threat_assessment_direction_cue (
    bool b_left_blocked, bool b_middle_blocked, bool b_right_blocked);

/*!
 * @brief Put the state machine into CLEAR.
 *
 * @param[out] p_state  State to initialise.
 */
void
threat_assessment_init (threat_assessment_t * p_state)
{
    p_state->b_active = false;
    p_state->b_seen = false;
    p_state->last_seen_ms = 0u;
    p_state->last_d_mm = 0u;
    p_state->last_ttc_ms = RTE_TTC_NONE_MS;
    p_state->hist_count = 0u;
    p_state->depart_frames = 0u;
    p_state->b_jump_pending = false;
    p_state->grip = RTE_GRIP_UNKNOWN;
    p_state->b_stop_cue = false;
    p_state->stop_cue_on_ms = 0u;
    p_state->last_side = RTE_SIDE_CENTER;
    p_state->b_left_blocked = false;
    p_state->b_middle_blocked = false;
    p_state->b_right_blocked = false;

    for (size_t idx = 0u; idx < THREAT_ASSESSMENT_HISTORY_LEN; idx++)
    {
        p_state->hist_d_mm[idx] = 0u;
        p_state->hist_ms[idx] = 0u;
    }

    for (size_t idx = 0u; idx < RTE_RADAR_NUM_TARGETS; idx++)
    {
        p_state->track_x_mm[idx] = 0;
        p_state->track_y_mm[idx] = 0;
        p_state->track_ms[idx] = 0u;
        p_state->track_miss_frames[idx] = 0u;
        p_state->track_valid[idx] = false;
    }
}

/*!
 * @brief Radar fault: force CLEAR so a stale tone can never stay on (FR10).
 *
 * The grip state is not part of the radar state, so it survives the reset.
 *
 * @param[in,out] p_state  State to reset.
 *
 * @return true when an obstacle was active (the caller posts CLEAR).
 */
bool
threat_assessment_reset (threat_assessment_t * p_state)
{
    bool b_was_active = p_state->b_active;
    rte_grip_state_t grip = p_state->grip;

    threat_assessment_init(p_state);
    p_state->grip = grip;

    return (b_was_active);
}

/*!
 * @brief Store a new grip state (FR6).
 *
 * RELEASED ends an active alert. HELD and UNKNOWN never end one, and the next
 * frame starts the alert if a target is already inside 2000 mm.
 *
 * @param[in,out] p_state  Decision state.
 * @param[in]     grip     New grip state.
 *
 * @return true when an active alert was ended (the caller posts CLEAR).
 */
bool
threat_assessment_set_grip (threat_assessment_t * p_state,
                            rte_grip_state_t grip)
{
    bool b_cleared = false;

    p_state->grip = grip;

    if ((RTE_GRIP_RELEASED == grip) && p_state->b_active)
    {
        p_state->b_active = false;
        p_state->last_ttc_ms = RTE_TTC_NONE_MS;
        p_state->b_stop_cue = false;
        b_cleared = true;
    }

    return (b_cleared);
}

/*!
 * @brief Feed the targets of one valid frame (FR8, FR9, FR10, FR13).
 *
 * @param[in,out] p_state    Decision state.
 * @param[in]     p_targets  The three target slots of the frame.
 * @param[in]     now_ms     Time the frame was received.
 * @param[out]    p_out      What the caller must send.
 */
void
threat_assessment_update (threat_assessment_t * p_state,
                          const rte_radar_target_t * p_targets,
                          uint32_t now_ms, threat_assessment_out_t * p_out)
{
    size_t nearest_index = 0u;
    uint32_t nearest_d_mm = 0u;
    size_t any_index = 0u;
    uint32_t any_d_mm = 0u;
    bool b_any_valid;
    bool b_trend_departing = false;
    bool b_found;
    size_t speed_exempt_index = RTE_RADAR_NUM_TARGETS;
    bool b_hold;
    const bool b_was_active = p_state->b_active;
    threat_assessment_motion_t motion[RTE_RADAR_NUM_TARGETS];

    p_out->b_post_obstacle = false;
    p_out->b_post_clear = false;
    p_out->d_mm = 0u;
    p_out->ttc_ms = RTE_TTC_NONE_MS;
    p_out->cue = 0u;
    p_out->b_mio_present = false;
    p_out->mio_track_slot = RTE_RADAR_NUM_TARGETS;
    p_out->mio_risk_mm = 0u;
    p_out->b_mio_collision = false;

    // OPEN-18: match this frame's targets to persistent tracks first, so
    // the selection below has real motion data to work with regardless of
    // which path (hold, selection, or no target) the frame takes.
    threat_assessment_assign_tracks(p_state, p_targets, now_ms, motion);

    // OPEN-20: per-side corridor occupancy. Independent of MIO selection
    // below (it does not care which target is "most important", only
    // whether each side has anything close on it), so it runs
    // unconditionally every frame, not inside the hold/departing branches.
    threat_assessment_scan_sides(p_state, p_targets);

    // Nearest valid target of any speed: its range feeds the trend. The MIO
    // tie-break is not applied here: this scan only measures range for the
    // jump gate and departure trend, not which target gets reported.
    b_any_valid = threat_assessment_find_nearest(
        p_state, p_targets, motion, true, RTE_RADAR_NUM_TARGETS,
        RTE_RADAR_NUM_TARGETS, false, now_ms, &any_index, &any_d_mm);

    // Jump gate: a one-frame jump away is ignored once while the alert is on.
    b_hold = b_any_valid &&
             threat_assessment_jump_gate(p_state, any_d_mm, now_ms);

    if (b_hold)
    {
        // Keep the alert and the history as they were; the output below
        // repeats the last distance.
    }
    else
    {
        if (b_any_valid)
        {
            b_trend_departing = threat_assessment_trend_away(
                p_state, any_d_mm, now_ms);

            // The sensor speed of the nearest target is only believed when
            // its range also grows (report D5).
            if (!threat_assessment_range_confirms_speed(
                    p_state, any_d_mm, now_ms))
            {
                speed_exempt_index = any_index;
            }

            threat_assessment_push_history(p_state, any_d_mm, now_ms);
        }
        else
        {
            p_state->hist_count = 0u;
        }

        // The final selection (FR9 MIO, OPEN-16): this is the one scan whose
        // result is actually reported, so the tie-break applies here.
        b_found = threat_assessment_find_nearest(
            p_state, p_targets, motion, false,
            b_trend_departing ? any_index : RTE_RADAR_NUM_TARGETS,
            speed_exempt_index, true, now_ms, &nearest_index, &nearest_d_mm);

        // A far target must not end the alert on the first frame in which the
        // near target looks like it is leaving. Then this frame is treated
        // like a frame with no target: the 2 trend frames or the 500 ms hold
        // decide, as they do when no other target is present.
        if (b_found && threat_assessment_far_target_must_wait(
                p_state, b_any_valid, any_index, any_d_mm, nearest_index,
                nearest_d_mm))
        {
            b_found = false;
        }

        if (b_found)
        {
            p_state->depart_frames = 0u;
            threat_assessment_on_target(p_state, &p_targets[nearest_index],
                                        nearest_d_mm, now_ms, p_out);

            // OPEN-19: bench-debug info about this frame's selection, filled
            // after the fact so it never changes what got selected above.
            p_out->b_mio_present = true;
            p_out->mio_track_slot = motion[nearest_index].track_slot;
            p_out->mio_risk_mm = threat_assessment_effective_risk_mm(
                &p_targets[nearest_index], nearest_d_mm,
                &motion[nearest_index], now_ms);
            p_out->b_mio_collision = threat_assessment_predicts_collision(
                &p_targets[nearest_index], &motion[nearest_index], now_ms);
        }
        else
        {
            // Only a real range trend counts: the sensor speed alone can
            // read 24 cm/s on a target that has not moved (bench log
            // 2026-10-18).
            if (b_any_valid && b_trend_departing)
            {
                if (p_state->depart_frames <
                    THREAT_ASSESSMENT_DEPART_CLEAR_FRAMES)
                {
                    p_state->depart_frames++;
                }
            }
            else
            {
                p_state->depart_frames = 0u;
            }

            threat_assessment_on_no_target(p_state, now_ms, p_out);
        }
    }

    if (RTE_GRIP_RELEASED == p_state->grip)
    {
        // FR6: only a confirmed release silences the alert. The frame was
        // still processed above, so the range history stays current. Normally
        // the alert is already off (threat_assessment_set_grip ended it);
        // if it was still on, it ends here with one CLEAR.
        p_state->b_active = false;
        p_state->b_stop_cue = false;
        p_out->b_post_clear = b_was_active;
    }

    if (p_state->b_active)
    {
        // The distance comes from threat_assessment_distance_mm(), which
        // never exceeds 46339, so the narrowing cast cannot lose bits.
        p_out->b_post_obstacle = true;
        p_out->d_mm = (uint16_t) p_state->last_d_mm;
        p_out->ttc_ms = p_state->last_ttc_ms;
        p_out->cue = threat_assessment_build_cue(p_state);
    }

    p_out->b_active = p_state->b_active;
}

/*!
 * @brief Run the decision for one frame and send the result via the RTE.
 *
 * @param[in,out] p_state    Decision state.
 * @param[in]     p_targets  The three target slots of the frame.
 * @param[in]     now_ms     Time the frame was received.
 */
void
threat_assessment_process_frame (threat_assessment_t * p_state,
                                 const rte_radar_target_t * p_targets,
                                 uint32_t now_ms)
{
    threat_assessment_out_t out;

    threat_assessment_update(p_state, p_targets, now_ms, &out);

    if (out.b_post_obstacle)
    {
        rte_write_audio_msg(RTE_AUDIO_OBSTACLE, out.d_mm, out.cue);
        rte_write_ble_obstacle(true, out.d_mm, out.ttc_ms);
    }

    if (out.b_post_clear)
    {
        rte_write_audio_msg(RTE_AUDIO_OBSTACLE_CLEAR, 0u, 0u);
        rte_write_ble_obstacle(false, 0u, RTE_TTC_NONE_MS);
    }

    // OPEN-19: bench-debug only; a no-op unless FEATURE_DEBUG_LOG_MIO is 1.
    rte_write_mio_debug(out.b_mio_present, out.mio_track_slot, out.d_mm,
                        out.mio_risk_mm, out.b_mio_collision);

    // OPEN-20: bench-debug only, same flag; the FR9 direction cue in plain
    // language, so it does not have to be decoded from the raw byte by
    // hand on the bench.
    rte_write_dir_debug(p_state->b_left_blocked, p_state->b_middle_blocked,
                        p_state->b_right_blocked, out.b_active, out.cue);
}

/*!
 * @brief Handle a RADAR_FAULT: go to CLEAR and tell Audio Alert and BLE.
 *
 * @param[in,out] p_state  Decision state.
 */
void
threat_assessment_process_fault (threat_assessment_t * p_state)
{
    if (threat_assessment_reset(p_state))
    {
        rte_write_audio_msg(RTE_AUDIO_OBSTACLE_CLEAR, 0u, 0u);
        rte_write_ble_obstacle(false, 0u, RTE_TTC_NONE_MS);
    }
}

/*!
 * @brief Handle a grip state message: store it, end the alert on RELEASED.
 *
 * @param[in,out] p_state  Decision state.
 * @param[in]     grip     New grip state.
 */
void
threat_assessment_process_grip (threat_assessment_t * p_state,
                                rte_grip_state_t grip)
{
    if (threat_assessment_set_grip(p_state, grip))
    {
        rte_write_audio_msg(RTE_AUDIO_OBSTACLE_CLEAR, 0u, 0u);
        rte_write_ble_obstacle(false, 0u, RTE_TTC_NONE_MS);
    }
}

/*!
 * @brief Distance from the sensor to a target.
 *
 * The distance is worked out by the radar driver and arrives in the radar
 * message in 2 mm steps, so this only reads it.
 *
 * @param[in] p_target  The target.
 *
 * @return Distance in mm.
 */
uint16_t
threat_assessment_distance_mm (const rte_radar_target_t * p_target)
{
    return (p_target->d_mm);
}

/*!
 * @brief Closing speed of a target.
 *
 * @param[in] p_target  The target.
 *
 * @return Closing speed in cm/s: positive = approaching.
 */
int32_t
threat_assessment_closing_cms (const rte_radar_target_t * p_target)
{
    int32_t speed_cms = p_target->speed_cms;

    return (g_threat_assessment_lcfg.b_negative_speed_is_approaching
                ? -speed_cms
                : speed_cms);
}

/*!
 * @brief Test whether a target moves away faster than the departing limit.
 *
 * @param[in] p_target  The target.
 *
 * @return true when departing. A stationary target is not departing.
 */
bool
threat_assessment_is_departing (const rte_radar_target_t * p_target)
{
    return (threat_assessment_closing_cms(p_target) <
            -THREAT_ASSESSMENT_DEPARTING_CMS);
}

/*!
 * @brief Time to collision for a target.
 *
 * @param[in] d_mm         Distance in mm.
 * @param[in] closing_cms  Closing speed in cm/s.
 *
 * @return TTC in ms, capped below RTE_TTC_NONE_MS, or RTE_TTC_NONE_MS when
 *         the target is not approaching.
 */
uint16_t
threat_assessment_ttc_ms (uint32_t d_mm, int32_t closing_cms)
{
    uint16_t ttc_ms = RTE_TTC_NONE_MS;

    if (closing_cms > 0)
    {
        // The cast is safe: closing_cms was just checked to be positive.
        uint32_t value = (d_mm * THREAT_ASSESSMENT_TTC_FACTOR) /
                         (uint32_t) closing_cms;

        if (value >= RTE_TTC_NONE_MS)
        {
            ttc_ms = RTE_TTC_NONE_MS - 1u;
        }
        else
        {
            // The cast is safe: value is below RTE_TTC_NONE_MS.
            ttc_ms = (uint16_t) value;
        }
    }

    return (ttc_ms);
}

/*!
 * @brief Match this frame's targets to persistent tracks by greedy global
 *        nearest-distance assignment (OPEN-18, HLK-LD2450 MIO checklist
 *        section 6 "targets are associated across consecutive frames"),
 *        and refresh the track state for next frame.
 *
 * Unlike taking each target's single nearest previous position
 * independently (OPEN-17's first cut), this assigns globally: picks the
 * single closest (target, track) pair first, removes both from the pool,
 * and repeats, so two targets can never both claim the same track in one
 * frame. This is a greedy assignment, not the optimal (Hungarian) one; at
 * up to RTE_RADAR_NUM_TARGETS (3) objects the two agree closely enough in
 * practice that the much larger assignment-problem machinery is not worth
 * it (see docs/deviations.md OPEN-18).
 *
 * A track that goes unmatched for up to
 * THREAT_ASSESSMENT_TRACK_LOST_GRACE_FRAMES is kept alive rather than
 * dropped at once, so a target briefly lost (one noisy or invalid frame)
 * keeps its motion history instead of restarting as a new, unknown object.
 *
 * @param[in,out] p_state    Decision state (the persistent tracks).
 * @param[in]     p_targets  The three target slots of this frame.
 * @param[in]     now_ms     Time of this frame.
 * @param[out]    p_motion   Per-target-slot motion data for this frame
 *                           (array of RTE_RADAR_NUM_TARGETS), for the
 *                           selection step below to use.
 */
static void
threat_assessment_assign_tracks (threat_assessment_t * p_state,
                                 const rte_radar_target_t * p_targets,
                                 uint32_t now_ms,
                                 threat_assessment_motion_t * p_motion)
{
    size_t assign[RTE_RADAR_NUM_TARGETS];
    bool track_taken[RTE_RADAR_NUM_TARGETS];
    const int32_t gate_sq = (int32_t) THREAT_ASSESSMENT_ASSOC_GATE_MM *
                            (int32_t) THREAT_ASSESSMENT_ASSOC_GATE_MM;

    for (size_t idx = 0u; idx < RTE_RADAR_NUM_TARGETS; idx++)
    {
        assign[idx] = RTE_RADAR_NUM_TARGETS;
        track_taken[idx] = false;
        p_motion[idx].b_has_track = false;
        p_motion[idx].track_slot = RTE_RADAR_NUM_TARGETS;
        p_motion[idx].prev_x_mm = 0;
        p_motion[idx].prev_y_mm = 0;
        p_motion[idx].prev_ms = 0u;
    }

    // Greedy global assignment: repeatedly take the single closest
    // (target, track) pair left, within the gate, until none is left to
    // take. At most RTE_RADAR_NUM_TARGETS passes are ever needed.
    for (size_t pass = 0u; pass < RTE_RADAR_NUM_TARGETS; pass++)
    {
        size_t best_target = RTE_RADAR_NUM_TARGETS;
        size_t best_track = RTE_RADAR_NUM_TARGETS;
        int32_t best_dist_sq = gate_sq;

        for (size_t t_idx = 0u; t_idx < RTE_RADAR_NUM_TARGETS; t_idx++)
        {
            bool b_target_free = p_targets[t_idx].b_valid &&
                                 (RTE_RADAR_NUM_TARGETS == assign[t_idx]);

            for (size_t k_idx = 0u;
                 b_target_free && (k_idx < RTE_RADAR_NUM_TARGETS); k_idx++)
            {
                int32_t dx;
                int32_t dy;
                int32_t dist_sq;
                bool b_track_free = p_state->track_valid[k_idx] &&
                                    !track_taken[k_idx];

                dx = (int32_t) p_targets[t_idx].x_mm -
                    (int32_t) p_state->track_x_mm[k_idx];
                dy = (int32_t) p_targets[t_idx].y_mm -
                    (int32_t) p_state->track_y_mm[k_idx];
                dist_sq = (dx * dx) + (dy * dy);

                if (b_track_free && (dist_sq <= best_dist_sq))
                {
                    best_target = t_idx;
                    best_track = k_idx;
                    best_dist_sq = dist_sq;
                }
            }
        }

        if (RTE_RADAR_NUM_TARGETS == best_target)
        {
            break;
        }

        assign[best_target] = best_track;
        track_taken[best_track] = true;
    }

    // Matched targets: hand the selection step the track's position from
    // BEFORE this frame (the velocity needs a before and an after), then
    // refresh the track to this frame's position so next frame sees this
    // frame as "before".
    for (size_t idx = 0u; idx < RTE_RADAR_NUM_TARGETS; idx++)
    {
        if (RTE_RADAR_NUM_TARGETS != assign[idx])
        {
            size_t k_idx = assign[idx];

            p_motion[idx].b_has_track = true;
            p_motion[idx].track_slot = k_idx;
            p_motion[idx].prev_x_mm = p_state->track_x_mm[k_idx];
            p_motion[idx].prev_y_mm = p_state->track_y_mm[k_idx];
            p_motion[idx].prev_ms = p_state->track_ms[k_idx];

            p_state->track_x_mm[k_idx] = p_targets[idx].x_mm;
            p_state->track_y_mm[k_idx] = p_targets[idx].y_mm;
            p_state->track_ms[k_idx] = now_ms;
            p_state->track_miss_frames[k_idx] = 0u;
        }
    }

    // Unmatched valid targets claim a free track slot (a genuinely new
    // object - or no free slot this round, in which case it simply starts
    // with no motion data, same as the first frame it is ever seen in).
    for (size_t idx = 0u; idx < RTE_RADAR_NUM_TARGETS; idx++)
    {
        if (p_targets[idx].b_valid && (RTE_RADAR_NUM_TARGETS == assign[idx]))
        {
            for (size_t k_idx = 0u; k_idx < RTE_RADAR_NUM_TARGETS; k_idx++)
            {
                if (!p_state->track_valid[k_idx])
                {
                    p_state->track_valid[k_idx] = true;
                    p_state->track_x_mm[k_idx] = p_targets[idx].x_mm;
                    p_state->track_y_mm[k_idx] = p_targets[idx].y_mm;
                    p_state->track_ms[k_idx] = now_ms;
                    p_state->track_miss_frames[k_idx] = 0u;
                    break;
                }
            }
        }
    }

    // Tracks nothing matched this frame: age them, and drop any missing
    // too long - stale enough that a reappearance nearby is no longer
    // trusted as "the same object" (it may free its slot next frame).
    for (size_t k_idx = 0u; k_idx < RTE_RADAR_NUM_TARGETS; k_idx++)
    {
        if (p_state->track_valid[k_idx] && !track_taken[k_idx])
        {
            if (p_state->track_miss_frames[k_idx] <
                THREAT_ASSESSMENT_TRACK_LOST_GRACE_FRAMES)
            {
                p_state->track_miss_frames[k_idx]++;
            }
            else
            {
                p_state->track_valid[k_idx] = false;
            }
        }
    }
}

/*!
 * @brief Predict whether a target is on course to pass within the safety
 *        radius of the sensor (OPEN-17/OPEN-18, HLK-LD2450 MIO checklist
 *        section 3 "target motion... influences risk", "distinguishes
 *        approaching, receding and crossing targets", and section 6 "time
 *        to closest approach... predicted clearance... configurable safety
 *        radius").
 *
 * Uses the 2D position delta against the matched track (not just the
 * sensor's own radial speed), so a target crossing the corridor is caught
 * the same way as one closing head-on. All integer math (no float):
 * positions and velocities stay in mm and mm/s.
 *
 * @param[in] p_target The target to check.
 * @param[in] p_motion Its motion data for this frame (from
 *                     threat_assessment_assign_tracks()).
 * @param[in] now_ms   Time of this frame.
 *
 * @return true when the predicted closest approach is within
 *         THREAT_ASSESSMENT_SAFETY_RADIUS_MM and within
 *         THREAT_ASSESSMENT_TCA_HORIZON_MS. false for a target with no
 *         matched track, one that is not actually closing (the checklist's
 *         "receding" case), or one whose predicted path clears the safety
 *         radius (including a near-zero relative speed, which simply never
 *         reaches the "approaching" branch below).
 */
static bool
threat_assessment_predicts_collision (
    const rte_radar_target_t * p_target,
    const threat_assessment_motion_t * p_motion, uint32_t now_ms)
{
    bool b_collision = false;

    if (p_motion->b_has_track)
    {
        uint32_t dt_ms =
            (now_ms > p_motion->prev_ms) ? (now_ms - p_motion->prev_ms) : 0u;

        // A zero or implausibly old interval gives no reliable velocity.
        if ((dt_ms > 0u) && (dt_ms <= THREAT_ASSESSMENT_TCA_HORIZON_MS))
        {
            int32_t x_mm = p_target->x_mm;
            int32_t y_mm = p_target->y_mm;
            int32_t vx_mm_s =
                (((int32_t) x_mm - (int32_t) p_motion->prev_x_mm) * 1000) /
                (int32_t) dt_ms;
            int32_t vy_mm_s =
                (((int32_t) y_mm - (int32_t) p_motion->prev_y_mm) * 1000) /
                (int32_t) dt_ms;
            int64_t pv = ((int64_t) x_mm * vx_mm_s) +
                         ((int64_t) y_mm * vy_mm_s);

            // pv < 0 means position and velocity point opposite ways: the
            // target is actually closing on the sensor. pv >= 0 is
            // stationary, receding, or moving parallel - never a collision
            // course, whatever its current distance.
            if (pv < 0)
            {
                int64_t vv = ((int64_t) vx_mm_s * vx_mm_s) +
                             ((int64_t) vy_mm_s * vy_mm_s);

                // Time to closest approach: t* = -(P.V)/(V.V), clamped to
                // the horizon (vv > 0 here, since pv < 0 required motion).
                int64_t tca_ms = (-pv * 1000) / vv;
                int64_t px;
                int64_t py;
                int64_t clearance_sq;
                int64_t safety_sq =
                    (int64_t) THREAT_ASSESSMENT_SAFETY_RADIUS_MM *
                    THREAT_ASSESSMENT_SAFETY_RADIUS_MM;

                if (tca_ms > THREAT_ASSESSMENT_TCA_HORIZON_MS)
                {
                    tca_ms = THREAT_ASSESSMENT_TCA_HORIZON_MS;
                }

                px = x_mm + ((vx_mm_s * tca_ms) / 1000);
                py = y_mm + ((vy_mm_s * tca_ms) / 1000);
                clearance_sq = (px * px) + (py * py);
                b_collision = (clearance_sq <= safety_sq);
            }
        }
    }

    return (b_collision);
}

/*!
 * @brief A target's distance, reduced when it is predicted to pass close
 *        (OPEN-17, checklist section 3 "MIO selection considers more than
 *        just the nearest distance").
 *
 * This is a selection heuristic only: callers must keep using the real
 * d_mm (not this value) for anything reported to the person or fed into
 * the hysteresis/TTC/history logic above, which all depend on true range.
 *
 * @param[in] p_target The target.
 * @param[in] d_mm     Its real distance in mm.
 * @param[in] p_motion Its motion data for this frame.
 * @param[in] now_ms   Time of this frame.
 *
 * @return d_mm, or d_mm minus THREAT_ASSESSMENT_COLLISION_BONUS_MM (never
 *         below 0) when a collision course is predicted.
 */
static uint32_t
threat_assessment_effective_risk_mm (
    const rte_radar_target_t * p_target, uint32_t d_mm,
    const threat_assessment_motion_t * p_motion, uint32_t now_ms)
{
    uint32_t risk_mm = d_mm;

    if (threat_assessment_predicts_collision(p_target, p_motion, now_ms))
    {
        risk_mm = (d_mm > THREAT_ASSESSMENT_COLLISION_BONUS_MM)
                      ? (d_mm - THREAT_ASSESSMENT_COLLISION_BONUS_MM)
                      : 0u;
    }

    return (risk_mm);
}

/*!
 * @brief Decide whether a candidate target outranks the current best (MIO
 *        tie-break, OPEN-16/OPEN-17/OPEN-18, HLK-LD2450 MIO checklist
 *        sections 3 and 4).
 *
 * Compares EFFECTIVE risk distance (threat_assessment_effective_risk_mm:
 * real distance, reduced when a 2D collision course is predicted), not raw
 * distance, so a farther target closing in or crossing the corridor can
 * outrank a nearer but safe one (checklist section 9's named test). When
 * neither target triggers that reduction, effective distance equals real
 * distance and this behaves exactly as the original OPEN-16 rule. Within
 * THREAT_ASSESSMENT_MIO_TIE_BAND_MM of effective distance: a real
 * (approaching) TTC beats no TTC; if both or neither have one, the side
 * already being reported wins (stability, checklist section 5); strict
 * distance is the last-resort tie-break so the result is always
 * deterministic.
 *
 * @param[in] p_candidate   The target being considered.
 * @param[in] cand_d_mm     Its real distance in mm.
 * @param[in] p_cand_motion Its motion data for this frame.
 * @param[in] p_best        The current best target.
 * @param[in] best_d_mm     Its real distance in mm.
 * @param[in] p_best_motion Its motion data for this frame.
 * @param[in] p_state       Decision state (for the side already reported).
 * @param[in] now_ms        Time of this frame.
 *
 * @return true when the candidate should replace the current best.
 */
static bool
threat_assessment_candidate_wins (
    const rte_radar_target_t * p_candidate, uint32_t cand_d_mm,
    const threat_assessment_motion_t * p_cand_motion,
    const rte_radar_target_t * p_best, uint32_t best_d_mm,
    const threat_assessment_motion_t * p_best_motion,
    const threat_assessment_t * p_state, uint32_t now_ms)
{
    bool b_wins;
    uint32_t cand_risk_mm = threat_assessment_effective_risk_mm(
        p_candidate, cand_d_mm, p_cand_motion, now_ms);
    uint32_t best_risk_mm = threat_assessment_effective_risk_mm(
        p_best, best_d_mm, p_best_motion, now_ms);
    uint32_t diff_mm = (cand_risk_mm > best_risk_mm)
                           ? (cand_risk_mm - best_risk_mm)
                           : (best_risk_mm - cand_risk_mm);

    if (diff_mm > THREAT_ASSESSMENT_MIO_TIE_BAND_MM)
    {
        b_wins = (cand_risk_mm < best_risk_mm);
    }
    else
    {
        uint16_t cand_ttc = threat_assessment_ttc_ms(
            cand_d_mm, threat_assessment_closing_cms(p_candidate));
        uint16_t best_ttc = threat_assessment_ttc_ms(
            best_d_mm, threat_assessment_closing_cms(p_best));

        if (cand_ttc != best_ttc)
        {
            b_wins = (cand_ttc < best_ttc);
        }
        else if (p_candidate->side != p_best->side)
        {
            b_wins = (p_candidate->side == p_state->last_side);
        }
        else
        {
            b_wins = (cand_d_mm < best_d_mm);
        }
    }

    return (b_wins);
}

/*!
 * @brief Find the nearest valid target (FR8, FR13), or the MIO when two are
 *        close enough to tie (OPEN-16, OPEN-17, OPEN-18).
 *
 * @param[in]  p_state             Decision state (for the tie-break only;
 *                                 unused when b_mio_tiebreak is false).
 * @param[in]  p_targets           The three target slots.
 * @param[in]  p_motion            This frame's per-target motion data (for
 *                                 the tie-break only).
 * @param[in]  b_include_departing  false skips targets departing by speed.
 * @param[in]  skip_index          Slot to ignore (RTE_RADAR_NUM_TARGETS: none).
 * @param[in]  speed_exempt_index  Slot whose speed verdict is ignored (none: RTE_RADAR_NUM_TARGETS).
 * @param[in]  b_mio_tiebreak      true to apply the near-tie rule above;
 *                                 false for plain strict-nearest (unchanged
 *                                 behaviour, used by the trend/jump-gate scan).
 * @param[in]  now_ms              Time of this frame (for the tie-break's
 *                                 collision prediction; unused otherwise).
 * @param[out] p_index    Index of the selected target.
 * @param[out] p_d_mm     Its distance in mm.
 *
 * @return true when such a target exists.
 */
static bool
threat_assessment_find_nearest (
    const threat_assessment_t * p_state, const rte_radar_target_t * p_targets,
    const threat_assessment_motion_t * p_motion, bool b_include_departing,
    size_t skip_index, size_t speed_exempt_index, bool b_mio_tiebreak,
    uint32_t now_ms, size_t * p_index, uint32_t * p_d_mm)
{
    bool b_found = false;

    for (size_t idx = 0u; idx < RTE_RADAR_NUM_TARGETS; idx++)
    {
        bool b_ok_speed = b_include_departing || (idx == speed_exempt_index) ||
                          (!threat_assessment_is_departing(&p_targets[idx]));

        if ((p_targets[idx].b_valid) && (idx != skip_index) && b_ok_speed)
        {
            uint32_t d_mm = threat_assessment_distance_mm(&p_targets[idx]);
            bool b_better;

            if (!b_found)
            {
                b_better = true;
            }
            else if (b_mio_tiebreak)
            {
                b_better = threat_assessment_candidate_wins(
                    &p_targets[idx], d_mm, &p_motion[idx],
                    &p_targets[*p_index], *p_d_mm, &p_motion[*p_index],
                    p_state, now_ms);
            }
            else
            {
                b_better = (d_mm < *p_d_mm);
            }

            if (b_better)
            {
                b_found = true;
                *p_index = idx;
                *p_d_mm = d_mm;
            }
        }
    }

    return (b_found);
}

/*!
 * @brief Apply the start and stop hysteresis for a frame with a target.
 *
 * @param[in,out] p_state   Decision state.
 * @param[in]     p_target  Nearest non-departing target.
 * @param[in]     d_mm      Its distance in mm.
 * @param[in]     now_ms    Time the frame was received.
 * @param[out]    p_out     Receives post_clear when the alert ends.
 */
static void
threat_assessment_on_target (threat_assessment_t * p_state,
                                 const rte_radar_target_t * p_target,
                                 uint32_t d_mm, uint32_t now_ms,
                                 threat_assessment_out_t * p_out)
{
    p_state->b_seen = true;
    p_state->last_seen_ms = now_ms;

    if (!p_state->b_active)
    {
        p_state->b_active = (d_mm < THREAT_ASSESSMENT_START_MM);
    }
    else if (d_mm > THREAT_ASSESSMENT_STOP_MM)
    {
        p_state->b_active = false;
        p_state->b_stop_cue = false;
        p_out->b_post_clear = true;
    }
    else
    {
        // Between 2000 and 2100 mm the current state is kept (hysteresis).
    }

    if (p_state->b_active)
    {
        p_state->last_d_mm = d_mm;
        p_state->last_ttc_ms = threat_assessment_ttc_ms(
            d_mm, threat_assessment_closing_cms(p_target));
        p_state->last_side = p_target->side;
        threat_assessment_set_stop_cue(p_state, d_mm, now_ms);
    }
}

/*!
 * @brief Hold the alert for a while with no target, then clear it (FR10).
 *
 * Clears at once when only departing targets were seen for several frames.
 *
 * @param[in,out] p_state  Decision state.
 * @param[in]     now_ms   Time the frame was received.
 * @param[out]    p_out    Receives post_clear when the alert ends.
 */
static void
threat_assessment_on_no_target (threat_assessment_t * p_state,
                                    uint32_t now_ms,
                                    threat_assessment_out_t * p_out)
{
    if (p_state->b_active)
    {
        p_state->last_ttc_ms = RTE_TTC_NONE_MS;

        // Unsigned subtraction stays correct when the clock wraps.
        if (((now_ms - p_state->last_seen_ms) >=
             THREAT_ASSESSMENT_NO_TARGET_HOLD_MS) ||
            (p_state->depart_frames >= THREAT_ASSESSMENT_DEPART_CLEAR_FRAMES))
        {
            p_state->b_active = false;
            p_state->b_stop_cue = false;
            p_out->b_post_clear = true;
        }
    }
}

/*!
 * @brief FR9 cue byte: STOP sub-hysteresis, independent of the alert's own
 * start/stop distance.
 *
 * ON is checked every frame the alert is active. OFF needs both the range to
 * have recovered past the OFF distance AND the minimum hold to have elapsed
 * since STOP last turned on: the hold only ever delays the end, never the
 * start, and never re-arms it early if the range dips back under ON first.
 *
 * @param[in,out] p_state  Decision state.
 * @param[in]     d_mm     Distance of the alerting target this frame.
 * @param[in]     now_ms   Time the frame was received.
 */
static void
threat_assessment_set_stop_cue (threat_assessment_t * p_state,
                                   uint32_t d_mm, uint32_t now_ms)
{
    if (!p_state->b_stop_cue)
    {
        if (d_mm <= THREAT_ASSESSMENT_STOP_CUE_ON_MM)
        {
            p_state->b_stop_cue = true;
            p_state->stop_cue_on_ms = now_ms;
        }
    }
    else if (d_mm > THREAT_ASSESSMENT_STOP_CUE_OFF_MM)
    {
        // Unsigned subtraction stays correct when the clock wraps.
        uint32_t held_ms = now_ms - p_state->stop_cue_on_ms;

        if (held_ms >= THREAT_ASSESSMENT_STOP_CUE_MIN_HOLD_MS)
        {
            p_state->b_stop_cue = false;
        }
        // else: the minimum hold has not elapsed, STOP stays on.
    }
    else
    {
        // Between ON and OFF (or already past OFF but still held): no change.
    }
}

/*!
 * @brief Build the FR9 cue byte for the currently active alert.
 *
 * OPEN-20: the cue byte's meaning changed from "which side the alerting
 * target is on" to "which way is open to walk", driven by per-side
 * corridor occupancy (threat_assessment_scan_sides()) rather than
 * the single MIO target. The bit layout (RTE_CUE_STOP, RTE_CUE_SIDE_*) is
 * unchanged, so nothing downstream of the byte (the RTE message, Brian's
 * bring-up decode in alert.c) needed a code change, only a re-read of what
 * the bits now mean.
 *
 * @param[in] p_state  Decision state.
 *
 * @return The byte for rte_audio_msg_t.cue (0 if called while not active;
 *         not expected, the caller only calls this when p_state->b_active).
 */
static uint8_t
threat_assessment_build_cue (const threat_assessment_t * p_state)
{
    return (threat_assessment_direction_cue(p_state->b_left_blocked,
                                            p_state->b_middle_blocked,
                                            p_state->b_right_blocked));
}

/*!
 * @brief Apply START/STOP hysteresis to one side's corridor occupancy.
 *
 * Mirrors FR8/NFR8 (THREAT_ASSESSMENT_START_MM / _STOP_MM): a side becomes
 * blocked once something on it is within START_MM, and clears once nothing
 * on it is within STOP_MM (nearest_d_mm reaching UINT32_MAX, from no valid
 * target on that side this frame, clears it the same way a distant target
 * would). Inside the gap between the two thresholds, the side keeps
 * whatever it already was, same as the main alert's own hysteresis.
 *
 * @param[in,out] p_blocked     This side's current blocked state.
 * @param[in]     nearest_d_mm  Nearest valid target on this side this
 *                              frame, or UINT32_MAX if none.
 */
static void
threat_assessment_hold_side (bool * p_blocked,
                                           uint32_t nearest_d_mm)
{
    if (nearest_d_mm <= THREAT_ASSESSMENT_START_MM)
    {
        *p_blocked = true;
    }
    else if (nearest_d_mm > THREAT_ASSESSMENT_STOP_MM)
    {
        *p_blocked = false;
    }
    else
    {
        // Between START and STOP: no change.
    }
}

/*!
 * @brief Scan this frame's targets and update all three sides' occupancy.
 *
 * DRAFT (OPEN-20): unlike the main alert, a side with no valid target this
 * frame clears immediately (nearest_d_mm treated as UINT32_MAX) rather than
 * waiting out a hold like THREAT_ASSESSMENT_NO_TARGET_HOLD_MS - a one-frame
 * sensor dropout could flicker a side open for a frame. Not bench-measured;
 * flagged as a known simplification, not treated as settled behaviour.
 * Departing targets are NOT excluded here (unlike MIO selection): a target
 * moving away still physically occupies that side of the corridor right
 * now, which is what this state is meant to capture.
 *
 * @param[in,out] p_state    Decision state (the three *_blocked fields).
 * @param[in]     p_targets  This frame's three target slots.
 */
static void
threat_assessment_scan_sides (threat_assessment_t * p_state,
                                       const rte_radar_target_t * p_targets)
{
    uint32_t left_d_mm = UINT32_MAX;
    uint32_t middle_d_mm = UINT32_MAX;
    uint32_t right_d_mm = UINT32_MAX;

    for (size_t idx = 0u; idx < RTE_RADAR_NUM_TARGETS; idx++)
    {
        if (p_targets[idx].b_valid)
        {
            uint32_t d_mm = (uint32_t) p_targets[idx].d_mm;

            if (RTE_SIDE_LEFT == p_targets[idx].side)
            {
                if (d_mm < left_d_mm)
                {
                    left_d_mm = d_mm;
                }
            }
            else if (RTE_SIDE_RIGHT == p_targets[idx].side)
            {
                if (d_mm < right_d_mm)
                {
                    right_d_mm = d_mm;
                }
            }
            else
            {
                if (d_mm < middle_d_mm)
                {
                    middle_d_mm = d_mm;
                }
            }
        }
    }

    threat_assessment_hold_side(&p_state->b_left_blocked,
                                              left_d_mm);
    threat_assessment_hold_side(&p_state->b_middle_blocked,
                                              middle_d_mm);
    threat_assessment_hold_side(&p_state->b_right_blocked,
                                              right_d_mm);
}

/*!
 * @brief Map the three sides' occupancy to the FR9 direction cue.
 *
 * Full 8-row table (OPEN-20, agreed with Putra 11 Nov): straight ahead is
 * preferred whenever it is safe, so a single side being blocked (with the
 * other two open) just holds CENTER rather than steering needlessly; the
 * only side that, blocked alone, forces a choice is the middle itself,
 * since both outer sides are then equally open - that genuine tie defaults
 * to LEFT (an arbitrary but fixed choice, not derived from the sensor).
 *
 * | Left | Middle | Right | Cue            |
 * |------|--------|-------|----------------|
 * |  -   |   -    |   -   | CENTER         |
 * |  X   |   -    |   -   | CENTER         |
 * |  -   |   -    |   X   | CENTER         |
 * |  -   |   X    |   -   | LEFT (default) |
 * |  X   |   X    |   -   | RIGHT          |
 * |  -   |   X    |   X   | LEFT           |
 * |  X   |   -    |   X   | CENTER         |
 * |  X   |   X    |   X   | STOP           |
 *
 * @param[in] b_left_blocked    Left side occupied within range.
 * @param[in] b_middle_blocked  Middle occupied within range.
 * @param[in] b_right_blocked   Right side occupied within range.
 *
 * @return The FR9 cue byte (RTE_CUE_STOP / RTE_CUE_SIDE_* bits).
 */
static uint8_t
threat_assessment_direction_cue (bool b_left_blocked, bool b_middle_blocked,
                                 bool b_right_blocked)
{
    uint8_t cue = 0u;
    rte_cue_side_t direction = RTE_SIDE_CENTER;

    if (b_left_blocked && b_middle_blocked && b_right_blocked)
    {
        cue |= RTE_CUE_STOP;
    }
    else if (b_left_blocked && b_middle_blocked)
    {
        direction = RTE_SIDE_RIGHT;
    }
    else if (b_middle_blocked && b_right_blocked)
    {
        direction = RTE_SIDE_LEFT;
    }
    else if (b_middle_blocked)
    {
        // Middle blocked alone: both sides equally open, no sensor-derived
        // reason to prefer one - fixed default, agreed with Putra 11 Nov.
        direction = RTE_SIDE_LEFT;
    }
    else
    {
        // Nothing blocked, left-only, right-only, or left+right (middle is
        // the only - or an equally safe - open path either way): stay
        // centre.
    }

    cue |= (uint8_t) (((uint8_t) direction & RTE_CUE_SIDE_MASK) <<
                      RTE_CUE_SIDE_SHIFT);

    return (cue);
}

/*!
 * @brief Tell whether the range backs up a "departing" sensor speed.
 *
 * Compares with the sample THREAT_ASSESSMENT_TREND_FRAMES frames ago, like the
 * range trend, but with half the speed limit. When there is not enough or too
 * old history the answer is true, so the speed verdict is kept (report D5).
 *
 * @param[in] p_state  Decision state with the range history.
 * @param[in] d_mm     Nearest range of this frame.
 * @param[in] now_ms   Time the frame was received.
 *
 * @return true when the speed verdict may be used.
 */
static bool
threat_assessment_range_confirms_speed (const threat_assessment_t * p_state,
                                        uint32_t d_mm, uint32_t now_ms)
{
    bool b_confirms = true;

    if (p_state->hist_count >= THREAT_ASSESSMENT_TREND_FRAMES)
    {
        const size_t old_idx = THREAT_ASSESSMENT_TREND_FRAMES - 1u;
        // Unsigned subtraction stays correct when the clock wraps.
        uint32_t dt_ms = now_ms - p_state->hist_ms[old_idx];
        int32_t delta_mm = (int32_t) d_mm -
                           (int32_t) p_state->hist_d_mm[old_idx];

        if ((dt_ms > 0u) && (dt_ms <= THREAT_ASSESSMENT_TREND_MAX_AGE_MS))
        {
            b_confirms = ((delta_mm * (int32_t) THREAT_ASSESSMENT_TTC_FACTOR) >
                          (THREAT_ASSESSMENT_SPEED_CONFIRM_CMS *
                           (int32_t) dt_ms));
        }
    }

    return (b_confirms);
}

/*!
 * @brief Tell whether a far target must wait before it can end the alert.
 *
 * True when an alert is on, the nearest target of any speed is still inside
 * the stop distance but was left out as departing (by sensor speed or by
 * range trend), and the target that remains is beyond the stop distance. One
 * noisy speed frame on the near target must not clear the alert just because
 * a far target exists (FR10, report D4).
 *
 * @param[in] p_state        Decision state.
 * @param[in] b_any_valid    A valid target of any speed exists.
 * @param[in] any_index      Slot of the nearest target of any speed.
 * @param[in] any_d_mm       Its distance in mm.
 * @param[in] nearest_index  Slot of the nearest non-departing target.
 * @param[in] nearest_d_mm   Its distance in mm.
 *
 * @return true when this frame must be handled as a frame with no target.
 */
static bool
threat_assessment_far_target_must_wait (const threat_assessment_t * p_state,
                                        bool b_any_valid, size_t any_index,
                                        uint32_t any_d_mm, size_t nearest_index,
                                        uint32_t nearest_d_mm)
{
    return (p_state->b_active && b_any_valid &&
            (nearest_index != any_index) &&
            (any_d_mm <= THREAT_ASSESSMENT_STOP_MM) &&
            (nearest_d_mm > THREAT_ASSESSMENT_STOP_MM));
}

/*!
 * @brief Jump gate: ignore a one-frame jump of the nearest range away.
 *
 * Only while an alert is active, only away from the user, and only once in a
 * row: if the next frame still shows the far range it is accepted. A jump
 * toward the user is never held back (NFR6).
 *
 * @param[in,out] p_state  Decision state (the pending flag).
 * @param[in]     d_mm     Nearest range of this frame.
 * @param[in]     now_ms   Time the frame was received.
 *
 * @return true when this frame must be ignored.
 */
static bool
threat_assessment_jump_gate (threat_assessment_t * p_state, uint32_t d_mm,
                             uint32_t now_ms)
{
    bool b_hold = false;
    bool b_jump = false;

    if (p_state->b_active && (p_state->hist_count > 0u))
    {
        // Unsigned subtraction stays correct when the clock wraps.
        uint32_t dt_ms = now_ms - p_state->hist_ms[0];

        b_jump = (dt_ms <= THREAT_ASSESSMENT_TREND_MAX_AGE_MS) &&
                 (d_mm > (p_state->hist_d_mm[0] +
                          THREAT_ASSESSMENT_JUMP_AWAY_MM));
    }

    if (b_jump && (!p_state->b_jump_pending))
    {
        p_state->b_jump_pending = true;
        b_hold = true;
    }
    else
    {
        p_state->b_jump_pending = false;
    }

    return (b_hold);
}

/*!
 * @brief Test whether the nearest range grew faster than the departing limit.
 *
 * Compares with the sample THREAT_ASSESSMENT_TREND_FRAMES frames ago. The
 * sensor speed is coarse and lags, so this catches slow departures that
 * report a speed of 0.
 *
 * @param[in] p_state  Decision state with the range history.
 * @param[in] d_mm     Nearest range of this frame.
 * @param[in] now_ms   Time the frame was received.
 *
 * @return true when the range trend is departing.
 */
static bool
threat_assessment_trend_away (const threat_assessment_t * p_state,
                                   uint32_t d_mm, uint32_t now_ms)
{
    bool b_departing = false;

    if (p_state->hist_count >= THREAT_ASSESSMENT_TREND_FRAMES)
    {
        const size_t old_idx = THREAT_ASSESSMENT_TREND_FRAMES - 1u;
        // Unsigned subtraction stays correct when the clock wraps.
        uint32_t dt_ms = now_ms - p_state->hist_ms[old_idx];
        int32_t delta_mm = (int32_t) d_mm -
                           (int32_t) p_state->hist_d_mm[old_idx];

        // d/dt in cm/s = delta_mm * 100 / dt_ms. The product stays far
        // below 2^31 because dt_ms is at most 400 and the range 46339.
        if ((dt_ms > 0u) && (dt_ms <= THREAT_ASSESSMENT_TREND_MAX_AGE_MS))
        {
            b_departing = ((delta_mm * (int32_t) THREAT_ASSESSMENT_TTC_FACTOR) >
                           (THREAT_ASSESSMENT_DEPARTING_CMS * (int32_t) dt_ms));
        }
    }

    return (b_departing);
}

/*!
 * @brief Remember the nearest range of this frame for the trend.
 *
 * @param[in,out] p_state  Decision state.
 * @param[in]     d_mm     Nearest range of this frame.
 * @param[in]     now_ms   Time the frame was received.
 */
static void
threat_assessment_push_history (threat_assessment_t * p_state,
                                uint32_t d_mm, uint32_t now_ms)
{
    for (size_t idx = THREAT_ASSESSMENT_HISTORY_LEN - 1u; idx > 0u; idx--)
    {
        p_state->hist_d_mm[idx] = p_state->hist_d_mm[idx - 1u];
        p_state->hist_ms[idx] = p_state->hist_ms[idx - 1u];
    }

    p_state->hist_d_mm[0] = d_mm;
    p_state->hist_ms[0] = now_ms;

    if (p_state->hist_count < THREAT_ASSESSMENT_HISTORY_LEN)
    {
        p_state->hist_count++;
    }
}

/*** end of file ***/
