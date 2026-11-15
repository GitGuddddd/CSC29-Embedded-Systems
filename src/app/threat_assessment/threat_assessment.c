/** @file threat_assessment.c
 *
 * @brief Glasses obstacle decision: per-lane occupancy, hysteresis.
 *
 * @par
 * COPYRIGHT NOTICE: (c) 2026 Group 29. All rights reserved.
 *
 * Pure algorithm: no Pico SDK and no RTOS. Output goes through the RTE.
 *
 * OPEN-22 (13 Nov): rewritten end to end. See threat_assessment.h and
 * docs/deviations.md OPEN-22 for the full before/after and the reasoning.
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "rte.h"
#include "rte_type.h"
#include "threat_assessment.h"
#include "threat_assessment_cfg.h"

/*! One frame's scan result per lane (OPEN-23): the nearest distance, and
 *  that nearest target's own live speed_cms - kept separate from
 *  threat_assessment_lane_t's held/latched state, since a TTC only ever
 *  makes sense from a reading taken this frame, never a held one. */
typedef struct
{
    uint32_t left_mm;
    int32_t  left_closing_cms;
    uint32_t middle_mm;
    int32_t  middle_closing_cms;
    uint32_t right_mm;
    int32_t  right_closing_cms;
} threat_assessment_scan_t;

// OPEN-25 (14 Nov): every lane "not BLOCKED", for the two CLEAR paths
// (threat_assessment_process_fault()/_process_grip()) that send straight
// to BLE without going through threat_assessment_update() and so have no
// threat_assessment_out_t of their own to read these from.
static const uint16_t g_no_lane_d_mm[3] = { RTE_LANE_NONE_MM,
                                            RTE_LANE_NONE_MM,
                                            RTE_LANE_NONE_MM };
static const uint16_t g_no_lane_ttc_ms[3] = { RTE_TTC_NONE_MS,
                                              RTE_TTC_NONE_MS,
                                              RTE_TTC_NONE_MS };

static void threat_assessment_init_lane (threat_assessment_lane_t * p_lane);
static void threat_assessment_scan_lanes (
    const rte_radar_target_t * p_targets, threat_assessment_scan_t * p_scan);
static void threat_assessment_hold_lane (threat_assessment_lane_t * p_lane,
                                         uint32_t nearest_mm,
                                         uint32_t now_ms);
static uint32_t threat_assessment_pick_lane (
    const threat_assessment_t * p_state);
static void threat_assessment_lane_out (
    const threat_assessment_t * p_state,
    const threat_assessment_scan_t * p_scan,
    threat_assessment_out_t * p_out);
static uint8_t threat_assessment_build_cue (
    const threat_assessment_t * p_state);
static uint8_t threat_assessment_direction_cue (
    bool b_left_blocked, bool b_middle_blocked, bool b_right_blocked);

/*!
 * @brief Put one lane into its empty/open starting state.
 *
 * @param[out] p_lane  Lane to initialise.
 */
static void
threat_assessment_init_lane (threat_assessment_lane_t * p_lane)
{
    p_lane->b_blocked = false;
    p_lane->last_d_mm = 0u;
    p_lane->hold_pending_ms = UINT32_MAX;
}

/*!
 * @brief Put the state machine into CLEAR.
 *
 * @param[out] p_state  State to initialise.
 */
void
threat_assessment_init (threat_assessment_t * p_state)
{
    p_state->b_active = false;
    p_state->grip = RTE_GRIP_UNKNOWN;
    threat_assessment_init_lane(&p_state->left);
    threat_assessment_init_lane(&p_state->middle);
    threat_assessment_init_lane(&p_state->right);
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
 * frame starts the alert if a target is already inside a lane's START_MM.
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
        b_cleared = true;
    }

    return (b_cleared);
}

/*!
 * @brief Feed the targets of one valid frame (FR8, FR9, FR10).
 *
 * OPEN-22: every target is evaluated into its lane (left/middle/right, by
 * the RTE_SIDE_* classification radar_monitor.c already assigned) every
 * frame; nothing is excluded for moving away, and no single target is
 * singled out as "the" obstacle. The alert is active whenever any lane is
 * BLOCKED, and reports the nearest distance among the currently blocked
 * lanes.
 *
 * OPEN-23 (13 Nov) then OPEN-25 (14 Nov): TTC is restored per lane, from
 * each lane's own live, per-frame speed_cms this frame, only while that
 * lane has such a live reading (not during its hold-grace window). See
 * threat_assessment_lane_out(). OPEN-25 dropped OPEN-23's original
 * single-lane scoping (TTC only for whichever lane drives d_mm): the
 * nearest lane by distance is not always the one closing in soonest, so
 * scoping to one lane could silently drop a farther, faster-closing
 * lane's TTC from BLE entirely.
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
    threat_assessment_scan_t scan;
    const bool b_was_active = p_state->b_active;

    p_out->b_post_obstacle = false;
    p_out->b_post_clear = false;
    p_out->d_mm = 0u;
    p_out->cue = 0u;

    for (size_t idx = 0u; idx < 3u; idx++)
    {
        // Default every lane to "not BLOCKED" (OPEN-25); only a lane
        // found BLOCKED below gets a real value from
        // threat_assessment_lane_out().
        p_out->lane_d_mm[idx] = RTE_LANE_NONE_MM;
        p_out->lane_ttc_ms[idx] = RTE_TTC_NONE_MS;
    }

    threat_assessment_scan_lanes(p_targets, &scan);
    threat_assessment_hold_lane(&p_state->left, scan.left_mm, now_ms);
    threat_assessment_hold_lane(&p_state->middle, scan.middle_mm, now_ms);
    threat_assessment_hold_lane(&p_state->right, scan.right_mm, now_ms);

    p_state->b_active = p_state->left.b_blocked ||
                        p_state->middle.b_blocked ||
                        p_state->right.b_blocked;

    if (RTE_GRIP_RELEASED == p_state->grip)
    {
        // FR6: only a confirmed release silences the alert. The lanes were
        // still scanned above, so their occupancy stays current; the
        // alert itself is forced off here regardless of what they found.
        p_state->b_active = false;
    }

    if (p_state->b_active)
    {
        // Each lane's last_d_mm never exceeds a target's d_mm, which never
        // exceeds 46339 (threat_assessment_distance_mm()), so the
        // narrowing cast cannot lose bits.
        p_out->b_post_obstacle = true;
        p_out->d_mm = (uint16_t) threat_assessment_pick_lane(p_state);
        p_out->cue = threat_assessment_build_cue(p_state);
        threat_assessment_lane_out(p_state, &scan, p_out);
    }
    else if (b_was_active)
    {
        p_out->b_post_clear = true;
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
        rte_write_ble_obstacle(true, out.lane_d_mm, out.lane_ttc_ms);
    }

    if (out.b_post_clear)
    {
        // out.lane_d_mm/lane_ttc_ms are still every entry's RTE_LANE_
        // NONE_MM/RTE_TTC_NONE_MS pre-set at the top of
        // threat_assessment_update() - threat_assessment_lane_out() only
        // ran while active, which this branch is the opposite of.
        rte_write_audio_msg(RTE_AUDIO_OBSTACLE_CLEAR, 0u, 0u);
        rte_write_ble_obstacle(false, out.lane_d_mm, out.lane_ttc_ms);
    }

    // OPEN-20/OPEN-22: bench-debug only, a no-op unless FEATURE_DEBUG_LOGS
    // and FEATURE_DEBUG_LOG_DIR are both 1. The FR9 direction cue in plain
    // language, plus (OPEN-22) the distance driving loudness/pitch, so
    // neither has to be decoded from the raw byte by hand on the bench.
    // OPEN-19's separate [MIO] line is gone: there is no more "selected
    // object" to report now that MIO selection itself is removed.
    rte_write_dir_debug(p_state->left.b_blocked, p_state->middle.b_blocked,
                        p_state->right.b_blocked, out.b_active, out.d_mm,
                        out.cue);
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
        rte_write_ble_obstacle(false, g_no_lane_d_mm, g_no_lane_ttc_ms);
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
        rte_write_ble_obstacle(false, g_no_lane_d_mm, g_no_lane_ttc_ms);
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
 * OPEN-22 removed every per-target velocity use (departing exclusion, TTC,
 * collision prediction). OPEN-23 (13 Nov) brings back exactly one of
 * those, TTC, called again from threat_assessment_scan_lanes() - using
 * only this function's raw, per-frame speed_cms, never frame-to-frame
 * position differencing (the hazard OPEN-17 removed; see
 * docs/deviations.md OPEN-17 and OPEN-23).
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
 * @brief Time to collision for a target.
 *
 * OPEN-23 (13 Nov): called again, for each BLOCKED lane, using that
 * lane's own live speed_cms this frame (see docs/deviations.md OPEN-23).
 * OPEN-25 (14 Nov): called from threat_assessment_lane_out() now, once
 * per BLOCKED lane, rather than once for a single picked lane. Also kept
 * as its own pure, tested utility.
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
 * @brief Scan this frame's targets: each lane's nearest distance, and
 *        that nearest target's own closing speed (OPEN-23).
 *
 * Every valid target is evaluated, regardless of its speed: OPEN-22
 * deliberately does not exclude a departing target here (nor anywhere
 * else any more) - a target moving away still physically occupies that
 * lane right now, which is what lane occupancy is meant to capture. The
 * closing_cms recorded alongside each lane's nearest_mm (OPEN-23) is
 * purely informational (TTC input), not a gate: it never affects which
 * target counts as nearest, or whether a lane is occupied.
 *
 * @param[in]  p_targets  This frame's three target slots.
 * @param[out] p_scan     Nearest distance and that target's closing speed,
 *                        per lane; UINT32_MAX distance (closing_cms
 *                        meaningless, left 0) for a lane with no valid
 *                        target this frame.
 */
static void
threat_assessment_scan_lanes (const rte_radar_target_t * p_targets,
                              threat_assessment_scan_t * p_scan)
{
    p_scan->left_mm = UINT32_MAX;
    p_scan->left_closing_cms = 0;
    p_scan->middle_mm = UINT32_MAX;
    p_scan->middle_closing_cms = 0;
    p_scan->right_mm = UINT32_MAX;
    p_scan->right_closing_cms = 0;

    for (size_t idx = 0u; idx < RTE_RADAR_NUM_TARGETS; idx++)
    {
        if (p_targets[idx].b_valid)
        {
            uint32_t d_mm = (uint32_t) p_targets[idx].d_mm;
            int32_t closing_cms =
                threat_assessment_closing_cms(&p_targets[idx]);

            if (RTE_SIDE_LEFT == p_targets[idx].side)
            {
                if (d_mm < p_scan->left_mm)
                {
                    p_scan->left_mm = d_mm;
                    p_scan->left_closing_cms = closing_cms;
                }
            }
            else if (RTE_SIDE_RIGHT == p_targets[idx].side)
            {
                if (d_mm < p_scan->right_mm)
                {
                    p_scan->right_mm = d_mm;
                    p_scan->right_closing_cms = closing_cms;
                }
            }
            else
            {
                if (d_mm < p_scan->middle_mm)
                {
                    p_scan->middle_mm = d_mm;
                    p_scan->middle_closing_cms = closing_cms;
                }
            }
        }
    }
}

/*!
 * @brief Apply START/STOP hysteresis and the lane hold timer to one lane.
 *
 * FR8/NFR8, per lane: BLOCKED once something in the lane is within
 * THREAT_ASSESSMENT_START_MM; stays BLOCKED, with whatever distance was
 * last confirmed, while a target remains anywhere inside
 * THREAT_ASSESSMENT_STOP_MM. OPEN-22: once nothing in the lane is within
 * STOP_MM - whether because every target there physically cleared it, it
 * jumped outward in one frame, or the sensor simply stopped reporting one
 * there this frame - the lane only actually clears after that has been
 * true for THREAT_ASSESSMENT_LANE_HOLD_MS in a row. This single hold
 * timer is what used to be two separate mechanisms (the away-jump gate
 * and the global no-target hold): both looked the same from a lane's
 * point of view (last_d_mm stops being refreshed), so one timer absorbs
 * both.
 *
 * @param[in,out] p_lane      This lane's persistent state.
 * @param[in]     nearest_mm  Nearest valid target in this lane this frame,
 *                             or UINT32_MAX if none.
 * @param[in]     now_ms      Time this frame was received.
 */
static void
threat_assessment_hold_lane (threat_assessment_lane_t * p_lane,
                             uint32_t nearest_mm, uint32_t now_ms)
{
    if (nearest_mm <= THREAT_ASSESSMENT_START_MM)
    {
        p_lane->b_blocked = true;
        p_lane->last_d_mm = nearest_mm;
        p_lane->hold_pending_ms = UINT32_MAX;
    }
    else if (nearest_mm <= THREAT_ASSESSMENT_STOP_MM)
    {
        // Between START and STOP: a target is still confirmed present in
        // the lane. Keep whatever BLOCKED state the lane already had (the
        // main hysteresis band), and cancel any pending clear.
        if (p_lane->b_blocked)
        {
            p_lane->last_d_mm = nearest_mm;
            p_lane->hold_pending_ms = UINT32_MAX;
        }
    }
    else if (p_lane->b_blocked)
    {
        // Beyond STOP_MM, or no valid target in this lane at all this
        // frame (nearest_mm is UINT32_MAX, which is > STOP_MM either way).
        if (UINT32_MAX == p_lane->hold_pending_ms)
        {
            p_lane->hold_pending_ms = now_ms;
        }
        else
        {
            // Unsigned subtraction stays correct when the clock wraps.
            uint32_t held_ms = now_ms - p_lane->hold_pending_ms;

            if (held_ms >= THREAT_ASSESSMENT_LANE_HOLD_MS)
            {
                p_lane->b_blocked = false;
                p_lane->hold_pending_ms = UINT32_MAX;
            }
        }
    }
    else
    {
        // Already open: nothing to do.
    }
}

/*!
 * @brief Nearest distance among the currently BLOCKED lanes.
 *
 * Drives the FR9 OBSTACLE(d_mm) distance (loudness/pitch in Brian's
 * alert.c): the nearest thing the user is actually being warned about,
 * even while other, farther lanes are also BLOCKED. OPEN-25 (14 Nov):
 * no longer also picks out a single lane's TTC for BLE - see
 * threat_assessment_lane_out(), which reports every BLOCKED lane's TTC
 * instead of just this one.
 *
 * @param[in] p_state  Decision state.
 *
 * @return Distance in mm. Only meaningful when p_state->b_active is true
 *         (some lane is BLOCKED); undefined (UINT32_MAX) otherwise, but
 *         the caller only calls this while active.
 */
static uint32_t
threat_assessment_pick_lane (const threat_assessment_t * p_state)
{
    uint32_t nearest_mm = UINT32_MAX;

    if (p_state->left.b_blocked && (p_state->left.last_d_mm < nearest_mm))
    {
        nearest_mm = p_state->left.last_d_mm;
    }

    if (p_state->middle.b_blocked &&
        (p_state->middle.last_d_mm < nearest_mm))
    {
        nearest_mm = p_state->middle.last_d_mm;
    }

    if (p_state->right.b_blocked && (p_state->right.last_d_mm < nearest_mm))
    {
        nearest_mm = p_state->right.last_d_mm;
    }

    return (nearest_mm);
}

/*!
 * @brief Fill every BLOCKED lane's distance and TTC for BLE (OPEN-25).
 *
 * Unlike threat_assessment_pick_lane() (one distance, for FR9 audio),
 * this reports all three lanes independently: a farther lane closing in
 * faster than the nearest one would otherwise never reach BLE at all.
 * p_out's lane_d_mm/lane_ttc_ms arrays are pre-set to RTE_LANE_NONE_MM/
 * RTE_TTC_NONE_MS by the caller (threat_assessment_update()) before this
 * runs, so only a BLOCKED lane needs writing here; a lane with no live
 * reading this frame (held via its hold-grace window) keeps its real
 * last_d_mm but RTE_TTC_NONE_MS for ttc_ms, same rule OPEN-23 used for
 * the single picked lane.
 *
 * @param[in]  p_state  Decision state.
 * @param[in]  p_scan   This frame's scan result (OPEN-23).
 * @param[out] p_out    lane_d_mm/lane_ttc_ms updated for each BLOCKED
 *                      lane.
 */
static void
threat_assessment_lane_out (const threat_assessment_t * p_state,
                            const threat_assessment_scan_t * p_scan,
                            threat_assessment_out_t * p_out)
{
    if (p_state->left.b_blocked)
    {
        // Same 46339 mm bound as threat_assessment_pick_lane(), so the
        // narrowing cast cannot lose bits.
        p_out->lane_d_mm[RTE_SIDE_LEFT] = (uint16_t) p_state->left.last_d_mm;

        if (UINT32_MAX != p_scan->left_mm)
        {
            p_out->lane_ttc_ms[RTE_SIDE_LEFT] = threat_assessment_ttc_ms(
                p_state->left.last_d_mm, p_scan->left_closing_cms);
        }
    }

    if (p_state->middle.b_blocked)
    {
        p_out->lane_d_mm[RTE_SIDE_CENTER] =
            (uint16_t) p_state->middle.last_d_mm;

        if (UINT32_MAX != p_scan->middle_mm)
        {
            p_out->lane_ttc_ms[RTE_SIDE_CENTER] = threat_assessment_ttc_ms(
                p_state->middle.last_d_mm, p_scan->middle_closing_cms);
        }
    }

    if (p_state->right.b_blocked)
    {
        p_out->lane_d_mm[RTE_SIDE_RIGHT] =
            (uint16_t) p_state->right.last_d_mm;

        if (UINT32_MAX != p_scan->right_mm)
        {
            p_out->lane_ttc_ms[RTE_SIDE_RIGHT] = threat_assessment_ttc_ms(
                p_state->right.last_d_mm, p_scan->right_closing_cms);
        }
    }
}

/*!
 * @brief Build the FR9 cue byte for the currently active alert.
 *
 * @param[in] p_state  Decision state.
 *
 * @return The byte for rte_audio_msg_t.cue (0 if called while not active;
 *         not expected, the caller only calls this when p_state->b_active).
 */
static uint8_t
threat_assessment_build_cue (const threat_assessment_t * p_state)
{
    return (threat_assessment_direction_cue(p_state->left.b_blocked,
                                            p_state->middle.b_blocked,
                                            p_state->right.b_blocked));
}

/*!
 * @brief Map the three lanes' occupancy to the FR9 direction cue.
 *
 * Full 8-row table (OPEN-20, agreed with Putra 11 Nov, unchanged by
 * OPEN-22's lane redesign): straight ahead is preferred whenever it is
 * safe, so a single lane being blocked (with the other two open) just
 * holds CENTER rather than steering needlessly; the only lane that,
 * blocked alone, forces a choice is the middle itself, since both outer
 * lanes are then equally open - that genuine tie defaults to LEFT (an
 * arbitrary but fixed choice, not derived from the sensor).
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
 * @param[in] b_left_blocked    Left lane occupied within range.
 * @param[in] b_middle_blocked  Middle lane occupied within range.
 * @param[in] b_right_blocked   Right lane occupied within range.
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

/*** end of file ***/
