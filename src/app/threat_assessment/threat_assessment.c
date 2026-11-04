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

static bool threat_assessment_find_nearest (
    const rte_radar_target_t * p_targets, bool b_include_departing,
    size_t skip_index, size_t speed_exempt_index, size_t * p_index,
    uint32_t * p_d_mm);
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

    for (size_t idx = 0u; idx < THREAT_ASSESSMENT_HISTORY_LEN; idx++)
    {
        p_state->hist_d_mm[idx] = 0u;
        p_state->hist_ms[idx] = 0u;
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

    p_out->b_post_obstacle = false;
    p_out->b_post_clear = false;
    p_out->d_mm = 0u;
    p_out->ttc_ms = RTE_TTC_NONE_MS;

    // Nearest valid target of any speed: its range feeds the trend.
    b_any_valid = threat_assessment_find_nearest(
        p_targets, true, RTE_RADAR_NUM_TARGETS, RTE_RADAR_NUM_TARGETS,
        &any_index, &any_d_mm);

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

        b_found = threat_assessment_find_nearest(
            p_targets, false,
            b_trend_departing ? any_index : RTE_RADAR_NUM_TARGETS,
            speed_exempt_index, &nearest_index, &nearest_d_mm);

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
        p_out->b_post_clear = b_was_active;
    }

    if (p_state->b_active)
    {
        // The distance comes from threat_assessment_distance_mm(), which
        // never exceeds 46339, so the narrowing cast cannot lose bits.
        p_out->b_post_obstacle = true;
        p_out->d_mm = (uint16_t) p_state->last_d_mm;
        p_out->ttc_ms = p_state->last_ttc_ms;
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
        rte_write_audio_msg(RTE_AUDIO_OBSTACLE, out.d_mm);
        rte_write_ble_obstacle(true, out.d_mm, out.ttc_ms);
    }

    if (out.b_post_clear)
    {
        rte_write_audio_msg(RTE_AUDIO_OBSTACLE_CLEAR, 0u);
        rte_write_ble_obstacle(false, 0u, RTE_TTC_NONE_MS);
    }
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
        rte_write_audio_msg(RTE_AUDIO_OBSTACLE_CLEAR, 0u);
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
        rte_write_audio_msg(RTE_AUDIO_OBSTACLE_CLEAR, 0u);
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
 * @brief Find the nearest valid target (FR8, FR13).
 *
 * @param[in]  p_targets           The three target slots.
 * @param[in]  b_include_departing  false skips targets departing by speed.
 * @param[in]  skip_index          Slot to ignore (RTE_RADAR_NUM_TARGETS: none).
 * @param[in]  speed_exempt_index  Slot whose speed verdict is ignored (none: RTE_RADAR_NUM_TARGETS).
 * @param[out] p_index    Index of the nearest target.
 * @param[out] p_d_mm     Its distance in mm.
 *
 * @return true when such a target exists.
 */
static bool
threat_assessment_find_nearest (const rte_radar_target_t * p_targets,
                                bool b_include_departing, size_t skip_index,
                                size_t speed_exempt_index, size_t * p_index,
                                uint32_t * p_d_mm)
{
    bool b_found = false;

    for (size_t idx = 0u; idx < RTE_RADAR_NUM_TARGETS; idx++)
    {
        bool b_ok_speed = b_include_departing || (idx == speed_exempt_index) ||
                          (!threat_assessment_is_departing(&p_targets[idx]));

        if ((p_targets[idx].b_valid) && (idx != skip_index) && b_ok_speed)
        {
            uint32_t d_mm = threat_assessment_distance_mm(&p_targets[idx]);

            if ((!b_found) || (d_mm < *p_d_mm))
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
            p_out->b_post_clear = true;
        }
    }
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
