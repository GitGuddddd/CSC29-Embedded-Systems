/** @file threat_assessment.c
 *
 * @brief Glasses obstacle decision: nearest target, hysteresis, TTC, pitch.
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

// Highest power of four that fits in 32 bits, start of the square root.
#define THREAT_ASSESSMENT_ISQRT_START_BIT    (0x40000000u)
#define THREAT_ASSESSMENT_ISQRT_BIT_STEP     (2u)

static uint32_t threat_assessment_isqrt (uint32_t value);
static bool threat_assessment_find_nearest (
    const rte_radar_target_t * p_targets, bool b_include_departing,
    size_t skip_index, size_t * p_index, uint32_t * p_d_mm);
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

    for (size_t idx = 0u; idx < THREAT_ASSESSMENT_HISTORY_LEN; idx++)
    {
        p_state->hist_d_mm[idx] = 0u;
        p_state->hist_ms[idx] = 0u;
    }
}

/*!
 * @brief Radar fault: force CLEAR so a stale tone can never stay on (FR10).
 *
 * @param[in,out] p_state  State to reset.
 *
 * @return true when an obstacle was active (the caller posts CLEAR).
 */
bool
threat_assessment_reset (threat_assessment_t * p_state)
{
    bool b_was_active = p_state->b_active;

    threat_assessment_init(p_state);

    return (b_was_active);
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

    p_out->b_post_obstacle = false;
    p_out->b_post_clear = false;
    p_out->d_mm = 0u;
    p_out->ttc_ms = RTE_TTC_NONE_MS;

    // Nearest valid target of any speed: its range feeds the trend.
    b_any_valid = threat_assessment_find_nearest(
        p_targets, true, RTE_RADAR_NUM_TARGETS, &any_index, &any_d_mm);

    if (b_any_valid)
    {
        b_trend_departing = threat_assessment_trend_away(
            p_state, any_d_mm, now_ms);
        threat_assessment_push_history(p_state, any_d_mm, now_ms);
    }
    else
    {
        p_state->hist_count = 0u;
    }

    if (threat_assessment_find_nearest(
            p_targets, false,
            b_trend_departing ? any_index : RTE_RADAR_NUM_TARGETS,
            &nearest_index, &nearest_d_mm))
    {
        p_state->depart_frames = 0u;
        threat_assessment_on_target(p_state, &p_targets[nearest_index],
                                        nearest_d_mm, now_ms, p_out);
    }
    else
    {
        // Only a real range trend counts: the sensor speed alone can read
        // 24 cm/s on a target that has not moved (bench log 2026-10-18).
        if (b_any_valid && b_trend_departing)
        {
            if (p_state->depart_frames < THREAT_ASSESSMENT_DEPART_CLEAR_FRAMES)
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
 * @brief Straight-line distance from the sensor to a target.
 *
 * @param[in] p_target  The target.
 *
 * @return Distance in mm, at most 46339.
 */
uint16_t
threat_assessment_distance_mm (const rte_radar_target_t * p_target)
{
    int32_t x_mm = p_target->x_mm;
    int32_t y_mm = p_target->y_mm;

    // The products are at most 2^30 each, so they fit in int32_t and are
    // never negative; the casts to uint32_t only change the type.
    uint32_t sum_sq = (uint32_t) (x_mm * x_mm) + (uint32_t) (y_mm * y_mm);

    // The root of a value below 2^31 is at most 46339, which fits uint16_t.
    return ((uint16_t) threat_assessment_isqrt(sum_sq));
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
 * @brief Buzzer pitch for an obstacle distance (FR9).
 *
 * Linear map: 2000 mm gives 1500 Hz and 300 mm gives 3500 Hz. Distances
 * outside that range are clamped.
 *
 * @param[in] d_mm  Distance in mm.
 *
 * @return Pitch in Hz.
 */
uint16_t
threat_assessment_pitch_hz (uint32_t d_mm)
{
    const uint32_t span_mm = THREAT_ASSESSMENT_PITCH_FAR_MM -
                             THREAT_ASSESSMENT_PITCH_NEAR_MM;
    const uint32_t span_hz = THREAT_ASSESSMENT_PITCH_NEAR_HZ -
                             THREAT_ASSESSMENT_PITCH_FAR_HZ;
    uint32_t pitch_hz = THREAT_ASSESSMENT_PITCH_FAR_HZ;

    if (d_mm <= THREAT_ASSESSMENT_PITCH_NEAR_MM)
    {
        pitch_hz = THREAT_ASSESSMENT_PITCH_NEAR_HZ;
    }
    else if (d_mm < THREAT_ASSESSMENT_PITCH_FAR_MM)
    {
        pitch_hz += ((THREAT_ASSESSMENT_PITCH_FAR_MM - d_mm) * span_hz) /
                    span_mm;
    }
    else
    {
        // At or beyond the far edge the pitch stays at the lowest value.
        pitch_hz = THREAT_ASSESSMENT_PITCH_FAR_HZ;
    }

    // The cast is safe: the result is between 1500 and 3500.
    return ((uint16_t) pitch_hz);
}

/*!
 * @brief Integer square root (no math library).
 *
 * @param[in] value  Radicand.
 *
 * @return Largest r with r * r <= value.
 */
static uint32_t
threat_assessment_isqrt (uint32_t value)
{
    uint32_t remainder = value;
    uint32_t result = 0u;
    uint32_t bit = THREAT_ASSESSMENT_ISQRT_START_BIT;

    while (bit > remainder)
    {
        bit >>= THREAT_ASSESSMENT_ISQRT_BIT_STEP;
    }

    while (0u != bit)
    {
        if (remainder >= (result + bit))
        {
            remainder -= (result + bit);
            result = (result >> 1u) + bit;
        }
        else
        {
            result >>= 1u;
        }

        bit >>= THREAT_ASSESSMENT_ISQRT_BIT_STEP;
    }

    return (result);
}

/*!
 * @brief Find the nearest valid target (FR8, FR13).
 *
 * @param[in]  p_targets           The three target slots.
 * @param[in]  b_include_departing  false skips targets departing by speed.
 * @param[in]  skip_index          Slot to ignore (RTE_RADAR_NUM_TARGETS: none).
 * @param[out] p_index    Index of the nearest target.
 * @param[out] p_d_mm     Its distance in mm.
 *
 * @return true when such a target exists.
 */
static bool
threat_assessment_find_nearest (const rte_radar_target_t * p_targets,
                                bool b_include_departing, size_t skip_index,
                                size_t * p_index, uint32_t * p_d_mm)
{
    bool b_found = false;

    for (size_t idx = 0u; idx < RTE_RADAR_NUM_TARGETS; idx++)
    {
        bool b_ok_speed = b_include_departing ||
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
