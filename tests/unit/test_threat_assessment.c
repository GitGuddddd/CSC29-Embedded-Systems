/** @file test_threat_assessment.c
 *
 * @brief Unit tests of the obstacle decision (FR6, FR8, FR9, FR10).
 *
 * @par
 * COPYRIGHT NOTICE: (c) 2026 Group 29. All rights reserved.
 *
 * OPEN-22 (13 Nov): rewritten for the per-lane occupancy redesign. The
 * MIO-selection/tie-break (OPEN-16), collision-prediction (OPEN-17),
 * track-ID (OPEN-18) and departing-trend/jump-gate tests are gone along
 * with the code they tested; the direction-cue (OPEN-20) and grip (FR6)
 * tests carry over unchanged, since neither was touched by OPEN-22.
 */

#include <stdbool.h>
#include <stdint.h>

#include "unity.h"

#include "mock_hal.h"
#include "rte_type.h"
#include "threat_assessment.h"

static threat_assessment_t g_state;

static rte_radar_target_t
make_target_side (int16_t d_mm, int16_t speed_cms, rte_cue_side_t side)
{
    rte_radar_target_t target = { true, 0, 0, (uint16_t) d_mm, speed_cms,
                                  side };

    return (target);
}

static rte_radar_target_t
make_target (int16_t d_mm, int16_t speed_cms)
{
    return (make_target_side(d_mm, speed_cms, RTE_SIDE_CENTER));
}

// Fills the three slots: slot 0 = given target, the other two empty.
static void
make_frame (rte_radar_target_t * p_targets, int16_t d_mm, int16_t speed_cms)
{
    rte_radar_target_t empty = { false, 0, 0, 0u, 0, RTE_SIDE_CENTER };

    p_targets[0] = make_target(d_mm, speed_cms);
    p_targets[1] = empty;
    p_targets[2] = empty;
}

static void
make_empty_frame (rte_radar_target_t * p_targets)
{
    rte_radar_target_t empty = { false, 0, 0, 0u, 0, RTE_SIDE_CENTER };

    p_targets[0] = empty;
    p_targets[1] = empty;
    p_targets[2] = empty;
}

static rte_cue_side_t
cue_side (uint8_t cue)
{
    return ((rte_cue_side_t) ((cue >> RTE_CUE_SIDE_SHIFT) &
                              RTE_CUE_SIDE_MASK));
}

void
setUp (void)
{
    mock_reset();
    mock_rte_reset();
    threat_assessment_init(&g_state);
}

void
tearDown (void)
{
}

static void
test_distance_is_read_from_the_target (void)
{
    rte_radar_target_t front = make_target(526, 0);

    TEST_ASSERT_EQUAL_UINT16(526u, threat_assessment_distance_mm(&front));
}

static void
test_closing_cms_sign (void)
{
    rte_radar_target_t approaching = make_target(1000, -16);
    rte_radar_target_t departing = make_target(1000, 16);

    TEST_ASSERT_EQUAL_INT32(16, threat_assessment_closing_cms(&approaching));
    TEST_ASSERT_EQUAL_INT32(-16, threat_assessment_closing_cms(&departing));
}

static void
test_ttc (void)
{
    TEST_ASSERT_EQUAL_UINT16(2000u, threat_assessment_ttc_ms(1000u, 50));
    TEST_ASSERT_EQUAL_UINT16(RTE_TTC_NONE_MS,
                             threat_assessment_ttc_ms(1000u, 0));
    TEST_ASSERT_EQUAL_UINT16(RTE_TTC_NONE_MS,
                             threat_assessment_ttc_ms(1000u, -5));
    TEST_ASSERT_EQUAL_UINT16(0xFFFEu, threat_assessment_ttc_ms(46339u, 1));
}

static void
test_stationary_object_inside_range_raises_alert (void)
{
    rte_radar_target_t targets[RTE_RADAR_NUM_TARGETS];
    threat_assessment_out_t out;

    make_frame(targets, 526, 0);
    threat_assessment_update(&g_state, targets, 0u, &out);

    TEST_ASSERT_TRUE(out.b_post_obstacle);
    TEST_ASSERT_EQUAL_UINT16(526u, out.d_mm);
    TEST_ASSERT_EQUAL_UINT16(RTE_TTC_NONE_MS,
                             out.lane_ttc_ms[RTE_SIDE_CENTER]);
}

static void
test_alert_starts_at_1800_and_not_above (void)
{
    rte_radar_target_t targets[RTE_RADAR_NUM_TARGETS];
    threat_assessment_out_t out;

    make_frame(targets, 1801, 0);
    threat_assessment_update(&g_state, targets, 0u, &out);
    TEST_ASSERT_FALSE(out.b_active);

    make_frame(targets, 1800, 0);
    threat_assessment_update(&g_state, targets, 140u, &out);
    TEST_ASSERT_TRUE(out.b_active);
}

static void
test_hysteresis_keeps_blocked_between_1800_and_1950 (void)
{
    rte_radar_target_t targets[RTE_RADAR_NUM_TARGETS];
    threat_assessment_out_t out;

    make_frame(targets, 1700, 0);
    threat_assessment_update(&g_state, targets, 0u, &out);
    TEST_ASSERT_TRUE(out.b_active);

    // 1950 mm: at THREAT_ASSESSMENT_STOP_MM, not above it, so the lane is
    // still confirmed BLOCKED (no hold pending).
    make_frame(targets, 1950, 0);
    threat_assessment_update(&g_state, targets, 140u, &out);
    TEST_ASSERT_TRUE(out.b_active);
    TEST_ASSERT_FALSE(out.b_post_clear);
}

static void
test_crossing_1950_clears_only_after_the_lane_hold (void)
{
    rte_radar_target_t targets[RTE_RADAR_NUM_TARGETS];
    threat_assessment_out_t out;

    make_frame(targets, 1700, 0);
    threat_assessment_update(&g_state, targets, 0u, &out);
    TEST_ASSERT_TRUE(out.b_active);

    // Crosses above STOP_MM: the hold timer starts, the lane does not
    // clear on this same frame (OPEN-22: this is the behaviour that used
    // to need a separate away-jump gate).
    make_frame(targets, 1951, 0);
    threat_assessment_update(&g_state, targets, 140u, &out);
    TEST_ASSERT_TRUE(out.b_active);
    TEST_ASSERT_FALSE(out.b_post_clear);

    // 160 ms into the 200 ms hold: still held.
    make_frame(targets, 1951, 0);
    threat_assessment_update(&g_state, targets, 300u, &out);
    TEST_ASSERT_TRUE(out.b_active);

    // 201 ms since the hold started (at t=140): now clears.
    make_frame(targets, 1951, 0);
    threat_assessment_update(&g_state, targets, 341u, &out);
    TEST_ASSERT_FALSE(out.b_active);
    TEST_ASSERT_TRUE(out.b_post_clear);
    TEST_ASSERT_FALSE(out.b_post_obstacle);

    // The clear is sent exactly once.
    threat_assessment_update(&g_state, targets, 500u, &out);
    TEST_ASSERT_FALSE(out.b_post_clear);
}

static void
test_lane_hold_absorbs_target_vanishing (void)
{
    rte_radar_target_t targets[RTE_RADAR_NUM_TARGETS];
    threat_assessment_out_t out;

    make_frame(targets, 1500, 0);
    threat_assessment_update(&g_state, targets, 0u, &out);
    TEST_ASSERT_TRUE(out.b_active);

    // The target disappears entirely (sensor dropout or a one-frame
    // outward jump both look the same from here): the hold starts now.
    make_empty_frame(targets);
    threat_assessment_update(&g_state, targets, 50u, &out);
    TEST_ASSERT_TRUE(out.b_active);
    TEST_ASSERT_EQUAL_UINT16(1500u, out.d_mm);

    // 199 ms after the hold started: still held.
    threat_assessment_update(&g_state, targets, 249u, &out);
    TEST_ASSERT_TRUE(out.b_active);

    // 200 ms after the hold started: clears.
    threat_assessment_update(&g_state, targets, 250u, &out);
    TEST_ASSERT_FALSE(out.b_active);
    TEST_ASSERT_TRUE(out.b_post_clear);
}

static void
test_lane_hold_is_cancelled_if_the_target_reappears (void)
{
    rte_radar_target_t targets[RTE_RADAR_NUM_TARGETS];
    threat_assessment_out_t out;

    make_frame(targets, 1500, 0);
    threat_assessment_update(&g_state, targets, 0u, &out);

    make_empty_frame(targets);
    threat_assessment_update(&g_state, targets, 50u, &out);
    TEST_ASSERT_TRUE(out.b_active);

    // The target reappears within the hold window: the hold is cancelled
    // and the lane's distance is refreshed, not left stale.
    make_frame(targets, 1600, 0);
    threat_assessment_update(&g_state, targets, 100u, &out);
    TEST_ASSERT_TRUE(out.b_active);
    TEST_ASSERT_EQUAL_UINT16(1600u, out.d_mm);

    // It disappears again: this starts a brand-new 200 ms hold from t=250,
    // not a continuation of the first one.
    make_empty_frame(targets);
    threat_assessment_update(&g_state, targets, 250u, &out);
    TEST_ASSERT_TRUE(out.b_active);

    threat_assessment_update(&g_state, targets, 449u, &out);
    TEST_ASSERT_TRUE(out.b_active);

    threat_assessment_update(&g_state, targets, 450u, &out);
    TEST_ASSERT_FALSE(out.b_active);
}

static void
test_clock_wrap_does_not_break_the_lane_hold_timer (void)
{
    rte_radar_target_t targets[RTE_RADAR_NUM_TARGETS];
    threat_assessment_out_t out;

    make_frame(targets, 1500, 0);
    threat_assessment_update(&g_state, targets, 0xFFFFFF50u, &out);
    TEST_ASSERT_TRUE(out.b_active);

    // The hold starts 100 ms before the 32-bit clock wraps.
    make_empty_frame(targets);
    threat_assessment_update(&g_state, targets, 4294967196u, &out);
    TEST_ASSERT_TRUE(out.b_active);

    // 150 ms later, across the wrap: the hold is not over yet.
    threat_assessment_update(&g_state, targets, 50u, &out);
    TEST_ASSERT_TRUE(out.b_active);

    // 300 ms since the hold started: now it clears.
    threat_assessment_update(&g_state, targets, 200u, &out);
    TEST_ASSERT_FALSE(out.b_active);
    TEST_ASSERT_TRUE(out.b_post_clear);
}

static void
test_departing_target_still_blocks_its_lane (void)
{
    rte_radar_target_t targets[RTE_RADAR_NUM_TARGETS];
    threat_assessment_out_t out;

    // Moving away fast (speed_cms positive means departing, A7). OPEN-22
    // deliberately does not exclude it: it still physically occupies the
    // lane right now, so the alert stays active until it actually clears
    // THREAT_ASSESSMENT_STOP_MM.
    make_frame(targets, 1500, 50);
    threat_assessment_update(&g_state, targets, 0u, &out);
    TEST_ASSERT_TRUE(out.b_active);
    TEST_ASSERT_EQUAL_UINT16(1500u, out.d_mm);
}

static void
test_nearest_blocked_lane_drives_d_mm (void)
{
    rte_radar_target_t targets[RTE_RADAR_NUM_TARGETS];
    threat_assessment_out_t out;

    targets[0] = make_target_side(1600, 0, RTE_SIDE_LEFT);
    targets[1] = make_target_side(1200, 0, RTE_SIDE_RIGHT);
    targets[2] = make_target_side(2200, 0, RTE_SIDE_CENTER);
    threat_assessment_update(&g_state, targets, 0u, &out);

    TEST_ASSERT_TRUE(out.b_active);
    TEST_ASSERT_EQUAL_UINT16(1200u, out.d_mm);
}

static void
test_ttc_uses_live_speed_of_its_own_lane (void)
{
    rte_radar_target_t targets[RTE_RADAR_NUM_TARGETS];
    threat_assessment_out_t out;

    // OPEN-23/OPEN-25: a live reading this frame (not held), so the
    // CENTER lane's ttc_ms is computed directly from this frame's own
    // speed_cms. make_frame() always puts its target in CENTER.
    make_frame(targets, 1000, -20);
    threat_assessment_update(&g_state, targets, 0u, &out);

    TEST_ASSERT_TRUE(out.b_active);
    TEST_ASSERT_EQUAL_UINT16(1000u, out.d_mm);
    TEST_ASSERT_EQUAL_UINT16(threat_assessment_ttc_ms(1000u, 20),
                             out.lane_ttc_ms[RTE_SIDE_CENTER]);
}

static void
test_ttc_is_none_while_the_lane_is_only_held (void)
{
    rte_radar_target_t targets[RTE_RADAR_NUM_TARGETS];
    threat_assessment_out_t out;

    make_frame(targets, 1500, -20);
    threat_assessment_update(&g_state, targets, 0u, &out);
    TEST_ASSERT_TRUE(out.lane_ttc_ms[RTE_SIDE_CENTER] < RTE_TTC_NONE_MS);

    // The target vanishes: the lane stays BLOCKED (and keeps reporting
    // its last distance) purely via its hold timer, with no live
    // speed_cms this frame - OPEN-23/OPEN-25 must not compute a TTC from
    // a reading that is no longer current.
    make_empty_frame(targets);
    threat_assessment_update(&g_state, targets, 50u, &out);
    TEST_ASSERT_TRUE(out.b_active);
    TEST_ASSERT_EQUAL_UINT16(1500u, out.d_mm);
    TEST_ASSERT_EQUAL_UINT16(RTE_TTC_NONE_MS,
                             out.lane_ttc_ms[RTE_SIDE_CENTER]);
}

static void
test_ttc_is_none_for_a_departing_target (void)
{
    rte_radar_target_t targets[RTE_RADAR_NUM_TARGETS];
    threat_assessment_out_t out;

    // Departing (positive speed_cms, A7) but still inside STOP_MM, so the
    // lane is live and BLOCKED (OPEN-22 does not exclude it) - but
    // threat_assessment_ttc_ms() itself returns NONE for a non-approaching
    // closing speed, live reading or not.
    make_frame(targets, 1500, 30);
    threat_assessment_update(&g_state, targets, 0u, &out);

    TEST_ASSERT_TRUE(out.b_active);
    TEST_ASSERT_EQUAL_UINT16(1500u, out.d_mm);
    TEST_ASSERT_EQUAL_UINT16(RTE_TTC_NONE_MS,
                             out.lane_ttc_ms[RTE_SIDE_CENTER]);
}

static void
test_ttc_holds_a_brief_speed_dropout (void)
{
    rte_radar_target_t targets[RTE_RADAR_NUM_TARGETS];
    threat_assessment_out_t out;

    // OPEN-26: a real approaching reading establishes a held speed.
    make_frame(targets, 1500, -20);
    threat_assessment_update(&g_state, targets, 0u, &out);
    TEST_ASSERT_TRUE(out.lane_ttc_ms[RTE_SIDE_CENTER] < RTE_TTC_NONE_MS);

    // The lane is still live every frame (d_mm keeps updating, unlike
    // test_ttc_is_none_while_the_lane_is_only_held's vanished target) -
    // but this one frame's own speed_cms is unusable. 50 ms later, well
    // inside the hold window: ttc_ms must use the held closing speed
    // (20 cm/s) together with THIS frame's distance (1490 mm), not the
    // stale distance from the first frame.
    make_frame(targets, 1490, 0);
    threat_assessment_update(&g_state, targets, 50u, &out);
    TEST_ASSERT_TRUE(out.b_active);
    TEST_ASSERT_EQUAL_UINT16(1490u, out.d_mm);
    TEST_ASSERT_EQUAL_UINT16(threat_assessment_ttc_ms(1490u, 20),
                             out.lane_ttc_ms[RTE_SIDE_CENTER]);
}

static void
test_ttc_drops_to_none_once_the_speed_hold_expires (void)
{
    rte_radar_target_t targets[RTE_RADAR_NUM_TARGETS];
    threat_assessment_out_t out;

    make_frame(targets, 1500, -20);
    threat_assessment_update(&g_state, targets, 0u, &out);

    // Speed reading goes unusable starting at t=50 (the hold's pending
    // timer starts here).
    make_frame(targets, 1490, 0);
    threat_assessment_update(&g_state, targets, 50u, &out);
    TEST_ASSERT_TRUE(out.lane_ttc_ms[RTE_SIDE_CENTER] < RTE_TTC_NONE_MS);

    // 201 ms after the dropout started (50 + 201 = 251): past
    // THREAT_ASSESSMENT_LANE_HOLD_MS (200 ms), same window the existing
    // occupancy hold uses. A brief dropout is smoothed over; a sustained
    // one must still end up at NONE, not hold forever.
    make_frame(targets, 1480, 0);
    threat_assessment_update(&g_state, targets, 251u, &out);
    TEST_ASSERT_TRUE(out.b_active);
    TEST_ASSERT_EQUAL_UINT16(1480u, out.d_mm);
    TEST_ASSERT_EQUAL_UINT16(RTE_TTC_NONE_MS,
                             out.lane_ttc_ms[RTE_SIDE_CENTER]);
}

static void
test_ttc_hold_does_not_resume_across_a_full_occupancy_gap (void)
{
    rte_radar_target_t targets[RTE_RADAR_NUM_TARGETS];
    threat_assessment_out_t out;

    make_frame(targets, 1500, -20);
    threat_assessment_update(&g_state, targets, 0u, &out);
    TEST_ASSERT_TRUE(out.lane_ttc_ms[RTE_SIDE_CENTER] < RTE_TTC_NONE_MS);

    // The target vanishes entirely (no live reading at all this frame,
    // unlike the two tests above where it stayed live with a bad speed
    // reading) - the lane stays BLOCKED via hold_pending_ms, and this
    // must clear the held speed too, not just report NONE for now.
    make_empty_frame(targets);
    threat_assessment_update(&g_state, targets, 50u, &out);
    TEST_ASSERT_EQUAL_UINT16(RTE_TTC_NONE_MS,
                             out.lane_ttc_ms[RTE_SIDE_CENTER]);

    // A new live reading reappears, itself unusable (speed 0). If the
    // speed hold had not been cleared above, this would wrongly reuse
    // the first frame's 20 cm/s from a completely different approach.
    make_frame(targets, 1400, 0);
    threat_assessment_update(&g_state, targets, 100u, &out);
    TEST_ASSERT_TRUE(out.b_active);
    TEST_ASSERT_EQUAL_UINT16(1400u, out.d_mm);
    TEST_ASSERT_EQUAL_UINT16(RTE_TTC_NONE_MS,
                             out.lane_ttc_ms[RTE_SIDE_CENTER]);
}

static void
test_two_independent_lane_hold_timers_overlap (void)
{
    rte_radar_target_t targets[RTE_RADAR_NUM_TARGETS];
    rte_radar_target_t empty = { false, 0, 0, 0u, 0, RTE_SIDE_CENTER };
    threat_assessment_out_t out;

    // Left and right both blocked, left nearer: d_mm comes from left.
    targets[0] = make_target_side(1500, 0, RTE_SIDE_LEFT);
    targets[1] = make_target_side(1600, 0, RTE_SIDE_RIGHT);
    targets[2] = empty;
    threat_assessment_update(&g_state, targets, 0u, &out);
    TEST_ASSERT_TRUE(out.b_active);
    TEST_ASSERT_EQUAL_UINT16(1500u, out.d_mm);

    // Left vanishes: its hold starts now (t=50). Right is still live and
    // refreshed, so the nearer-of-the-two-blocked-lanes distance is still
    // left's held 1500.
    targets[0] = empty;
    threat_assessment_update(&g_state, targets, 50u, &out);
    TEST_ASSERT_TRUE(out.b_active);
    TEST_ASSERT_EQUAL_UINT16(1500u, out.d_mm);

    // Right vanishes too: its own, independently-timed hold starts now
    // (t=100), 50 ms after left's. Both lanes are now "blocked" purely
    // via their own hold timers, started at different times.
    targets[1] = empty;
    threat_assessment_update(&g_state, targets, 100u, &out);
    TEST_ASSERT_TRUE(out.b_active);
    TEST_ASSERT_EQUAL_UINT16(1500u, out.d_mm);

    // 199 ms into left's hold (started at 50): still held. Right's hold
    // (started at 100) is only 149 ms in: also still held.
    threat_assessment_update(&g_state, targets, 249u, &out);
    TEST_ASSERT_TRUE(out.b_active);
    TEST_ASSERT_EQUAL_UINT16(1500u, out.d_mm);

    // 200 ms into left's hold: left clears. Right's hold (150 ms so far)
    // has not. The alert must stay active on right's say alone, and d_mm
    // must switch to right's distance - the behaviour that would quietly
    // break if the two lanes' hold timers shared any state.
    threat_assessment_update(&g_state, targets, 250u, &out);
    TEST_ASSERT_TRUE(out.b_active);
    TEST_ASSERT_FALSE(out.b_post_clear);
    TEST_ASSERT_EQUAL_UINT16(1600u, out.d_mm);

    // 200 ms into right's hold (started at 100): right clears too, and
    // only now does the alert actually end.
    threat_assessment_update(&g_state, targets, 300u, &out);
    TEST_ASSERT_FALSE(out.b_active);
    TEST_ASSERT_TRUE(out.b_post_clear);
}

static void
test_process_frame_sends_audio_and_ble (void)
{
    rte_radar_target_t targets[RTE_RADAR_NUM_TARGETS];

    make_frame(targets, 800, -20);
    threat_assessment_process_frame(&g_state, targets, 0u);

    TEST_ASSERT_EQUAL_size_t(1u, mock_audio_count());
    TEST_ASSERT_EQUAL(RTE_AUDIO_OBSTACLE, mock_audio_get(0u).type);
    TEST_ASSERT_EQUAL_UINT16(800u, mock_audio_get(0u).d_mm);
    TEST_ASSERT_EQUAL_size_t(1u, mock_ble_count());
    TEST_ASSERT_TRUE(mock_ble_get(0u).b_active);
    // FR13 (OPEN-23, then per-lane OPEN-25): make_frame() puts its
    // target in CENTER, which has a live reading this frame with closing
    // speed 20 cm/s (make_frame's -20 speed_cms, approaching under
    // lcfg's sign convention), so CENTER's lane_ttc_ms is a real computed
    // value, not NONE; LEFT/RIGHT are not BLOCKED at all. See
    // docs/deviations.md OPEN-23/OPEN-25.
    TEST_ASSERT_EQUAL_UINT16(800u, mock_ble_get(0u).d_mm[RTE_SIDE_CENTER]);
    TEST_ASSERT_EQUAL_UINT16(4000u,
                             mock_ble_get(0u).ttc_ms[RTE_SIDE_CENTER]);
    TEST_ASSERT_EQUAL_UINT16(RTE_LANE_NONE_MM,
                             mock_ble_get(0u).d_mm[RTE_SIDE_LEFT]);
    TEST_ASSERT_EQUAL_UINT16(RTE_LANE_NONE_MM,
                             mock_ble_get(0u).d_mm[RTE_SIDE_RIGHT]);
}

static void
test_radar_fault_forces_clear_only_when_active (void)
{
    rte_radar_target_t targets[RTE_RADAR_NUM_TARGETS];

    threat_assessment_process_fault(&g_state);
    TEST_ASSERT_EQUAL_size_t(0u, mock_audio_count());

    make_frame(targets, 800, 0);
    threat_assessment_process_frame(&g_state, targets, 0u);
    threat_assessment_process_fault(&g_state);

    TEST_ASSERT_EQUAL_size_t(2u, mock_audio_count());
    TEST_ASSERT_EQUAL(RTE_AUDIO_OBSTACLE_CLEAR, mock_audio_get(1u).type);
    TEST_ASSERT_FALSE(mock_ble_get(1u).b_active);

    threat_assessment_process_fault(&g_state);
    TEST_ASSERT_EQUAL_size_t(2u, mock_audio_count());
}

static void
test_grip_starts_unknown_and_unknown_alerts (void)
{
    rte_radar_target_t targets[RTE_RADAR_NUM_TARGETS];
    threat_assessment_out_t out;

    TEST_ASSERT_EQUAL(RTE_GRIP_UNKNOWN, g_state.grip);

    make_frame(targets, 1500, 0);
    threat_assessment_update(&g_state, targets, 0u, &out);

    TEST_ASSERT_TRUE(out.b_post_obstacle);
    TEST_ASSERT_EQUAL_UINT16(1500u, out.d_mm);
}

static void
test_held_alerts (void)
{
    rte_radar_target_t targets[RTE_RADAR_NUM_TARGETS];
    threat_assessment_out_t out;

    threat_assessment_process_grip(&g_state, RTE_GRIP_HELD);
    make_frame(targets, 1500, 0);
    threat_assessment_update(&g_state, targets, 0u, &out);

    TEST_ASSERT_TRUE(out.b_post_obstacle);
    TEST_ASSERT_EQUAL_size_t(0u, mock_audio_count());
}

static void
test_released_suppresses_alert_and_posts_nothing (void)
{
    rte_radar_target_t targets[RTE_RADAR_NUM_TARGETS];

    threat_assessment_process_grip(&g_state, RTE_GRIP_RELEASED);
    make_frame(targets, 500, -30);

    for (uint32_t now_ms = 0u; now_ms < 500u; now_ms += 100u)
    {
        threat_assessment_process_frame(&g_state, targets, now_ms);
    }

    TEST_ASSERT_FALSE(g_state.b_active);
    TEST_ASSERT_EQUAL_size_t(0u, mock_audio_count());
    TEST_ASSERT_EQUAL_size_t(0u, mock_ble_count());
}

static void
test_release_while_active_clears_once (void)
{
    rte_radar_target_t targets[RTE_RADAR_NUM_TARGETS];

    make_frame(targets, 800, 0);
    threat_assessment_process_frame(&g_state, targets, 0u);
    TEST_ASSERT_EQUAL_size_t(1u, mock_audio_count());

    threat_assessment_process_grip(&g_state, RTE_GRIP_RELEASED);
    TEST_ASSERT_EQUAL_size_t(2u, mock_audio_count());
    TEST_ASSERT_EQUAL(RTE_AUDIO_OBSTACLE_CLEAR, mock_audio_get(1u).type);
    TEST_ASSERT_FALSE(mock_ble_get(1u).b_active);

    // The target is still there, but nothing more is sent, and a second
    // RELEASED does not repeat the CLEAR.
    threat_assessment_process_frame(&g_state, targets, 100u);
    threat_assessment_process_grip(&g_state, RTE_GRIP_RELEASED);
    TEST_ASSERT_EQUAL_size_t(2u, mock_audio_count());
}

static void
test_alert_resumes_when_grip_is_held_or_unknown_again (void)
{
    rte_radar_target_t targets[RTE_RADAR_NUM_TARGETS];
    threat_assessment_out_t out;

    make_frame(targets, 1500, 0);
    threat_assessment_process_grip(&g_state, RTE_GRIP_RELEASED);
    threat_assessment_update(&g_state, targets, 0u, &out);
    TEST_ASSERT_FALSE(out.b_active);

    // HELD: the next frame starts the alert at once (target is inside the
    // lane's START_MM).
    threat_assessment_process_grip(&g_state, RTE_GRIP_HELD);
    threat_assessment_update(&g_state, targets, 100u, &out);
    TEST_ASSERT_TRUE(out.b_post_obstacle);

    // UNKNOWN is treated as HELD: a dropout never silences the warning.
    threat_assessment_process_grip(&g_state, RTE_GRIP_UNKNOWN);
    threat_assessment_update(&g_state, targets, 200u, &out);
    TEST_ASSERT_TRUE(out.b_post_obstacle);

    // And UNKNOWN after RELEASED brings the alert back.
    threat_assessment_process_grip(&g_state, RTE_GRIP_RELEASED);
    threat_assessment_process_grip(&g_state, RTE_GRIP_UNKNOWN);
    threat_assessment_update(&g_state, targets, 300u, &out);
    TEST_ASSERT_TRUE(out.b_post_obstacle);
}

static void
test_grip_survives_a_radar_fault (void)
{
    rte_radar_target_t targets[RTE_RADAR_NUM_TARGETS];
    threat_assessment_out_t out;

    threat_assessment_process_grip(&g_state, RTE_GRIP_RELEASED);
    threat_assessment_process_fault(&g_state);
    TEST_ASSERT_EQUAL(RTE_GRIP_RELEASED, g_state.grip);

    make_frame(targets, 500, 0);
    threat_assessment_update(&g_state, targets, 0u, &out);
    TEST_ASSERT_FALSE(out.b_post_obstacle);
}

static void
test_direction_cue_middle_only_defaults_left (void)
{
    rte_radar_target_t targets[RTE_RADAR_NUM_TARGETS];
    threat_assessment_out_t out;

    targets[0] = make_target_side(2200, 0, RTE_SIDE_LEFT);
    targets[1] = make_target_side(1800, 0, RTE_SIDE_CENTER);
    targets[2] = make_target_side(2200, 0, RTE_SIDE_RIGHT);
    threat_assessment_update(&g_state, targets, 0u, &out);

    TEST_ASSERT_TRUE(out.b_active);
    TEST_ASSERT_EQUAL(RTE_SIDE_LEFT, cue_side(out.cue));
    TEST_ASSERT_EQUAL(0u, out.cue & RTE_CUE_STOP);
}

static void
test_direction_cue_left_only_stays_middle (void)
{
    rte_radar_target_t targets[RTE_RADAR_NUM_TARGETS];
    threat_assessment_out_t out;

    targets[0] = make_target_side(1800, 0, RTE_SIDE_LEFT);
    targets[1] = make_target_side(2200, 0, RTE_SIDE_CENTER);
    targets[2] = make_target_side(2200, 0, RTE_SIDE_RIGHT);
    threat_assessment_update(&g_state, targets, 0u, &out);

    TEST_ASSERT_TRUE(out.b_active);
    TEST_ASSERT_EQUAL(RTE_SIDE_CENTER, cue_side(out.cue));
    TEST_ASSERT_EQUAL(0u, out.cue & RTE_CUE_STOP);
}

static void
test_direction_cue_right_only_stays_middle (void)
{
    rte_radar_target_t targets[RTE_RADAR_NUM_TARGETS];
    threat_assessment_out_t out;

    targets[0] = make_target_side(2200, 0, RTE_SIDE_LEFT);
    targets[1] = make_target_side(2200, 0, RTE_SIDE_CENTER);
    targets[2] = make_target_side(1800, 0, RTE_SIDE_RIGHT);
    threat_assessment_update(&g_state, targets, 0u, &out);

    TEST_ASSERT_TRUE(out.b_active);
    TEST_ASSERT_EQUAL(RTE_SIDE_CENTER, cue_side(out.cue));
    TEST_ASSERT_EQUAL(0u, out.cue & RTE_CUE_STOP);
}

static void
test_direction_cue_left_and_middle_blocked_goes_right (void)
{
    rte_radar_target_t targets[RTE_RADAR_NUM_TARGETS];
    threat_assessment_out_t out;

    targets[0] = make_target_side(1800, 0, RTE_SIDE_LEFT);
    targets[1] = make_target_side(1800, 0, RTE_SIDE_CENTER);
    targets[2] = make_target_side(2200, 0, RTE_SIDE_RIGHT);
    threat_assessment_update(&g_state, targets, 0u, &out);

    TEST_ASSERT_TRUE(out.b_active);
    TEST_ASSERT_EQUAL(RTE_SIDE_RIGHT, cue_side(out.cue));
    TEST_ASSERT_EQUAL(0u, out.cue & RTE_CUE_STOP);
}

static void
test_direction_cue_middle_and_right_blocked_goes_left (void)
{
    rte_radar_target_t targets[RTE_RADAR_NUM_TARGETS];
    threat_assessment_out_t out;

    targets[0] = make_target_side(2200, 0, RTE_SIDE_LEFT);
    targets[1] = make_target_side(1800, 0, RTE_SIDE_CENTER);
    targets[2] = make_target_side(1800, 0, RTE_SIDE_RIGHT);
    threat_assessment_update(&g_state, targets, 0u, &out);

    TEST_ASSERT_TRUE(out.b_active);
    TEST_ASSERT_EQUAL(RTE_SIDE_LEFT, cue_side(out.cue));
    TEST_ASSERT_EQUAL(0u, out.cue & RTE_CUE_STOP);
}

static void
test_direction_cue_left_and_right_blocked_stays_middle (void)
{
    rte_radar_target_t targets[RTE_RADAR_NUM_TARGETS];
    threat_assessment_out_t out;

    targets[0] = make_target_side(1800, 0, RTE_SIDE_LEFT);
    targets[1] = make_target_side(2200, 0, RTE_SIDE_CENTER);
    targets[2] = make_target_side(1800, 0, RTE_SIDE_RIGHT);
    threat_assessment_update(&g_state, targets, 0u, &out);

    TEST_ASSERT_TRUE(out.b_active);
    TEST_ASSERT_EQUAL(RTE_SIDE_CENTER, cue_side(out.cue));
    TEST_ASSERT_EQUAL(0u, out.cue & RTE_CUE_STOP);
}

static void
test_direction_cue_all_three_blocked_is_stop (void)
{
    rte_radar_target_t targets[RTE_RADAR_NUM_TARGETS];
    threat_assessment_out_t out;

    targets[0] = make_target_side(1800, 0, RTE_SIDE_LEFT);
    targets[1] = make_target_side(1800, 0, RTE_SIDE_CENTER);
    targets[2] = make_target_side(1800, 0, RTE_SIDE_RIGHT);
    threat_assessment_update(&g_state, targets, 0u, &out);

    TEST_ASSERT_TRUE(out.b_active);
    TEST_ASSERT_NOT_EQUAL(0u, out.cue & RTE_CUE_STOP);
}

static void
test_direction_cue_all_clear_is_center_and_inactive (void)
{
    rte_radar_target_t targets[RTE_RADAR_NUM_TARGETS];
    threat_assessment_out_t out;

    targets[0] = make_target_side(2200, 0, RTE_SIDE_LEFT);
    targets[1] = make_target_side(2200, 0, RTE_SIDE_CENTER);
    targets[2] = make_target_side(2200, 0, RTE_SIDE_RIGHT);
    threat_assessment_update(&g_state, targets, 0u, &out);

    TEST_ASSERT_FALSE(out.b_active);
    TEST_ASSERT_EQUAL_UINT8(0u, out.cue);
}

int
main (void)
{
    UNITY_BEGIN();

    RUN_TEST(test_distance_is_read_from_the_target);
    RUN_TEST(test_closing_cms_sign);
    RUN_TEST(test_ttc);
    RUN_TEST(test_stationary_object_inside_range_raises_alert);
    RUN_TEST(test_alert_starts_at_1800_and_not_above);
    RUN_TEST(test_hysteresis_keeps_blocked_between_1800_and_1950);
    RUN_TEST(test_crossing_1950_clears_only_after_the_lane_hold);
    RUN_TEST(test_lane_hold_absorbs_target_vanishing);
    RUN_TEST(test_lane_hold_is_cancelled_if_the_target_reappears);
    RUN_TEST(test_clock_wrap_does_not_break_the_lane_hold_timer);
    RUN_TEST(test_departing_target_still_blocks_its_lane);
    RUN_TEST(test_nearest_blocked_lane_drives_d_mm);
    RUN_TEST(test_direction_cue_middle_only_defaults_left);
    RUN_TEST(test_direction_cue_left_only_stays_middle);
    RUN_TEST(test_direction_cue_right_only_stays_middle);
    RUN_TEST(test_direction_cue_left_and_middle_blocked_goes_right);
    RUN_TEST(test_direction_cue_middle_and_right_blocked_goes_left);
    RUN_TEST(test_direction_cue_left_and_right_blocked_stays_middle);
    RUN_TEST(test_direction_cue_all_three_blocked_is_stop);
    RUN_TEST(test_direction_cue_all_clear_is_center_and_inactive);
    RUN_TEST(test_ttc_uses_live_speed_of_its_own_lane);
    RUN_TEST(test_ttc_is_none_while_the_lane_is_only_held);
    RUN_TEST(test_ttc_is_none_for_a_departing_target);
    RUN_TEST(test_ttc_holds_a_brief_speed_dropout);
    RUN_TEST(test_ttc_drops_to_none_once_the_speed_hold_expires);
    RUN_TEST(test_ttc_hold_does_not_resume_across_a_full_occupancy_gap);
    RUN_TEST(test_two_independent_lane_hold_timers_overlap);
    RUN_TEST(test_process_frame_sends_audio_and_ble);
    RUN_TEST(test_radar_fault_forces_clear_only_when_active);
    RUN_TEST(test_grip_starts_unknown_and_unknown_alerts);
    RUN_TEST(test_held_alerts);
    RUN_TEST(test_released_suppresses_alert_and_posts_nothing);
    RUN_TEST(test_release_while_active_clears_once);
    RUN_TEST(test_alert_resumes_when_grip_is_held_or_unknown_again);
    RUN_TEST(test_grip_survives_a_radar_fault);

    return (UNITY_END());
}

/*** end of file ***/
