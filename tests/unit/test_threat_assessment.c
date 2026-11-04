/** @file test_threat_assessment.c
 *
 * @brief Unit tests of the obstacle decision (FR8, FR9, FR10, FR13, NFR8).
 *
 * @par
 * COPYRIGHT NOTICE: (c) 2026 Group 29. All rights reserved.
 */

#include <stdbool.h>
#include <stdint.h>

#include "unity.h"

#include "mock_hal.h"
#include "rte_type.h"
#include "threat_assessment.h"

static threat_assessment_t g_state;

static rte_radar_target_t
make_target (int16_t d_mm, int16_t speed_cms)
{
    rte_radar_target_t target = { true, 0, 0, (uint16_t) d_mm, speed_cms };

    return (target);
}

// Fills the three slots: slot 0 = given target, the other two empty.
static void
make_frame (rte_radar_target_t * p_targets, int16_t y_mm, int16_t speed_cms)
{
    rte_radar_target_t empty = { false, 0, 0, 0u, 0 };

    p_targets[0] = make_target(y_mm, speed_cms);
    p_targets[1] = empty;
    p_targets[2] = empty;
}

static void
make_empty_frame (rte_radar_target_t * p_targets)
{
    rte_radar_target_t empty = { false, 0, 0, 0u, 0 };

    p_targets[0] = empty;
    p_targets[1] = empty;
    p_targets[2] = empty;
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
test_departing_limit (void)
{
    rte_radar_target_t approaching = make_target(1000, -16);
    rte_radar_target_t still = make_target(1000, 0);
    rte_radar_target_t slow_away = make_target(1000, 8);
    rte_radar_target_t limit = make_target(1000, 10);
    rte_radar_target_t fast_away = make_target(1000, 16);

    TEST_ASSERT_FALSE(threat_assessment_is_departing(&approaching));
    TEST_ASSERT_FALSE(threat_assessment_is_departing(&still));
    TEST_ASSERT_FALSE(threat_assessment_is_departing(&slow_away));
    TEST_ASSERT_FALSE(threat_assessment_is_departing(&limit));
    TEST_ASSERT_TRUE(threat_assessment_is_departing(&fast_away));
    TEST_ASSERT_EQUAL_INT32(16, threat_assessment_closing_cms(&approaching));
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
    TEST_ASSERT_EQUAL_UINT16(RTE_TTC_NONE_MS, out.ttc_ms);
}

static void
test_departing_target_is_ignored_and_nearest_non_departing_wins (void)
{
    rte_radar_target_t targets[RTE_RADAR_NUM_TARGETS];
    threat_assessment_out_t out;

    targets[0] = make_target(500, 40);
    targets[1] = make_target(1800, -8);
    targets[2] = make_target(1200, 0);
    threat_assessment_update(&g_state, targets, 0u, &out);

    TEST_ASSERT_TRUE(out.b_post_obstacle);
    TEST_ASSERT_EQUAL_UINT16(1200u, out.d_mm);
}

static void
test_alert_starts_below_2000_and_not_at_2000 (void)
{
    rte_radar_target_t targets[RTE_RADAR_NUM_TARGETS];
    threat_assessment_out_t out;

    make_frame(targets, 2000, 0);
    threat_assessment_update(&g_state, targets, 0u, &out);
    TEST_ASSERT_FALSE(out.b_active);

    make_frame(targets, 1999, 0);
    threat_assessment_update(&g_state, targets, 140u, &out);
    TEST_ASSERT_TRUE(out.b_active);
}

static void
test_hysteresis_stops_only_above_2100 (void)
{
    rte_radar_target_t targets[RTE_RADAR_NUM_TARGETS];
    threat_assessment_out_t out;

    make_frame(targets, 1900, 0);
    threat_assessment_update(&g_state, targets, 0u, &out);
    TEST_ASSERT_TRUE(out.b_active);

    make_frame(targets, 2100, 0);
    threat_assessment_update(&g_state, targets, 140u, &out);
    TEST_ASSERT_TRUE(out.b_active);
    TEST_ASSERT_FALSE(out.b_post_clear);

    make_frame(targets, 2101, 0);
    threat_assessment_update(&g_state, targets, 280u, &out);
    TEST_ASSERT_FALSE(out.b_active);
    TEST_ASSERT_TRUE(out.b_post_clear);
    TEST_ASSERT_FALSE(out.b_post_obstacle);

    // The clear is sent exactly once.
    threat_assessment_update(&g_state, targets, 420u, &out);
    TEST_ASSERT_FALSE(out.b_post_clear);
}

static void
test_no_target_holds_half_second_then_clears (void)
{
    rte_radar_target_t targets[RTE_RADAR_NUM_TARGETS];
    threat_assessment_out_t out;

    make_frame(targets, 1500, 0);
    threat_assessment_update(&g_state, targets, 1000u, &out);
    make_empty_frame(targets);

    threat_assessment_update(&g_state, targets, 1140u, &out);
    TEST_ASSERT_TRUE(out.b_active);
    TEST_ASSERT_TRUE(out.b_post_obstacle);
    TEST_ASSERT_EQUAL_UINT16(1500u, out.d_mm);
    TEST_ASSERT_EQUAL_UINT16(RTE_TTC_NONE_MS, out.ttc_ms);

    // 499 ms after the last target the alert still holds (FR10: 500 ms).
    threat_assessment_update(&g_state, targets, 1499u, &out);
    TEST_ASSERT_TRUE(out.b_active);

    threat_assessment_update(&g_state, targets, 1500u, &out);
    TEST_ASSERT_FALSE(out.b_active);
    TEST_ASSERT_TRUE(out.b_post_clear);
}

static void
test_ttc_reported_for_approaching_target (void)
{
    rte_radar_target_t targets[RTE_RADAR_NUM_TARGETS];
    threat_assessment_out_t out;

    make_frame(targets, 1000, -50);
    threat_assessment_update(&g_state, targets, 0u, &out);

    TEST_ASSERT_EQUAL_UINT16(2000u, out.ttc_ms);
}

static void
test_clock_wrap_does_not_break_hold_timer (void)
{
    rte_radar_target_t targets[RTE_RADAR_NUM_TARGETS];
    threat_assessment_out_t out;

    make_frame(targets, 1500, 0);
    threat_assessment_update(&g_state, targets, 0xFFFFFF00u, &out);
    make_empty_frame(targets);

    // 0x140 ms (320 ms) later, across the wrap, the hold is not over...
    threat_assessment_update(&g_state, targets, 0x00000040u, &out);
    TEST_ASSERT_TRUE(out.b_active);

    // ...and 500 ms after the last target it is.
    threat_assessment_update(&g_state, targets, 0x000000F4u, &out);
    TEST_ASSERT_FALSE(out.b_active);
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
    TEST_ASSERT_EQUAL_UINT16(4000u, mock_ble_get(0u).ttc_ms);
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
test_slow_departure_with_zero_speed_is_caught_by_range_trend (void)
{
    rte_radar_target_t targets[RTE_RADAR_NUM_TARGETS];
    threat_assessment_out_t out;
    uint32_t now_ms = 0u;

    // 12 mm per 89 ms frame = 13 cm/s while the sensor reports speed 0.
    // The trend needs TREND_FRAMES (4) earlier frames, so these four frames
    // all report their own distance.
    for (int16_t y_mm = 500; y_mm <= 536; y_mm += 12)
    {
        make_frame(targets, y_mm, 0);
        threat_assessment_update(&g_state, targets, now_ms, &out);
        TEST_ASSERT_TRUE(out.b_active);
        TEST_ASSERT_EQUAL_UINT16((uint16_t) y_mm, out.d_mm);
        now_ms += 89u;
    }

    // 5th frame: the range grew 48 mm in 356 ms. The target is departing,
    // so the old distance is held for this one frame only.
    make_frame(targets, 548, 0);
    threat_assessment_update(&g_state, targets, now_ms, &out);
    TEST_ASSERT_TRUE(out.b_active);
    TEST_ASSERT_FALSE(out.b_post_clear);
    TEST_ASSERT_EQUAL_UINT16(536u, out.d_mm);
    now_ms += 89u;

    // 6th frame: still departing, so the alert ends now, not after the
    // 500 ms hold.
    make_frame(targets, 560, 0);
    threat_assessment_update(&g_state, targets, now_ms, &out);
    TEST_ASSERT_FALSE(out.b_active);
    TEST_ASSERT_TRUE(out.b_post_clear);
    TEST_ASSERT_FALSE(out.b_post_obstacle);
}

static void
test_sensor_speed_noise_on_still_target_does_not_clear (void)
{
    rte_radar_target_t targets[RTE_RADAR_NUM_TARGETS];
    threat_assessment_out_t out;
    const int16_t speed_cms[] = { 0, 24, 16, 24, 24, 16, 8, 0 };
    const size_t count = sizeof(speed_cms) / sizeof(speed_cms[0]);
    uint32_t now_ms = 0u;

    // Bench log: speed read 24 and 16 cm/s while the range stayed at 205 mm.
    for (size_t idx = 0u; idx < count; idx++)
    {
        make_frame(targets, 205, speed_cms[idx]);
        threat_assessment_update(&g_state, targets, now_ms, &out);
        TEST_ASSERT_TRUE(out.b_active);
        TEST_ASSERT_FALSE(out.b_post_clear);
        TEST_ASSERT_EQUAL_UINT16(205u, out.d_mm);
        now_ms += 89u;
    }
}

static void
test_one_noisy_departing_frame_does_not_end_the_alert (void)
{
    rte_radar_target_t targets[RTE_RADAR_NUM_TARGETS];
    threat_assessment_out_t out;

    make_frame(targets, 400, 0);
    threat_assessment_update(&g_state, targets, 0u, &out);
    make_frame(targets, 400, 24);
    threat_assessment_update(&g_state, targets, 89u, &out);
    TEST_ASSERT_TRUE(out.b_active);

    make_frame(targets, 400, 0);
    threat_assessment_update(&g_state, targets, 178u, &out);
    TEST_ASSERT_TRUE(out.b_active);
    TEST_ASSERT_EQUAL_UINT16(400u, out.d_mm);

    // The counter restarted: one more departing frame is again tolerated.
    make_frame(targets, 400, 24);
    threat_assessment_update(&g_state, targets, 267u, &out);
    TEST_ASSERT_TRUE(out.b_active);
}

static void
test_jitter_and_approach_are_not_a_departing_trend (void)
{
    rte_radar_target_t targets[RTE_RADAR_NUM_TARGETS];
    threat_assessment_out_t out;
    const int16_t jitter_mm[] = { 500, 505, 498, 504, 497, 503, 500 };
    const size_t count = sizeof(jitter_mm) / sizeof(jitter_mm[0]);
    uint32_t now_ms = 0u;

    for (size_t idx = 0u; idx < count; idx++)
    {
        make_frame(targets, jitter_mm[idx], 0);
        threat_assessment_update(&g_state, targets, now_ms, &out);
        TEST_ASSERT_TRUE(out.b_active);
        TEST_ASSERT_EQUAL_UINT16((uint16_t) jitter_mm[idx], out.d_mm);
        now_ms += 89u;
    }

    threat_assessment_init(&g_state);

    for (int16_t y_mm = 900; y_mm >= 600; y_mm -= 30)
    {
        make_frame(targets, y_mm, 0);
        threat_assessment_update(&g_state, targets, now_ms, &out);
        TEST_ASSERT_EQUAL_UINT16((uint16_t) y_mm, out.d_mm);
        now_ms += 89u;
    }
}

static void
test_trend_ignores_history_older_than_400_ms (void)
{
    rte_radar_target_t targets[RTE_RADAR_NUM_TARGETS];
    threat_assessment_out_t out;

    make_frame(targets, 500, 0);
    threat_assessment_update(&g_state, targets, 0u, &out);
    threat_assessment_update(&g_state, targets, 89u, &out);
    threat_assessment_update(&g_state, targets, 178u, &out);

    // Frames were lost: 3 frames ago is now 1 s away, no trend is judged.
    make_frame(targets, 700, 0);
    threat_assessment_update(&g_state, targets, 1178u, &out);
    TEST_ASSERT_TRUE(out.b_active);
    TEST_ASSERT_EQUAL_UINT16(700u, out.d_mm);
}

static void
test_other_slot_still_alerts_when_nearest_is_departing_by_trend (void)
{
    rte_radar_target_t targets[RTE_RADAR_NUM_TARGETS];
    rte_radar_target_t empty = { false, 0, 0, 0u, 0 };
    threat_assessment_out_t out;
    uint32_t now_ms = 0u;

    // Slot 0 walks away at 13 cm/s; slot 1 is a still wall at 1500 mm.
    // Five frames: the fifth is the first one the 4-frame trend can judge.
    for (int16_t y_mm = 500; y_mm <= 548; y_mm += 12)
    {
        targets[0] = make_target(y_mm, 0);
        targets[1] = make_target(1500, 0);
        targets[2] = empty;
        threat_assessment_update(&g_state, targets, now_ms, &out);
        now_ms += 89u;
    }

    TEST_ASSERT_TRUE(out.b_active);
    TEST_ASSERT_EQUAL_UINT16(1500u, out.d_mm);
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

    // HELD: the next frame starts the alert at once (target is inside 2 m).
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
test_release_keeps_range_history_so_trend_still_works (void)
{
    rte_radar_target_t targets[RTE_RADAR_NUM_TARGETS];
    threat_assessment_out_t out;
    uint32_t now_ms = 0u;

    // Frames arrive while released, then the cane is gripped again while the
    // same target slowly walks away: the trend is already known.
    threat_assessment_process_grip(&g_state, RTE_GRIP_RELEASED);

    for (int16_t y_mm = 500; y_mm <= 536; y_mm += 12)
    {
        make_frame(targets, y_mm, 0);
        threat_assessment_update(&g_state, targets, now_ms, &out);
        now_ms += 89u;
    }

    threat_assessment_process_grip(&g_state, RTE_GRIP_HELD);
    make_frame(targets, 548, 0);
    threat_assessment_update(&g_state, targets, now_ms, &out);

    TEST_ASSERT_FALSE(out.b_post_obstacle);
}

// Jump gate: a one-frame jump away is held once while the alert is active.
static void
test_jump_gate_holds_one_far_frame_then_accepts (void)
{
    rte_radar_target_t targets[RTE_RADAR_NUM_TARGETS];
    threat_assessment_out_t out;

    make_frame(targets, 1500, 0);
    threat_assessment_update(&g_state, targets, 0u, &out);
    TEST_ASSERT_TRUE(out.b_active);

    // 3000 mm is 1500 mm farther: ignored once, the alert and distance stay.
    make_frame(targets, 3000, 0);
    threat_assessment_update(&g_state, targets, 90u, &out);
    TEST_ASSERT_TRUE(out.b_active);
    TEST_ASSERT_TRUE(out.b_post_obstacle);
    TEST_ASSERT_FALSE(out.b_post_clear);
    TEST_ASSERT_EQUAL_UINT16(1500u, out.d_mm);

    // The next frame agrees: accepted, the alert ends once.
    threat_assessment_update(&g_state, targets, 180u, &out);
    TEST_ASSERT_FALSE(out.b_active);
    TEST_ASSERT_TRUE(out.b_post_clear);
}

static void
test_jump_gate_spike_that_goes_back_is_forgotten (void)
{
    rte_radar_target_t targets[RTE_RADAR_NUM_TARGETS];
    threat_assessment_out_t out;

    make_frame(targets, 1500, 0);
    threat_assessment_update(&g_state, targets, 0u, &out);

    make_frame(targets, 3000, 0);
    threat_assessment_update(&g_state, targets, 90u, &out);
    TEST_ASSERT_TRUE(out.b_active);

    make_frame(targets, 1480, 0);
    threat_assessment_update(&g_state, targets, 180u, &out);
    TEST_ASSERT_TRUE(out.b_active);
    TEST_ASSERT_EQUAL_UINT16(1480u, out.d_mm);

    // A new spike is held again: the earlier one left nothing pending.
    make_frame(targets, 3000, 0);
    threat_assessment_update(&g_state, targets, 270u, &out);
    TEST_ASSERT_TRUE(out.b_active);
    TEST_ASSERT_EQUAL_UINT16(1480u, out.d_mm);
}

static void
test_jump_gate_never_holds_a_jump_toward_the_user (void)
{
    rte_radar_target_t targets[RTE_RADAR_NUM_TARGETS];
    threat_assessment_out_t out;

    make_frame(targets, 1900, 0);
    threat_assessment_update(&g_state, targets, 0u, &out);
    TEST_ASSERT_TRUE(out.b_active);

    // A nearer target appears: used on the same frame.
    make_frame(targets, 500, 0);
    threat_assessment_update(&g_state, targets, 90u, &out);
    TEST_ASSERT_TRUE(out.b_active);
    TEST_ASSERT_EQUAL_UINT16(500u, out.d_mm);

    // From no alert, one frame inside 2000 mm starts it at once.
    threat_assessment_init(&g_state);
    make_frame(targets, 3000, 0);
    threat_assessment_update(&g_state, targets, 0u, &out);
    TEST_ASSERT_FALSE(out.b_active);
    make_frame(targets, 1500, 0);
    threat_assessment_update(&g_state, targets, 90u, &out);
    TEST_ASSERT_TRUE(out.b_active);
    TEST_ASSERT_TRUE(out.b_post_obstacle);
}

static void
test_jump_gate_ignores_a_change_at_the_limit_and_a_long_gap (void)
{
    rte_radar_target_t targets[RTE_RADAR_NUM_TARGETS];
    threat_assessment_out_t out;

    // Exactly 1000 mm farther is not a jump: 1500 to 2500 clears at once.
    make_frame(targets, 1500, 0);
    threat_assessment_update(&g_state, targets, 0u, &out);
    make_frame(targets, 2500, 0);
    threat_assessment_update(&g_state, targets, 90u, &out);
    TEST_ASSERT_FALSE(out.b_active);
    TEST_ASSERT_TRUE(out.b_post_clear);

    // A jump after frames were lost (gap above 400 ms) is accepted at once.
    threat_assessment_init(&g_state);
    make_frame(targets, 1500, 0);
    threat_assessment_update(&g_state, targets, 0u, &out);
    make_frame(targets, 3000, 0);
    threat_assessment_update(&g_state, targets, 600u, &out);
    TEST_ASSERT_FALSE(out.b_active);
}

static void
test_jump_gate_hold_does_not_stop_the_500ms_hold_timer (void)
{
    rte_radar_target_t targets[RTE_RADAR_NUM_TARGETS];
    threat_assessment_out_t out;

    make_frame(targets, 1500, 0);
    threat_assessment_update(&g_state, targets, 0u, &out);

    make_frame(targets, 3000, 0);
    threat_assessment_update(&g_state, targets, 90u, &out);
    TEST_ASSERT_TRUE(out.b_active);

    // The ignored frame is not "seen": nothing refreshed the alert, and the
    // second far frame ends it.
    threat_assessment_update(&g_state, targets, 180u, &out);
    TEST_ASSERT_FALSE(out.b_active);
}

// Far-target hold (report D4): a far target must not end the alert on the
// first frame in which the near target looks like it is leaving.

static void
make_two_target_frame (rte_radar_target_t * p_targets, int16_t near_mm,
                       int16_t near_speed_cms, int16_t far_mm)
{
    rte_radar_target_t empty = { false, 0, 0, 0u, 0 };

    p_targets[0] = make_target(near_mm, near_speed_cms);
    p_targets[1] = make_target(far_mm, 0);
    p_targets[2] = empty;
}

static void
test_far_target_does_not_clear_on_one_noisy_speed_frame (void)
{
    rte_radar_target_t targets[RTE_RADAR_NUM_TARGETS];
    threat_assessment_out_t out;
    uint32_t now_ms = 0u;

    // A hand at 400 mm and a still target at 3500 mm: the alert is on.
    make_two_target_frame(targets, 400, 0, 3500);
    threat_assessment_update(&g_state, targets, now_ms, &out);
    TEST_ASSERT_TRUE(out.b_active);
    now_ms += 89u;

    // One frame with +32 cm/s on the hand: held, no CLEAR.
    make_two_target_frame(targets, 400, 32, 3500);
    threat_assessment_update(&g_state, targets, now_ms, &out);
    TEST_ASSERT_TRUE(out.b_active);
    TEST_ASSERT_FALSE(out.b_post_clear);
    TEST_ASSERT_TRUE(out.b_post_obstacle);
    TEST_ASSERT_EQUAL_UINT16(400u, out.d_mm);
    now_ms += 89u;

    // Back to normal: the alert goes on with the hand's distance.
    make_two_target_frame(targets, 396, 0, 3500);
    threat_assessment_update(&g_state, targets, now_ms, &out);
    TEST_ASSERT_TRUE(out.b_active);
    TEST_ASSERT_FALSE(out.b_post_clear);
    TEST_ASSERT_EQUAL_UINT16(396u, out.d_mm);
}

static void
test_far_target_keeps_alert_when_speed_is_high_but_range_does_not_grow (void)
{
    rte_radar_target_t targets[RTE_RADAR_NUM_TARGETS];
    threat_assessment_out_t out;
    uint32_t now_ms = 0u;

    make_two_target_frame(targets, 400, 0, 3500);
    threat_assessment_update(&g_state, targets, now_ms, &out);
    TEST_ASSERT_TRUE(out.b_active);

    // The sensor keeps saying +40 cm/s but the range does not change (1 Nov
    // sway and hand captures). Once there is enough range history the speed
    // is not believed, so the alert goes on for as long as it lasts.
    for (int idx = 0; idx < 14; idx++)
    {
        now_ms += 89u;
        make_two_target_frame(targets, 400, 40, 3500);
        threat_assessment_update(&g_state, targets, now_ms, &out);
        TEST_ASSERT_TRUE(out.b_active);
        TEST_ASSERT_FALSE(out.b_post_clear);
        TEST_ASSERT_TRUE(out.b_post_obstacle);
    }
}

static void
test_speed_with_range_growth_still_ends_the_alert_after_the_hold (void)
{
    rte_radar_target_t targets[RTE_RADAR_NUM_TARGETS];
    threat_assessment_out_t out;
    uint32_t now_ms = 0u;
    int16_t y_mm = 500;

    // 4 frames with speed 0 build the range history; the hand moves away at
    // 6 mm per 89 ms frame (6.7 cm/s): above the 5 cm/s confirmation, below
    // the 10 cm/s trend limit.
    for (int idx = 0; idx < 4; idx++)
    {
        make_two_target_frame(targets, y_mm, 0, 3500);
        threat_assessment_update(&g_state, targets, now_ms, &out);
        TEST_ASSERT_TRUE(out.b_active);
        y_mm = (int16_t) (y_mm + 6);
        now_ms += 89u;
    }

    // Sensor speed +40 and a growing range: the speed is believed. The alert
    // is held while the last sighting is under 500 ms old ...
    for (int idx = 0; idx < 5; idx++)
    {
        make_two_target_frame(targets, y_mm, 40, 3500);
        threat_assessment_update(&g_state, targets, now_ms, &out);
        TEST_ASSERT_TRUE(out.b_active);
        TEST_ASSERT_FALSE(out.b_post_clear);
        y_mm = (int16_t) (y_mm + 6);
        now_ms += 89u;
    }

    // ... and ends when the hold has run out.
    make_two_target_frame(targets, y_mm, 40, 3500);
    threat_assessment_update(&g_state, targets, now_ms, &out);
    TEST_ASSERT_FALSE(out.b_active);
    TEST_ASSERT_TRUE(out.b_post_clear);
}

static void
test_speed_without_range_growth_and_no_far_target_keeps_the_alert (void)
{
    rte_radar_target_t targets[RTE_RADAR_NUM_TARGETS];
    threat_assessment_out_t out;
    uint32_t now_ms = 0u;

    // A person swaying at 1400 mm: the sensor reads +30 cm/s on most frames.
    for (int idx = 0; idx < 20; idx++)
    {
        make_frame(targets, (int16_t) (1400 + ((idx % 3) * 4)),
                   (int16_t) ((idx < 1) ? 0 : 30));
        threat_assessment_update(&g_state, targets, now_ms, &out);
        TEST_ASSERT_TRUE(out.b_active);
        TEST_ASSERT_FALSE(out.b_post_clear);
        now_ms += 89u;
    }
}

static void
test_far_target_clears_a_real_exit_after_two_trend_frames (void)
{
    rte_radar_target_t targets[RTE_RADAR_NUM_TARGETS];
    threat_assessment_out_t out;
    uint32_t now_ms = 0u;

    // The hand moves away at 13 cm/s (sensor speed 0) with a far target.
    for (int16_t y_mm = 500; y_mm <= 536; y_mm += 12)
    {
        make_two_target_frame(targets, y_mm, 0, 3500);
        threat_assessment_update(&g_state, targets, now_ms, &out);
        TEST_ASSERT_TRUE(out.b_active);
        TEST_ASSERT_EQUAL_UINT16((uint16_t) y_mm, out.d_mm);
        now_ms += 89u;
    }

    // 1st trend frame: held, the old distance is repeated.
    make_two_target_frame(targets, 548, 0, 3500);
    threat_assessment_update(&g_state, targets, now_ms, &out);
    TEST_ASSERT_TRUE(out.b_active);
    TEST_ASSERT_FALSE(out.b_post_clear);
    TEST_ASSERT_EQUAL_UINT16(536u, out.d_mm);
    now_ms += 89u;

    // 2nd trend frame: the alert ends now, not after the 500 ms hold.
    make_two_target_frame(targets, 560, 0, 3500);
    threat_assessment_update(&g_state, targets, now_ms, &out);
    TEST_ASSERT_FALSE(out.b_active);
    TEST_ASSERT_TRUE(out.b_post_clear);
}

static void
test_far_target_clears_at_once_when_the_near_one_is_beyond_stop (void)
{
    rte_radar_target_t targets[RTE_RADAR_NUM_TARGETS];
    threat_assessment_out_t out;
    uint32_t now_ms = 0u;

    make_two_target_frame(targets, 1900, 0, 3500);
    threat_assessment_update(&g_state, targets, now_ms, &out);
    TEST_ASSERT_TRUE(out.b_active);
    now_ms += 89u;

    // The near target is already beyond 2100 mm and leaving: nothing to wait
    // for, the alert ends on this frame.
    make_two_target_frame(targets, 2200, 40, 3500);
    threat_assessment_update(&g_state, targets, now_ms, &out);
    TEST_ASSERT_FALSE(out.b_active);
    TEST_ASSERT_TRUE(out.b_post_clear);
}

int
main (void)
{
    UNITY_BEGIN();
    RUN_TEST(test_distance_is_read_from_the_target);
    RUN_TEST(test_departing_limit);
    RUN_TEST(test_ttc);
    RUN_TEST(test_stationary_object_inside_range_raises_alert);
    RUN_TEST(test_departing_target_is_ignored_and_nearest_non_departing_wins);
    RUN_TEST(test_alert_starts_below_2000_and_not_at_2000);
    RUN_TEST(test_jump_gate_holds_one_far_frame_then_accepts);
    RUN_TEST(test_jump_gate_spike_that_goes_back_is_forgotten);
    RUN_TEST(test_jump_gate_never_holds_a_jump_toward_the_user);
    RUN_TEST(test_jump_gate_ignores_a_change_at_the_limit_and_a_long_gap);
    RUN_TEST(test_jump_gate_hold_does_not_stop_the_500ms_hold_timer);
    RUN_TEST(test_hysteresis_stops_only_above_2100);
    RUN_TEST(test_no_target_holds_half_second_then_clears);
    RUN_TEST(test_ttc_reported_for_approaching_target);
    RUN_TEST(test_clock_wrap_does_not_break_hold_timer);
    RUN_TEST(test_slow_departure_with_zero_speed_is_caught_by_range_trend);
    RUN_TEST(test_sensor_speed_noise_on_still_target_does_not_clear);
    RUN_TEST(test_one_noisy_departing_frame_does_not_end_the_alert);
    RUN_TEST(test_jitter_and_approach_are_not_a_departing_trend);
    RUN_TEST(test_trend_ignores_history_older_than_400_ms);
    RUN_TEST(test_other_slot_still_alerts_when_nearest_is_departing_by_trend);
    RUN_TEST(test_far_target_does_not_clear_on_one_noisy_speed_frame);
    RUN_TEST(test_far_target_keeps_alert_when_speed_is_high_but_range_does_not_grow);
    RUN_TEST(test_speed_with_range_growth_still_ends_the_alert_after_the_hold);
    RUN_TEST(test_speed_without_range_growth_and_no_far_target_keeps_the_alert);
    RUN_TEST(test_far_target_clears_a_real_exit_after_two_trend_frames);
    RUN_TEST(test_far_target_clears_at_once_when_the_near_one_is_beyond_stop);
    RUN_TEST(test_process_frame_sends_audio_and_ble);
    RUN_TEST(test_radar_fault_forces_clear_only_when_active);
    RUN_TEST(test_grip_starts_unknown_and_unknown_alerts);
    RUN_TEST(test_held_alerts);
    RUN_TEST(test_released_suppresses_alert_and_posts_nothing);
    RUN_TEST(test_release_while_active_clears_once);
    RUN_TEST(test_alert_resumes_when_grip_is_held_or_unknown_again);
    RUN_TEST(test_grip_survives_a_radar_fault);
    RUN_TEST(test_release_keeps_range_history_so_trend_still_works);

    return (UNITY_END());
}

/*** end of file ***/
