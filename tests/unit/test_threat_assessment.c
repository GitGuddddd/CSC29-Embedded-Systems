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
make_target (int16_t x_mm, int16_t y_mm, int16_t speed_cms)
{
    rte_radar_target_t target = { true, x_mm, y_mm, speed_cms, 360u };

    return (target);
}

// Fills the three slots: slot 0 = given target, the other two empty.
static void
make_frame (rte_radar_target_t * p_targets, int16_t y_mm, int16_t speed_cms)
{
    rte_radar_target_t empty = { false, 0, 0, 0, 0u };

    p_targets[0] = make_target(0, y_mm, speed_cms);
    p_targets[1] = empty;
    p_targets[2] = empty;
}

static void
make_empty_frame (rte_radar_target_t * p_targets)
{
    rte_radar_target_t empty = { false, 0, 0, 0, 0u };

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
test_distance_is_euclidean (void)
{
    rte_radar_target_t t345 = make_target(3000, 4000, 0);
    rte_radar_target_t front = make_target(0, 526, 0);
    rte_radar_target_t neg = make_target(-3000, -4000, 0);
    rte_radar_target_t max = make_target(32767, 32767, 0);

    TEST_ASSERT_EQUAL_UINT16(5000u, threat_assessment_distance_mm(&t345));
    TEST_ASSERT_EQUAL_UINT16(526u, threat_assessment_distance_mm(&front));
    TEST_ASSERT_EQUAL_UINT16(5000u, threat_assessment_distance_mm(&neg));
    TEST_ASSERT_EQUAL_UINT16(46339u, threat_assessment_distance_mm(&max));
}

static void
test_departing_limit (void)
{
    rte_radar_target_t approaching = make_target(0, 1000, -16);
    rte_radar_target_t still = make_target(0, 1000, 0);
    rte_radar_target_t slow_away = make_target(0, 1000, 8);
    rte_radar_target_t limit = make_target(0, 1000, 10);
    rte_radar_target_t fast_away = make_target(0, 1000, 16);

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
test_pitch_map (void)
{
    TEST_ASSERT_EQUAL_UINT16(1500u, threat_assessment_pitch_hz(2000u));
    TEST_ASSERT_EQUAL_UINT16(1500u, threat_assessment_pitch_hz(2100u));
    TEST_ASSERT_EQUAL_UINT16(3500u, threat_assessment_pitch_hz(300u));
    TEST_ASSERT_EQUAL_UINT16(3500u, threat_assessment_pitch_hz(0u));
    TEST_ASSERT_EQUAL_UINT16(2500u, threat_assessment_pitch_hz(1150u));
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

    targets[0] = make_target(0, 500, 40);
    targets[1] = make_target(0, 1800, -8);
    targets[2] = make_target(0, 1200, 0);
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
    for (int16_t y_mm = 500; y_mm <= 524; y_mm += 12)
    {
        make_frame(targets, y_mm, 0);
        threat_assessment_update(&g_state, targets, now_ms, &out);
        TEST_ASSERT_TRUE(out.b_active);
        TEST_ASSERT_EQUAL_UINT16((uint16_t) y_mm, out.d_mm);
        now_ms += 89u;
    }

    // 4th frame: the range grew 36 mm in 267 ms. The target is departing,
    // so the old distance is held for this one frame only.
    make_frame(targets, 536, 0);
    threat_assessment_update(&g_state, targets, now_ms, &out);
    TEST_ASSERT_TRUE(out.b_active);
    TEST_ASSERT_FALSE(out.b_post_clear);
    TEST_ASSERT_EQUAL_UINT16(524u, out.d_mm);
    now_ms += 89u;

    // 5th frame: still departing, so the alert ends now, not after the 500 ms hold.
    make_frame(targets, 548, 0);
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
    uint32_t now_ms = 0u;

    // Bench log: speed read 24 and 16 cm/s while the range stayed at 205 mm.
    for (size_t idx = 0u; idx < (sizeof(speed_cms) / sizeof(speed_cms[0]));
         idx++)
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
    uint32_t now_ms = 0u;

    for (size_t idx = 0u; idx < (sizeof(jitter_mm) / sizeof(jitter_mm[0]));
         idx++)
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
    rte_radar_target_t empty = { false, 0, 0, 0, 0u };
    threat_assessment_out_t out;
    uint32_t now_ms = 0u;

    // Slot 0 walks away at 13 cm/s; slot 1 is a still wall at 1500 mm.
    for (int16_t y_mm = 500; y_mm <= 536; y_mm += 12)
    {
        targets[0] = make_target(0, y_mm, 0);
        targets[1] = make_target(0, 1500, 0);
        targets[2] = empty;
        threat_assessment_update(&g_state, targets, now_ms, &out);
        now_ms += 89u;
    }

    TEST_ASSERT_TRUE(out.b_active);
    TEST_ASSERT_EQUAL_UINT16(1500u, out.d_mm);
}

int
main (void)
{
    UNITY_BEGIN();
    RUN_TEST(test_distance_is_euclidean);
    RUN_TEST(test_departing_limit);
    RUN_TEST(test_ttc);
    RUN_TEST(test_pitch_map);
    RUN_TEST(test_stationary_object_inside_range_raises_alert);
    RUN_TEST(test_departing_target_is_ignored_and_nearest_non_departing_wins);
    RUN_TEST(test_alert_starts_below_2000_and_not_at_2000);
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
    RUN_TEST(test_process_frame_sends_audio_and_ble);
    RUN_TEST(test_radar_fault_forces_clear_only_when_active);

    return (UNITY_END());
}

/*** end of file ***/
