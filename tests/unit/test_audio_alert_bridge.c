/** @file test_audio_alert_bridge.c
 *
 * @brief Bring-up bridge: radar messages -> Brian's alert.c -> buzzer tone.
 *
 * @par
 * COPYRIGHT NOTICE: (c) 2026 Group 29. All rights reserved.
 *
 * Uses Brian's unchanged alert logic and a fake buzzer. Expected tones come
 * from alert.c: pitch 1000 + (2000 - d) * 2 Hz, radar fault = two short
 * 1 kHz blips per second, a fault outranks the obstacle tone.
 */

#include <stdint.h>

#include "unity.h"

#include "alert.h"
#include "audio_alert_bridge.h"
#include "mock_buzzer.h"
#include "rte_type.h"

#define TEST_PITCH_AT_1362_MM_HZ    (2276u)
#define TEST_PITCH_AT_500_MM_HZ     (4000u)
#define TEST_FAULT_TONE_HZ          (1000u)

static void
send (rte_audio_msg_type_t type, uint16_t d_mm, uint8_t cue)
{
    rte_audio_msg_t msg = { .type = type, .d_mm = d_mm, .cue = cue };

    audio_alert_bridge_apply(&msg);
}

void
setUp (void)
{
    send(RTE_AUDIO_OBSTACLE_CLEAR, 0u, 0u);
    send(RTE_AUDIO_RADAR_OK, 0u, 0u);
    alert_update(0u);
    mock_buzzer_reset();
}

void
tearDown (void)
{
}

static void
test_obstacle_message_gives_the_pitch_of_alert_c (void)
{
    send(RTE_AUDIO_OBSTACLE, 1362u, 0u);
    alert_update(0u);
    TEST_ASSERT_EQUAL_UINT32(TEST_PITCH_AT_1362_MM_HZ,
                             mock_buzzer_last_tone_hz());
}

static void
test_closer_obstacle_is_higher (void)
{
    send(RTE_AUDIO_OBSTACLE, 500u, 0u);
    alert_update(0u);
    TEST_ASSERT_EQUAL_UINT32(TEST_PITCH_AT_500_MM_HZ,
                             mock_buzzer_last_tone_hz());
}

static void
test_clear_silences_the_tone (void)
{
    send(RTE_AUDIO_OBSTACLE, 1362u, 0u);
    alert_update(0u);
    send(RTE_AUDIO_OBSTACLE_CLEAR, 0u, 0u);
    alert_update(0u);
    TEST_ASSERT_EQUAL_UINT32(0u, mock_buzzer_last_tone_hz());
}

static void
test_distance_beyond_2000_mm_is_silent (void)
{
    send(RTE_AUDIO_OBSTACLE, 2002u, 0u);
    alert_update(0u);
    TEST_ASSERT_EQUAL_UINT32(0u, mock_buzzer_last_tone_hz());
}

static void
test_radar_fault_outranks_the_obstacle_tone (void)
{
    send(RTE_AUDIO_OBSTACLE, 1362u, 0u);
    send(RTE_AUDIO_RADAR_FAULT, 0u, 0u);
    alert_update(0u);
    TEST_ASSERT_EQUAL_UINT32(TEST_FAULT_TONE_HZ, mock_buzzer_last_tone_hz());
    alert_update(150u);
    TEST_ASSERT_EQUAL_UINT32(0u, mock_buzzer_last_tone_hz());
}

static void
test_radar_ok_brings_the_obstacle_tone_back (void)
{
    send(RTE_AUDIO_OBSTACLE, 1362u, 0u);
    send(RTE_AUDIO_RADAR_FAULT, 0u, 0u);
    alert_update(0u);
    send(RTE_AUDIO_RADAR_OK, 0u, 0u);
    alert_update(0u);
    TEST_ASSERT_EQUAL_UINT32(TEST_PITCH_AT_1362_MM_HZ,
                             mock_buzzer_last_tone_hz());
}

static void
test_null_message_is_ignored (void)
{
    audio_alert_bridge_apply(NULL);
    alert_update(0u);
    TEST_ASSERT_EQUAL_UINT32(0u, mock_buzzer_last_tone_hz());
}

// FR9 cue byte tests (prof feedback, 7 Nov): bring-up placeholder sounds in
// alert.c, see alert.h ALERT_CUE_*. Redesigned 8 Nov (Putra's call, design
// B): distance now speaks through loudness (duty_pct), side is always on
// (not just near STOP) via a fixed-timing beep count, and STOP only forces
// max loudness on top of whichever side pattern is already playing.
// CENTER (cue 0) is covered by the tests above, which never send a cue and
// so must behave exactly as before, except loudness now varies with
// distance instead of pulse rate.
//
// Beep timing (ALERT_BEEP_* in alert.c): pulse 150 ms, gap 150 ms (slot
// 300 ms) are fixed, so beep count (2 vs 3, i.e. direction) is unaffected
// by any of this. The pause AFTER a full burst is no longer fixed (11 Nov):
// it scales linearly with distance like duty_pct does, from 450 ms at the
// farthest distance (OBST_MAX_MM) down to 150 ms at the closest
// (OBST_MIN_MM), and STOP forces it straight to 150 ms regardless of
// distance - same pattern as STOP forcing max loudness.
//
// duty_pct at d=1362 mm: 10 + ((2000-1362)*40)/1800 = 10 + 14 = 24.
// pause_ms at d=1362 mm: 450 - ((2000-1362)*300)/1800 = 450 - 106 = 344.
// -> 2-beep burst = 600 ms, cycle = 944 ms. 3-beep burst = 900 ms,
//    cycle = 1244 ms.
// pause_ms at d=500 mm:  450 - ((2000-500)*300)/1800 = 450 - 250 = 200.

#define TEST_DUTY_AT_1362_MM_PCT    (24u)
#define TEST_DUTY_LOUDEST_PCT       (50u)

static void
test_center_loudness_tracks_distance (void)
{
    send(RTE_AUDIO_OBSTACLE, 1362u, 0u);
    alert_update(0u);
    TEST_ASSERT_EQUAL_UINT32(TEST_DUTY_AT_1362_MM_PCT,
                             mock_buzzer_last_duty_pct());
    // CENTER never turns off, unlike the old pulse-rate-for-distance tone.
    alert_update(5000u);
    TEST_ASSERT_EQUAL_UINT32(TEST_PITCH_AT_1362_MM_HZ,
                             mock_buzzer_last_tone_hz());
}

static void
test_cue_stop_gives_a_continuous_tone_at_max_volume (void)
{
    send(RTE_AUDIO_OBSTACLE, 1362u, RTE_CUE_STOP);
    alert_update(0u);
    TEST_ASSERT_EQUAL_UINT32(TEST_PITCH_AT_1362_MM_HZ,
                             mock_buzzer_last_tone_hz());
    TEST_ASSERT_EQUAL_UINT32(TEST_DUTY_LOUDEST_PCT,
                             mock_buzzer_last_duty_pct());
    // Unlike the plain continuous tone, STOP forces max volume regardless
    // of distance, and (like CENTER) never turns off.
    alert_update(1000u);
    TEST_ASSERT_EQUAL_UINT32(TEST_PITCH_AT_1362_MM_HZ,
                             mock_buzzer_last_tone_hz());
}

#define TEST_CYCLE_LEFT_AT_1362_MM_MS   (944u)

static void
test_cue_left_gives_two_beeps_then_a_pause (void)
{
    uint8_t cue = (uint8_t) (RTE_SIDE_LEFT << RTE_CUE_SIDE_SHIFT);
    bool b_heard_tone = false;

    send(RTE_AUDIO_OBSTACLE, 1362u, cue);

    // Pulse/gap are fixed at 150 ms each; burst is 2 beeps = 600 ms; pause
    // at d=1362 mm is 344 ms (see the comment block above), so the full
    // cycle is 944 ms. One full cycle must contain at least one audible
    // pulse (confirms a real on/off pattern, not silence or a stuck-on
    // tone).
    for (uint32_t t = 0u; t < TEST_CYCLE_LEFT_AT_1362_MM_MS; t += 10u)
    {
        alert_update(t);
        if (0u != mock_buzzer_last_tone_hz())
        {
            b_heard_tone = true;
            TEST_ASSERT_EQUAL_UINT32(TEST_DUTY_AT_1362_MM_PCT,
                                     mock_buzzer_last_duty_pct());
        }
    }
    TEST_ASSERT_TRUE(b_heard_tone);

    // The 344 ms pause after the 600 ms burst (600-944 ms of the cycle)
    // must be silent throughout.
    alert_update(700u);
    TEST_ASSERT_EQUAL_UINT32(0u, mock_buzzer_last_tone_hz());
    alert_update(900u);
    TEST_ASSERT_EQUAL_UINT32(0u, mock_buzzer_last_tone_hz());
}

static void
test_cue_pause_shrinks_when_the_obstacle_is_closer (void)
{
    // Same LEFT cue, two different distances: the closer one (500 mm, pause
    // 200 ms) must already be silent at t=750 ms (600 ms burst + 150 ms
    // into its 200 ms pause), where the farther one (1362 mm, pause 344 ms)
    // is still inside its longer pause too - so instead compare the point
    // where each cycle wraps back into its *next* burst: the near case
    // (cycle 800 ms) has started beeping again by t=810, the far case
    // (cycle 944 ms) has not.
    uint8_t cue = (uint8_t) (RTE_SIDE_LEFT << RTE_CUE_SIDE_SHIFT);

    send(RTE_AUDIO_OBSTACLE, 500u, cue);
    alert_update(810u);
    TEST_ASSERT_NOT_EQUAL_INT(0u, mock_buzzer_last_tone_hz());

    send(RTE_AUDIO_OBSTACLE, 1362u, cue);
    alert_update(810u);
    TEST_ASSERT_EQUAL_UINT32(0u, mock_buzzer_last_tone_hz());
}

static void
test_cue_stop_forces_the_fastest_pause (void)
{
    // STOP + LEFT at 1362 mm: without STOP the pause would be 344 ms
    // (cycle 944 ms, still silent at t=900, see the plain LEFT test above).
    // STOP must force the shortest pause (150 ms, cycle 750 ms) regardless
    // of distance, same way it forces the loudest volume - so by t=760 the
    // next burst has already started.
    uint8_t cue = (uint8_t) (RTE_CUE_STOP |
                             (RTE_SIDE_LEFT << RTE_CUE_SIDE_SHIFT));

    send(RTE_AUDIO_OBSTACLE, 1362u, cue);
    alert_update(760u);
    TEST_ASSERT_NOT_EQUAL_INT(0u, mock_buzzer_last_tone_hz());
    TEST_ASSERT_EQUAL_UINT32(TEST_DUTY_LOUDEST_PCT,
                             mock_buzzer_last_duty_pct());
}

static void
test_cue_right_gives_three_beeps_in_a_longer_burst (void)
{
    uint8_t left_cue = (uint8_t) (RTE_SIDE_LEFT << RTE_CUE_SIDE_SHIFT);
    uint8_t right_cue = (uint8_t) (RTE_SIDE_RIGHT << RTE_CUE_SIDE_SHIFT);

    TEST_ASSERT_NOT_EQUAL_INT(left_cue, right_cue);
    send(RTE_AUDIO_OBSTACLE, 1362u, right_cue);
    alert_update(0u);
    TEST_ASSERT_EQUAL_UINT32(TEST_PITCH_AT_1362_MM_HZ,
                             mock_buzzer_last_tone_hz());
    // 3-beep burst runs 300 ms longer than LEFT's 2-beep burst (600 ms):
    // at t=700 ms, RIGHT's burst (900 ms) is still going (3rd beep's slot
    // starts at 600 ms, tone on for its first 150 ms), where LEFT would
    // already be silent in its pause (see the LEFT test above).
    alert_update(700u);
    TEST_ASSERT_EQUAL_UINT32(TEST_PITCH_AT_1362_MM_HZ,
                             mock_buzzer_last_tone_hz());
}

static void
test_cue_stop_left_keeps_the_beep_pattern_at_max_volume (void)
{
    // Confirmed design B (8 Nov): STOP does not erase the side pattern,
    // it only forces loudness. STOP LEFT must still beep twice, not play
    // one flat continuous tone.
    uint8_t cue = (uint8_t) (RTE_CUE_STOP |
                             (RTE_SIDE_LEFT << RTE_CUE_SIDE_SHIFT));

    send(RTE_AUDIO_OBSTACLE, 1362u, cue);
    alert_update(0u);
    TEST_ASSERT_EQUAL_UINT32(TEST_DUTY_LOUDEST_PCT,
                             mock_buzzer_last_duty_pct());
    // Inside the pause of the 2-beep burst (same fixed timing as LEFT
    // without STOP): must still fall silent, proving the beep pattern
    // survived STOP rather than being overridden into a flat tone.
    alert_update(700u);
    TEST_ASSERT_EQUAL_UINT32(0u, mock_buzzer_last_tone_hz());
}

static void
test_cue_clear_resets_the_cue_for_the_next_obstacle (void)
{
    send(RTE_AUDIO_OBSTACLE, 1362u, RTE_CUE_STOP);
    alert_update(0u);
    send(RTE_AUDIO_OBSTACLE_CLEAR, 0u, 0u);
    // A later OBSTACLE with no cue must not inherit the old STOP cue: the
    // loudness must be back to the plain distance-based value, not STOP's
    // forced max.
    send(RTE_AUDIO_OBSTACLE, 1362u, 0u);
    alert_update(0u);
    TEST_ASSERT_EQUAL_UINT32(TEST_DUTY_AT_1362_MM_PCT,
                             mock_buzzer_last_duty_pct());
}

int
main (void)
{
    UNITY_BEGIN();
    RUN_TEST(test_obstacle_message_gives_the_pitch_of_alert_c);
    RUN_TEST(test_closer_obstacle_is_higher);
    RUN_TEST(test_clear_silences_the_tone);
    RUN_TEST(test_distance_beyond_2000_mm_is_silent);
    RUN_TEST(test_radar_fault_outranks_the_obstacle_tone);
    RUN_TEST(test_radar_ok_brings_the_obstacle_tone_back);
    RUN_TEST(test_null_message_is_ignored);
    RUN_TEST(test_center_loudness_tracks_distance);
    RUN_TEST(test_cue_stop_gives_a_continuous_tone_at_max_volume);
    RUN_TEST(test_cue_left_gives_two_beeps_then_a_pause);
    RUN_TEST(test_cue_pause_shrinks_when_the_obstacle_is_closer);
    RUN_TEST(test_cue_stop_forces_the_fastest_pause);
    RUN_TEST(test_cue_right_gives_three_beeps_in_a_longer_burst);
    RUN_TEST(test_cue_stop_left_keeps_the_beep_pattern_at_max_volume);
    RUN_TEST(test_cue_clear_resets_the_cue_for_the_next_obstacle);
    return (UNITY_END());
}

/*** end of file ***/
