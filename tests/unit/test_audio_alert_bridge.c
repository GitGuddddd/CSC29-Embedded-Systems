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
send (rte_audio_msg_type_t type, uint16_t d_mm)
{
    rte_audio_msg_t msg = { .type = type, .d_mm = d_mm };

    audio_alert_bridge_apply(&msg);
}

void
setUp (void)
{
    send(RTE_AUDIO_OBSTACLE_CLEAR, 0u);
    send(RTE_AUDIO_RADAR_OK, 0u);
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
    send(RTE_AUDIO_OBSTACLE, 1362u);
    alert_update(0u);
    TEST_ASSERT_EQUAL_UINT32(TEST_PITCH_AT_1362_MM_HZ, mock_buzzer_last_tone_hz());
}

static void
test_closer_obstacle_is_higher (void)
{
    send(RTE_AUDIO_OBSTACLE, 500u);
    alert_update(0u);
    TEST_ASSERT_EQUAL_UINT32(TEST_PITCH_AT_500_MM_HZ, mock_buzzer_last_tone_hz());
}

static void
test_clear_silences_the_tone (void)
{
    send(RTE_AUDIO_OBSTACLE, 1362u);
    alert_update(0u);
    send(RTE_AUDIO_OBSTACLE_CLEAR, 0u);
    alert_update(0u);
    TEST_ASSERT_EQUAL_UINT32(0u, mock_buzzer_last_tone_hz());
}

static void
test_distance_beyond_2000_mm_is_silent (void)
{
    send(RTE_AUDIO_OBSTACLE, 2002u);
    alert_update(0u);
    TEST_ASSERT_EQUAL_UINT32(0u, mock_buzzer_last_tone_hz());
}

static void
test_radar_fault_outranks_the_obstacle_tone (void)
{
    send(RTE_AUDIO_OBSTACLE, 1362u);
    send(RTE_AUDIO_RADAR_FAULT, 0u);
    alert_update(0u);
    TEST_ASSERT_EQUAL_UINT32(TEST_FAULT_TONE_HZ, mock_buzzer_last_tone_hz());
    alert_update(150u);
    TEST_ASSERT_EQUAL_UINT32(0u, mock_buzzer_last_tone_hz());
}

static void
test_radar_ok_brings_the_obstacle_tone_back (void)
{
    send(RTE_AUDIO_OBSTACLE, 1362u);
    send(RTE_AUDIO_RADAR_FAULT, 0u);
    alert_update(0u);
    send(RTE_AUDIO_RADAR_OK, 0u);
    alert_update(0u);
    TEST_ASSERT_EQUAL_UINT32(TEST_PITCH_AT_1362_MM_HZ, mock_buzzer_last_tone_hz());
}

static void
test_null_message_is_ignored (void)
{
    audio_alert_bridge_apply(NULL);
    alert_update(0u);
    TEST_ASSERT_EQUAL_UINT32(0u, mock_buzzer_last_tone_hz());
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
    return (UNITY_END());
}

/*** end of file ***/
