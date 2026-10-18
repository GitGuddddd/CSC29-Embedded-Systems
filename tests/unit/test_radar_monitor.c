/** @file test_radar_monitor.c
 *
 * @brief Unit tests of the Glasses_Radar supervision (FR11, statechart 6.6).
 *
 * @par
 * COPYRIGHT NOTICE: (c) 2026 Group 29. All rights reserved.
 */

#include <stdbool.h>
#include <stdint.h>

#include "unity.h"

#include "mock_hal.h"
#include "radar_monitor.h"
#include "rte_type.h"

static radar_monitor_t g_monitor;

void
setUp (void)
{
    mock_reset();
    mock_rte_reset();
    radar_monitor_init(&g_monitor);
}

void
tearDown (void)
{
}

static void
feed_good_frame (uint32_t now_ms)
{
    rte_radar_target_t empty = { false, 0, 0, 0, 0u };
    rte_radar_target_t targets[RTE_RADAR_NUM_TARGETS] = { empty, empty, empty };

    radar_monitor_on_frame(&g_monitor, targets, now_ms);
}

static void
test_starts_in_init_and_asks_for_configuration (void)
{
    TEST_ASSERT_EQUAL(RADAR_MONITOR_ACTION_CONFIGURE,
                      radar_monitor_get_action(&g_monitor));
}

static void
test_config_ok_goes_to_run_with_grace_deadline (void)
{
    radar_monitor_on_config_result(&g_monitor, true, 1000u);

    TEST_ASSERT_EQUAL(RADAR_MONITOR_ACTION_RECEIVE,
                      radar_monitor_get_action(&g_monitor));
    TEST_ASSERT_EQUAL_UINT32(3000u, radar_monitor_get_deadline_ms(&g_monitor));
}

static void
test_valid_frame_restarts_500ms_timer_and_forwards_frame (void)
{
    radar_monitor_on_config_result(&g_monitor, true, 0u);
    feed_good_frame(1200u);

    TEST_ASSERT_EQUAL_UINT32(1700u, radar_monitor_get_deadline_ms(&g_monitor));
    TEST_ASSERT_EQUAL_size_t(1u, mock_radar_frame_count());
    TEST_ASSERT_EQUAL_size_t(0u, mock_audio_count());
}

static void
test_failed_tries_wait_two_seconds_and_fault_after_three (void)
{
    radar_monitor_on_config_result(&g_monitor, false, 0u);
    TEST_ASSERT_EQUAL(RADAR_MONITOR_ACTION_WAIT_RETRY,
                      radar_monitor_get_action(&g_monitor));
    radar_monitor_on_retry_done(&g_monitor);
    TEST_ASSERT_EQUAL(RADAR_MONITOR_ACTION_CONFIGURE,
                      radar_monitor_get_action(&g_monitor));

    radar_monitor_on_config_result(&g_monitor, false, 2000u);
    radar_monitor_on_retry_done(&g_monitor);
    TEST_ASSERT_EQUAL_size_t(0u, mock_audio_count());

    radar_monitor_on_config_result(&g_monitor, false, 4000u);
    TEST_ASSERT_EQUAL_size_t(1u, mock_audio_count());
    TEST_ASSERT_EQUAL(RTE_AUDIO_RADAR_FAULT, mock_audio_get(0u).type);
    TEST_ASSERT_EQUAL_size_t(1u, mock_radar_fault_count());
    TEST_ASSERT_EQUAL(RADAR_MONITOR_ACTION_WAIT_RETRY,
                      radar_monitor_get_action(&g_monitor));
}

static void
test_fault_is_reported_once_while_retries_keep_failing (void)
{
    radar_monitor_on_config_result(&g_monitor, true, 0u);
    radar_monitor_on_timeout(&g_monitor, 500u);
    radar_monitor_on_timeout(&g_monitor, 2500u);

    for (uint32_t tries = 0u; tries < 5u; tries++)
    {
        radar_monitor_on_retry_done(&g_monitor);
        radar_monitor_on_config_result(&g_monitor, false, tries * 2000u);
    }

    TEST_ASSERT_EQUAL_size_t(1u, mock_audio_count());
    TEST_ASSERT_EQUAL_size_t(1u, mock_radar_fault_count());
}

static void
test_timeout_in_run_reports_fault_then_ok_when_frames_resume (void)
{
    radar_monitor_on_config_result(&g_monitor, true, 0u);
    feed_good_frame(100u);
    radar_monitor_on_timeout(&g_monitor, 600u);

    TEST_ASSERT_EQUAL(RTE_AUDIO_RADAR_FAULT, mock_audio_get(0u).type);
    TEST_ASSERT_EQUAL_size_t(1u, mock_radar_fault_count());

    // The monitor keeps listening for 2 s before it configures again.
    TEST_ASSERT_EQUAL(RADAR_MONITOR_ACTION_RECEIVE,
                      radar_monitor_get_action(&g_monitor));
    TEST_ASSERT_EQUAL_UINT32(2600u, radar_monitor_get_deadline_ms(&g_monitor));

    // The sensor resumes by itself: OK once, no new config.
    TEST_ASSERT_EQUAL_size_t(1u, mock_audio_count());
    feed_good_frame(900u);
    feed_good_frame(1040u);
    TEST_ASSERT_EQUAL_size_t(2u, mock_audio_count());
    TEST_ASSERT_EQUAL(RTE_AUDIO_RADAR_OK, mock_audio_get(1u).type);
    TEST_ASSERT_EQUAL(RADAR_MONITOR_ACTION_RECEIVE,
                      radar_monitor_get_action(&g_monitor));
    TEST_ASSERT_EQUAL_UINT32(1540u, radar_monitor_get_deadline_ms(&g_monitor));
}

static void
test_silent_sensor_is_configured_again_after_listening (void)
{
    radar_monitor_on_config_result(&g_monitor, true, 0u);
    feed_good_frame(100u);
    radar_monitor_on_timeout(&g_monitor, 600u);
    radar_monitor_on_timeout(&g_monitor, 2600u);

    TEST_ASSERT_EQUAL(RADAR_MONITOR_ACTION_CONFIGURE,
                      radar_monitor_get_action(&g_monitor));

    radar_monitor_on_config_result(&g_monitor, true, 2700u);
    TEST_ASSERT_EQUAL(RADAR_MONITOR_ACTION_RECEIVE,
                      radar_monitor_get_action(&g_monitor));

    // Still one FAULT, and the OK comes with the first valid frame.
    TEST_ASSERT_EQUAL_size_t(1u, mock_audio_count());
    feed_good_frame(3000u);
    TEST_ASSERT_EQUAL_size_t(2u, mock_audio_count());
    TEST_ASSERT_EQUAL(RTE_AUDIO_RADAR_OK, mock_audio_get(1u).type);
}

static void
test_timeout_outside_run_is_ignored (void)
{
    radar_monitor_on_timeout(&g_monitor, 0u);

    TEST_ASSERT_EQUAL_size_t(0u, mock_audio_count());
    TEST_ASSERT_EQUAL_size_t(0u, mock_radar_fault_count());
}

int
main (void)
{
    UNITY_BEGIN();
    RUN_TEST(test_starts_in_init_and_asks_for_configuration);
    RUN_TEST(test_config_ok_goes_to_run_with_grace_deadline);
    RUN_TEST(test_valid_frame_restarts_500ms_timer_and_forwards_frame);
    RUN_TEST(test_failed_tries_wait_two_seconds_and_fault_after_three);
    RUN_TEST(test_fault_is_reported_once_while_retries_keep_failing);
    RUN_TEST(test_timeout_in_run_reports_fault_then_ok_when_frames_resume);
    RUN_TEST(test_silent_sensor_is_configured_again_after_listening);
    RUN_TEST(test_timeout_outside_run_is_ignored);

    return (UNITY_END());
}

/*** end of file ***/
