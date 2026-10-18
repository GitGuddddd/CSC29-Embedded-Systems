/** @file test_alert_pipeline.c
 *
 * @brief Integration test: LD2450 bytes -> radar_monitor -> threat_assessment
 *        -> Audio Alert and BLE messages (FR7 to FR11, FR13).
 *
 * @par
 * COPYRIGHT NOTICE: (c) 2026 Group 29. All rights reserved.
 *
 * The test plays the roles of the two tasks. It calls the same component
 * functions as rte.c, in the same order, with the mock RTE in between.
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "unity.h"

#include "frame_builder.h"
#include "hal_time.h"
#include "ld2450.h"
#include "mock_hal.h"
#include "radar_monitor.h"
#include "rte_type.h"
#include "threat_assessment.h"

#define FRAME_PERIOD_MS    (140u)

static radar_monitor_t g_monitor;
static threat_assessment_t g_threat;
static size_t g_threat_cursor = 0u;

void
setUp (void)
{
    mock_reset();
    mock_rte_reset();
    radar_monitor_init(&g_monitor);
    threat_assessment_init(&g_threat);
    g_threat_cursor = 0u;
}

void
tearDown (void)
{
}

// Glasses_Threat: consume radar messages the monitor produced so far.
static void
run_threat_task (void)
{
    while (g_threat_cursor < (mock_radar_frame_count() +
                              mock_radar_fault_count()))
    {
        rte_radar_msg_t msg = mock_radar_get(g_threat_cursor);

        if (RTE_RADAR_MSG_FRAME == msg.type)
        {
            threat_assessment_process_frame(&g_threat, msg.targets,
                                            msg.time_ms);
        }
        else
        {
            threat_assessment_process_fault(&g_threat);
        }

        g_threat_cursor++;
    }
}

// Glasses_Radar: one pass through the monitor loop.
static void
run_radar_step (void)
{
    rte_radar_target_t targets[RTE_RADAR_NUM_TARGETS];
    ld2450_frame_status_t status = LD2450_FRAME_TIMEOUT;

    switch (radar_monitor_get_action(&g_monitor))
    {
        case RADAR_MONITOR_ACTION_CONFIGURE:
            radar_monitor_on_config_result(&g_monitor, ld2450_configure(),
                                           hal_time_now_ms());
            break;

        case RADAR_MONITOR_ACTION_WAIT_RETRY:
            mock_time_advance_ms(2000u);
            radar_monitor_on_retry_done(&g_monitor);
            break;

        case RADAR_MONITOR_ACTION_RECEIVE:
            status = ld2450_receive_frame(
                targets, radar_monitor_get_deadline_ms(&g_monitor));

            if (LD2450_FRAME_OK == status)
            {
                radar_monitor_on_frame(&g_monitor, targets, hal_time_now_ms());
            }
            else if (LD2450_FRAME_TIMEOUT == status)
            {
                radar_monitor_on_timeout(&g_monitor, hal_time_now_ms());
            }
            else
            {
                // Short body or bad footer: resync on the next call.
            }
            break;

        default:
            break;
    }

    run_threat_task();
}

static void
send_frame (int16_t y_mm, int16_t speed_cms)
{
    uint8_t frame[FRAME_BUILDER_FRAME_LEN];

    frame_builder_build_one(frame, 0, y_mm, speed_cms);
    mock_rx_push(frame, sizeof(frame));
    mock_time_advance_ms(FRAME_PERIOD_MS);
    run_radar_step();
}

static size_t
count_audio (rte_audio_msg_type_t type)
{
    size_t count = 0u;

    for (size_t idx = 0u; idx < mock_audio_count(); idx++)
    {
        if (type == mock_audio_get(idx).type)
        {
            count++;
        }
    }

    return (count);
}

static void
bring_radar_up (void)
{
    mock_rx_set_ack_on_write(true);
    run_radar_step();
    mock_rx_set_ack_on_write(false);
    TEST_ASSERT_EQUAL(RADAR_MONITOR_ACTION_RECEIVE,
                      radar_monitor_get_action(&g_monitor));
}

static void
test_person_walks_up_alert_starts_then_clears (void)
{
    bring_radar_up();

    // Far away: no alert.
    send_frame(3000, -20);
    send_frame(2500, -20);
    TEST_ASSERT_EQUAL_size_t(0u, mock_audio_count());

    // Crossing 2000 mm starts the alert, then OBSTACLE every frame.
    send_frame(1900, -20);
    send_frame(1700, -20);
    send_frame(1500, -20);
    TEST_ASSERT_EQUAL_size_t(3u, count_audio(RTE_AUDIO_OBSTACLE));
    TEST_ASSERT_EQUAL_UINT16(1500u, mock_audio_get(2u).d_mm);
    TEST_ASSERT_TRUE(mock_ble_get(2u).b_active);
    TEST_ASSERT_EQUAL_UINT16(7500u, mock_ble_get(2u).ttc_ms);

    // Walking away beyond 2100 mm clears it exactly once.
    send_frame(2050, 0);
    send_frame(2200, 0);
    send_frame(2300, 0);
    TEST_ASSERT_EQUAL_size_t(1u, count_audio(RTE_AUDIO_OBSTACLE_CLEAR));
    TEST_ASSERT_FALSE(mock_ble_get(mock_ble_count() - 1u).b_active);
}

static void
test_radar_dies_while_alert_active_clears_and_reports_fault (void)
{
    bring_radar_up();
    send_frame(1200, -10);
    send_frame(1100, -10);
    TEST_ASSERT_EQUAL_size_t(2u, count_audio(RTE_AUDIO_OBSTACLE));

    // No more bytes: the 500 ms deadline passes and the monitor times out.
    run_radar_step();

    TEST_ASSERT_EQUAL_size_t(1u, count_audio(RTE_AUDIO_RADAR_FAULT));
    TEST_ASSERT_EQUAL_size_t(1u, count_audio(RTE_AUDIO_OBSTACLE_CLEAR));
    TEST_ASSERT_FALSE(mock_ble_get(mock_ble_count() - 1u).b_active);

    // The sensor comes back: config acked, first frame sends RADAR_OK.
    run_radar_step();
    mock_rx_set_ack_on_write(true);
    run_radar_step();
    mock_rx_set_ack_on_write(false);
    send_frame(1000, 0);

    TEST_ASSERT_EQUAL_size_t(1u, count_audio(RTE_AUDIO_RADAR_OK));
    TEST_ASSERT_EQUAL_size_t(3u, count_audio(RTE_AUDIO_OBSTACLE));
}

static void
test_corrupted_frames_do_not_start_an_alert (void)
{
    uint8_t frame[FRAME_BUILDER_FRAME_LEN];

    bring_radar_up();
    frame_builder_build_one(frame, 0, 300, 0);
    frame[FRAME_BUILDER_FRAME_LEN - 1u] = 0x00u;
    mock_rx_push(frame, sizeof(frame));
    mock_time_advance_ms(FRAME_PERIOD_MS);
    run_radar_step();

    TEST_ASSERT_EQUAL_size_t(0u, mock_audio_count());
    TEST_ASSERT_EQUAL_size_t(0u, mock_radar_frame_count());
}

int
main (void)
{
    UNITY_BEGIN();
    RUN_TEST(test_person_walks_up_alert_starts_then_clears);
    RUN_TEST(test_radar_dies_while_alert_active_clears_and_reports_fault);
    RUN_TEST(test_corrupted_frames_do_not_start_an_alert);

    return (UNITY_END());
}

/*** end of file ***/
