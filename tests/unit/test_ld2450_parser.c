/** @file test_ld2450_parser.c
 *
 * @brief Unit tests of the LD2450 frame parser and config handshake (FR7).
 *
 * @par
 * COPYRIGHT NOTICE: (c) 2026 Group 29. All rights reserved.
 */

#include <stdbool.h>
#include <stdint.h>

#include "unity.h"

#include "frame_builder.h"
#include "hal_time.h"
#include "ld2450.h"
#include "mock_hal.h"
#include "rte_type.h"

void
setUp (void)
{
    mock_reset();
    mock_rte_reset();
}

void
tearDown (void)
{
}

static void
test_decode_signed_positive_negative_zero (void)
{
    TEST_ASSERT_EQUAL_INT16(16, ld2450_decode_signed(0x10u, 0x80u));
    TEST_ASSERT_EQUAL_INT16(-16, ld2450_decode_signed(0x10u, 0x00u));
    TEST_ASSERT_EQUAL_INT16(0, ld2450_decode_signed(0x00u, 0x80u));
    TEST_ASSERT_EQUAL_INT16(0, ld2450_decode_signed(0x00u, 0x00u));
    TEST_ASSERT_EQUAL_INT16(32767, ld2450_decode_signed(0xFFu, 0xFFu));
    TEST_ASSERT_EQUAL_INT16(-32767, ld2450_decode_signed(0xFFu, 0x7Fu));
}

static void
test_decode_body_valid_and_empty_slots (void)
{
    uint8_t frame[FRAME_BUILDER_FRAME_LEN];
    rte_radar_target_t targets[RTE_RADAR_NUM_TARGETS];

    frame_builder_build_one(frame, -120, 1500, -16);

    TEST_ASSERT_TRUE(ld2450_decode_body(&frame[4], targets));
    TEST_ASSERT_TRUE(targets[0].b_valid);
    TEST_ASSERT_EQUAL_INT16(-120, targets[0].x_mm);
    TEST_ASSERT_EQUAL_INT16(1500, targets[0].y_mm);
    TEST_ASSERT_EQUAL_INT16(-16, targets[0].speed_cms);
    TEST_ASSERT_EQUAL_UINT16(360, targets[0].resolution_mm);
    TEST_ASSERT_FALSE(targets[1].b_valid);
    TEST_ASSERT_FALSE(targets[2].b_valid);
}

static void
test_decode_body_rejects_bad_footer_and_keeps_output (void)
{
    uint8_t frame[FRAME_BUILDER_FRAME_LEN];
    rte_radar_target_t targets[RTE_RADAR_NUM_TARGETS];

    frame_builder_build_one(frame, 0, 1000, 0);
    targets[0].x_mm = 1234;
    frame[FRAME_BUILDER_FRAME_LEN - 1u] = 0xCDu;

    TEST_ASSERT_FALSE(ld2450_decode_body(&frame[4], targets));
    TEST_ASSERT_EQUAL_INT16(1234, targets[0].x_mm);
}

static void
test_receive_frame_ok (void)
{
    uint8_t frame[FRAME_BUILDER_FRAME_LEN];
    rte_radar_target_t targets[RTE_RADAR_NUM_TARGETS];

    frame_builder_build_one(frame, 0, 800, 8);
    mock_rx_push(frame, sizeof(frame));

    TEST_ASSERT_EQUAL(LD2450_FRAME_OK, ld2450_receive_frame(targets, 500u));
    TEST_ASSERT_EQUAL_INT16(800, targets[0].y_mm);
}

static void
test_receive_frame_resyncs_after_garbage_and_partial_header (void)
{
    static const uint8_t junk[] = { 0x12u, 0xAAu, 0xFFu, 0x99u, 0xAAu, 0xAAu };
    uint8_t frame[FRAME_BUILDER_FRAME_LEN];
    rte_radar_target_t targets[RTE_RADAR_NUM_TARGETS];

    frame_builder_build_one(frame, 0, 900, 0);
    mock_rx_push(junk, sizeof(junk));
    mock_rx_push(frame, sizeof(frame));

    TEST_ASSERT_EQUAL(LD2450_FRAME_OK, ld2450_receive_frame(targets, 500u));
    TEST_ASSERT_EQUAL_INT16(900, targets[0].y_mm);
}

static void
test_receive_frame_bad_footer_counts_and_next_frame_is_read (void)
{
    uint8_t bad[FRAME_BUILDER_FRAME_LEN];
    uint8_t good[FRAME_BUILDER_FRAME_LEN];
    rte_radar_target_t targets[RTE_RADAR_NUM_TARGETS];
    uint32_t before = ld2450_get_bad_footer_count();

    frame_builder_build_one(bad, 0, 700, 0);
    frame_builder_build_one(good, 0, 650, 0);
    bad[FRAME_BUILDER_FRAME_LEN - 2u] = 0x00u;
    mock_rx_push(bad, sizeof(bad));
    mock_rx_push(good, sizeof(good));

    TEST_ASSERT_EQUAL(LD2450_FRAME_BAD_FOOTER,
                      ld2450_receive_frame(targets, 500u));
    TEST_ASSERT_EQUAL_UINT32(before + 1u, ld2450_get_bad_footer_count());
    TEST_ASSERT_EQUAL(LD2450_FRAME_OK, ld2450_receive_frame(targets, 500u));
    TEST_ASSERT_EQUAL_INT16(650, targets[0].y_mm);
}

static void
test_receive_frame_short_body (void)
{
    uint8_t frame[FRAME_BUILDER_FRAME_LEN];
    rte_radar_target_t targets[RTE_RADAR_NUM_TARGETS];

    frame_builder_build_one(frame, 0, 700, 0);
    mock_rx_push(frame, 15u);

    TEST_ASSERT_EQUAL(LD2450_FRAME_SHORT, ld2450_receive_frame(targets, 500u));
}

static void
test_receive_frame_timeout_when_silent (void)
{
    rte_radar_target_t targets[RTE_RADAR_NUM_TARGETS];

    mock_time_set_ms(100u);

    TEST_ASSERT_EQUAL(LD2450_FRAME_TIMEOUT,
                      ld2450_receive_frame(targets, 600u));
    TEST_ASSERT_EQUAL_UINT32(600u, hal_time_now_ms());
}

static void
test_receive_frame_deadline_already_passed (void)
{
    rte_radar_target_t targets[RTE_RADAR_NUM_TARGETS];

    mock_time_set_ms(1000u);

    TEST_ASSERT_EQUAL(LD2450_FRAME_TIMEOUT,
                      ld2450_receive_frame(targets, 500u));
}

static void
test_receive_frame_deadline_across_clock_wrap (void)
{
    uint8_t frame[FRAME_BUILDER_FRAME_LEN];
    rte_radar_target_t targets[RTE_RADAR_NUM_TARGETS];

    frame_builder_build_one(frame, 0, 500, 0);
    mock_time_set_ms(0xFFFFFF00u);
    mock_rx_push(frame, sizeof(frame));

    // The deadline is 0x1F4 ms after the wrap; the frame is already there.
    TEST_ASSERT_EQUAL(LD2450_FRAME_OK, ld2450_receive_frame(targets, 0x1F4u));
}

static void
test_configure_succeeds_when_every_command_is_acked (void)
{
    mock_rx_set_ack_on_write(true);

    TEST_ASSERT_TRUE(ld2450_configure());
    TEST_ASSERT_EQUAL_size_t(3u, mock_tx_count());
}

static void
test_configure_fails_without_ack_after_first_command (void)
{
    mock_rx_set_ack_on_write(false);

    TEST_ASSERT_FALSE(ld2450_configure());
    TEST_ASSERT_EQUAL_size_t(1u, mock_tx_count());
}

int
main (void)
{
    UNITY_BEGIN();
    RUN_TEST(test_decode_signed_positive_negative_zero);
    RUN_TEST(test_decode_body_valid_and_empty_slots);
    RUN_TEST(test_decode_body_rejects_bad_footer_and_keeps_output);
    RUN_TEST(test_receive_frame_ok);
    RUN_TEST(test_receive_frame_resyncs_after_garbage_and_partial_header);
    RUN_TEST(test_receive_frame_bad_footer_counts_and_next_frame_is_read);
    RUN_TEST(test_receive_frame_short_body);
    RUN_TEST(test_receive_frame_timeout_when_silent);
    RUN_TEST(test_receive_frame_deadline_already_passed);
    RUN_TEST(test_receive_frame_deadline_across_clock_wrap);
    RUN_TEST(test_configure_succeeds_when_every_command_is_acked);
    RUN_TEST(test_configure_fails_without_ack_after_first_command);

    return (UNITY_END());
}

/*** end of file ***/
