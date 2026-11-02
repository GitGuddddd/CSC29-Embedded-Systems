/** @file test_rte_pack.c
 *
 * @brief Tests of the packed radar queue message (minimum-bit change).
 *
 * @par
 * COPYRIGHT NOTICE: (c) 2026 Group 29. All rights reserved.
 */

#include <stdbool.h>
#include <stdint.h>

#include "unity.h"

#include "mock_hal.h"
#include "rte_pack.h"
#include "rte_type.h"
#include "threat_assessment.h"

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

static rte_radar_data_t
make_data (rte_radar_msg_type_t type)
{
    rte_radar_data_t data;

    data.type = type;
    data.time_ms = 0u;
    data.grip = RTE_GRIP_UNKNOWN;

    for (size_t idx = 0u; idx < RTE_RADAR_NUM_TARGETS; idx++)
    {
        data.targets[idx].b_valid = false;
        data.targets[idx].x_mm = 0;
        data.targets[idx].y_mm = 0;
        data.targets[idx].d_mm = 0u;
        data.targets[idx].speed_cms = 0;
    }

    return (data);
}

static rte_radar_data_t
round_trip (const rte_radar_data_t * p_in)
{
    rte_radar_msg_t msg;
    rte_radar_data_t out;

    rte_pack_radar(p_in, &msg);
    rte_pack_unpack_radar(&msg, &out);

    return (out);
}

static void
test_message_is_13_bytes (void)
{
    TEST_ASSERT_EQUAL_UINT(13u, sizeof(rte_radar_msg_t));
}

static void
test_bit_helpers_round_trip_every_offset_and_width (void)
{
    for (uint32_t width = 1u; width <= 32u; width++)
    {
        for (uint32_t offset = 0u; offset <= 12u; offset++)
        {
            uint8_t buf[RTE_RADAR_MSG_BYTES] = { 0u };
            uint32_t value = 0xA5C3E1F7u;
            uint32_t mask = (32u == width) ? 0xFFFFFFFFu
                                           : ((1u << width) - 1u);

            rte_pack_put_bits(buf, offset, width, value);
            TEST_ASSERT_EQUAL_UINT32(value & mask,
                                     rte_pack_get_bits(buf, offset, width));
        }
    }
}

static void
test_bit_helpers_leave_neighbours_alone (void)
{
    uint8_t buf[RTE_RADAR_MSG_BYTES];

    for (size_t idx = 0u; idx < RTE_RADAR_MSG_BYTES; idx++)
    {
        buf[idx] = 0xFFu;
    }

    rte_pack_put_bits(buf, 5u, 12u, 0u);
    TEST_ASSERT_EQUAL_UINT32(0u, rte_pack_get_bits(buf, 5u, 12u));
    TEST_ASSERT_EQUAL_UINT32(0x1Fu, rte_pack_get_bits(buf, 0u, 5u));
    TEST_ASSERT_EQUAL_UINT32(0x7FFFFu, rte_pack_get_bits(buf, 17u, 19u));
}

static void
test_every_distance_step_round_trips (void)
{
    for (uint32_t steps = 0u; steps <= RTE_PACK_DISTANCE_MAX; steps++)
    {
        rte_radar_data_t in = make_data(RTE_RADAR_MSG_FRAME);
        rte_radar_data_t out;

        in.targets[1].b_valid = true;
        in.targets[1].d_mm = (uint16_t) (steps * 2u);
        in.targets[1].speed_cms = 1;

        out = round_trip(&in);
        TEST_ASSERT_EQUAL_UINT16(in.targets[1].d_mm, out.targets[1].d_mm);
    }
}

static void
test_distance_rounds_down_and_saturates (void)
{
    rte_radar_data_t in = make_data(RTE_RADAR_MSG_FRAME);
    rte_radar_data_t out;

    in.targets[0].b_valid = true;
    in.targets[1].b_valid = true;
    in.targets[2].b_valid = true;
    in.targets[0].d_mm = 1999u;
    in.targets[1].d_mm = 8191u;
    in.targets[2].d_mm = 46339u;

    out = round_trip(&in);
    TEST_ASSERT_EQUAL_UINT16(1998u, out.targets[0].d_mm);
    TEST_ASSERT_EQUAL_UINT16(8190u, out.targets[1].d_mm);
    TEST_ASSERT_EQUAL_UINT16(8190u, out.targets[2].d_mm);
}

static void
test_speed_round_trips_and_clamps (void)
{
    const int16_t speeds[] = { -255, -112, -8, 0, 8, 112, 255 };
    rte_radar_data_t in = make_data(RTE_RADAR_MSG_FRAME);
    rte_radar_data_t out;

    for (size_t idx = 0u; idx < (sizeof(speeds) / sizeof(speeds[0])); idx++)
    {
        in.targets[0].b_valid = true;
        in.targets[0].d_mm = 1000u;
        in.targets[0].speed_cms = speeds[idx];
        out = round_trip(&in);
        TEST_ASSERT_EQUAL_INT16(speeds[idx], out.targets[0].speed_cms);
    }

    in.targets[0].speed_cms = 300;
    out = round_trip(&in);
    TEST_ASSERT_EQUAL_INT16(255, out.targets[0].speed_cms);

    in.targets[0].speed_cms = -32767;
    out = round_trip(&in);
    TEST_ASSERT_EQUAL_INT16(-255, out.targets[0].speed_cms);
}

static void
test_time_type_grip_and_empty_slots (void)
{
    rte_radar_data_t in = make_data(RTE_RADAR_MSG_FRAME);
    rte_radar_data_t out;

    in.time_ms = 0xFFFFFFFFu;
    in.targets[2].b_valid = true;
    in.targets[2].d_mm = 700u;
    in.targets[2].speed_cms = -40;
    out = round_trip(&in);
    TEST_ASSERT_EQUAL_UINT32(0xFFFFFFFFu, out.time_ms);
    TEST_ASSERT_EQUAL(RTE_RADAR_MSG_FRAME, out.type);
    TEST_ASSERT_FALSE(out.targets[0].b_valid);
    TEST_ASSERT_FALSE(out.targets[1].b_valid);
    TEST_ASSERT_TRUE(out.targets[2].b_valid);

    in = make_data(RTE_RADAR_MSG_GRIP);
    in.grip = RTE_GRIP_RELEASED;
    out = round_trip(&in);
    TEST_ASSERT_EQUAL(RTE_RADAR_MSG_GRIP, out.type);
    TEST_ASSERT_EQUAL(RTE_GRIP_RELEASED, out.grip);

    in = make_data(RTE_RADAR_MSG_FAULT);
    out = round_trip(&in);
    TEST_ASSERT_EQUAL(RTE_RADAR_MSG_FAULT, out.type);
}

static void
test_reserved_codes_read_as_the_safe_value (void)
{
    rte_radar_msg_t msg = { { 0u } };
    rte_radar_data_t out;

    rte_pack_put_bits(msg.bytes, 0u, 2u, 3u);
    rte_pack_put_bits(msg.bytes, 2u, 2u, 3u);
    rte_pack_unpack_radar(&msg, &out);
    TEST_ASSERT_EQUAL(RTE_RADAR_MSG_FAULT, out.type);
    TEST_ASSERT_EQUAL(RTE_GRIP_UNKNOWN, out.grip);
}

static void
test_spare_bits_are_zero (void)
{
    rte_radar_data_t in = make_data(RTE_RADAR_MSG_FRAME);
    rte_radar_msg_t msg;

    in.time_ms = 0xFFFFFFFFu;

    for (size_t idx = 0u; idx < RTE_RADAR_NUM_TARGETS; idx++)
    {
        in.targets[idx].b_valid = true;
        in.targets[idx].d_mm = 0xFFFFu;
        in.targets[idx].speed_cms = -1;
    }

    rte_pack_radar(&in, &msg);
    TEST_ASSERT_EQUAL_UINT32(0u, rte_pack_get_bits(msg.bytes, 102u, 2u));
}

// Packed distance feeds Threat: the 2 mm step moves the stop edge by 1 mm.
static threat_assessment_out_t
feed_distance (threat_assessment_t * p_state, uint16_t d_mm, uint32_t now_ms)
{
    rte_radar_data_t in = make_data(RTE_RADAR_MSG_FRAME);
    rte_radar_data_t out;
    threat_assessment_out_t result;

    in.targets[0].b_valid = true;
    in.targets[0].d_mm = d_mm;
    in.targets[0].speed_cms = 0;
    out = round_trip(&in);
    threat_assessment_update(p_state, out.targets, now_ms, &result);

    return (result);
}

static void
test_packed_distance_edges_through_threat (void)
{
    threat_assessment_t state;
    threat_assessment_out_t out;

    threat_assessment_init(&state);

    // Frames are 1 s apart so the range trend (400 ms cap) stays out of the
    // way: this test is about the 2 mm distance edges, not about departing.

    out = feed_distance(&state, 2000u, 1000u);
    TEST_ASSERT_FALSE(out.b_active);

    out = feed_distance(&state, 1999u, 2000u);
    TEST_ASSERT_TRUE(out.b_active);
    TEST_ASSERT_EQUAL_UINT16(1998u, out.d_mm);

    out = feed_distance(&state, 2100u, 3000u);
    TEST_ASSERT_TRUE(out.b_active);

    // 2101 mm is sent as 2100 mm, so it still holds.
    out = feed_distance(&state, 2101u, 4000u);
    TEST_ASSERT_TRUE(out.b_active);

    out = feed_distance(&state, 2102u, 5000u);
    TEST_ASSERT_FALSE(out.b_active);
}

static void
test_saturated_distance_never_alerts (void)
{
    threat_assessment_t state;
    threat_assessment_out_t out;

    threat_assessment_init(&state);
    out = feed_distance(&state, 46339u, 100u);
    TEST_ASSERT_FALSE(out.b_active);
}

int
main (void)
{
    UNITY_BEGIN();
    RUN_TEST(test_message_is_13_bytes);
    RUN_TEST(test_bit_helpers_round_trip_every_offset_and_width);
    RUN_TEST(test_bit_helpers_leave_neighbours_alone);
    RUN_TEST(test_every_distance_step_round_trips);
    RUN_TEST(test_distance_rounds_down_and_saturates);
    RUN_TEST(test_speed_round_trips_and_clamps);
    RUN_TEST(test_time_type_grip_and_empty_slots);
    RUN_TEST(test_reserved_codes_read_as_the_safe_value);
    RUN_TEST(test_spare_bits_are_zero);
    RUN_TEST(test_packed_distance_edges_through_threat);
    RUN_TEST(test_saturated_distance_never_alerts);

    return (UNITY_END());
}

/*** end of file ***/
