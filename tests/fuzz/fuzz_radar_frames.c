/** @file fuzz_radar_frames.c
 *
 * @brief Fuzz test: corrupted UART streams and random targets.
 *
 * @par
 * COPYRIGHT NOTICE: (c) 2026 Group 29. All rights reserved.
 *
 * A fixed-seed generator makes every run repeatable. The test checks
 * invariants, not exact outputs: no crash, no out-of-range value, and the
 * obstacle state machine never posts inconsistent messages.
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "unity.h"

#include "frame_builder.h"
#include "ld2450.h"
#include "mock_hal.h"
#include "rte_type.h"
#include "threat_assessment.h"

#define FUZZ_ITERATIONS        (3000u)
#define FUZZ_MAX_DISTANCE_MM   (46339u)
#define FUZZ_MAX_CALLS         (400u)
#define FUZZ_LCG_MULTIPLIER    (1664525u)
#define FUZZ_LCG_INCREMENT     (1013904223u)

static uint32_t g_seed = 12345u;

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

// Linear congruential generator, good enough for test data.
static uint32_t
next_random (void)
{
    g_seed = (g_seed * FUZZ_LCG_MULTIPLIER) + FUZZ_LCG_INCREMENT;

    return (g_seed >> 8u);
}

static int16_t
random_signed (uint32_t range)
{
    // Test code: range is below 32768, so the result fits int16_t.
    return ((int16_t) ((int32_t) (next_random() % (2u * range + 1u)) -
                       (int32_t) range));
}

static void
test_parser_survives_corrupted_streams (void)
{
    uint8_t frame[FRAME_BUILDER_FRAME_LEN];
    rte_radar_target_t targets[RTE_RADAR_NUM_TARGETS];
    size_t ok_frames = 0u;

    for (uint32_t iter = 0u; iter < FUZZ_ITERATIONS; iter++)
    {
        frame_builder_target_t input[RTE_RADAR_NUM_TARGETS];
        uint32_t damage = next_random() % 4u;
        size_t length = FRAME_BUILDER_FRAME_LEN;

        for (size_t idx = 0u; idx < RTE_RADAR_NUM_TARGETS; idx++)
        {
            input[idx].x_mm = random_signed(32767u);
            input[idx].y_mm = random_signed(32767u);
            input[idx].speed_cms = random_signed(32767u);
        }

        frame_builder_build(frame, input);

        if (0u == damage)
        {
            frame[next_random() % FRAME_BUILDER_FRAME_LEN] ^=
                (uint8_t) (1u << (next_random() % 8u));
        }
        else if (1u == damage)
        {
            length = next_random() % FRAME_BUILDER_FRAME_LEN;
        }
        else if (2u == damage)
        {
            frame[next_random() % FRAME_BUILDER_FRAME_LEN] =
                (uint8_t) next_random();
        }
        else
        {
            // Undamaged frame.
        }

        mock_reset();
        mock_rx_push(frame, length);

        ld2450_frame_status_t status = ld2450_receive_frame(targets, 600u);

        if (LD2450_FRAME_OK == status)
        {
            ok_frames++;
        }

        TEST_ASSERT_TRUE(status <= LD2450_FRAME_BAD_FOOTER);
    }

    TEST_ASSERT_TRUE(ok_frames > 0u);
}

static void
test_pure_noise_always_terminates (void)
{
    uint8_t noise[256];
    rte_radar_target_t targets[RTE_RADAR_NUM_TARGETS];

    for (uint32_t iter = 0u; iter < FUZZ_ITERATIONS; iter++)
    {
        ld2450_frame_status_t status = LD2450_FRAME_OK;
        uint32_t calls = 0u;

        mock_reset();

        for (size_t idx = 0u; idx < sizeof(noise); idx++)
        {
            noise[idx] = (uint8_t) next_random();
        }

        mock_rx_push(noise, sizeof(noise));

        // Each call either consumes bytes or times out, so this ends.
        do
        {
            status = ld2450_receive_frame(targets, 600u);
            calls++;
        } while ((LD2450_FRAME_TIMEOUT != status) &&
                 (calls < FUZZ_MAX_CALLS));

        TEST_ASSERT_TRUE(calls < FUZZ_MAX_CALLS);
    }
}

static void
test_threat_state_machine_invariants_on_random_targets (void)
{
    threat_assessment_t state;
    uint32_t now_ms = 0u;
    bool b_was_active = false;

    threat_assessment_init(&state);

    for (uint32_t iter = 0u; iter < FUZZ_ITERATIONS; iter++)
    {
        rte_radar_target_t targets[RTE_RADAR_NUM_TARGETS];
        threat_assessment_out_t out;

        for (size_t idx = 0u; idx < RTE_RADAR_NUM_TARGETS; idx++)
        {
            bool b_empty = (0u == (next_random() % 3u));

            targets[idx].b_valid = !b_empty;
            targets[idx].x_mm = b_empty ? 0 : random_signed(4000u);
            targets[idx].y_mm = b_empty ? 0 : random_signed(4000u);
            targets[idx].speed_cms = b_empty ? 0 : random_signed(100u);
            targets[idx].resolution_mm = 360u;
        }

        now_ms += 100u + (next_random() % 100u);
        threat_assessment_update(&state, targets, now_ms, &out);

        TEST_ASSERT_EQUAL(out.b_active, state.b_active);
        TEST_ASSERT_EQUAL(out.b_post_obstacle, out.b_active);
        TEST_ASSERT_TRUE(out.d_mm <= FUZZ_MAX_DISTANCE_MM);
        TEST_ASSERT_TRUE(!((out.b_post_clear) && (out.b_active)));
        TEST_ASSERT_TRUE(!((out.b_post_clear) && (!b_was_active)));
        b_was_active = out.b_active;
    }
}

int
main (void)
{
    UNITY_BEGIN();
    RUN_TEST(test_parser_survives_corrupted_streams);
    RUN_TEST(test_pure_noise_always_terminates);
    RUN_TEST(test_threat_state_machine_invariants_on_random_targets);

    return (UNITY_END());
}

/*** end of file ***/
