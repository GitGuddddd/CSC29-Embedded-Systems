#include <math.h>

#include "unity.h"
#include "app/FallDetection/CaneMotion.h"

#define SAMPLE_PERIOD_MS (10U)
#define EVENT_BIT(event) (UINT32_C(1) << (uint32_t)(event))
#define ALARM_BITS (EVENT_BIT(CANE_MOTION_EVENT_DROP) | EVENT_BIT(CANE_MOTION_EVENT_FALL))

static CaneMotion_t s_motion;
static CaneMotion_AlarmPolicy_t s_alarm_policy;
static uint32_t s_next_sample_ms;

void setUp(void)
{
    s_motion = (CaneMotion_t){0};
    s_alarm_policy = CANE_MOTION_ALARM_FULL;
    s_next_sample_ms = 0U;
}

void tearDown(void)
{
}

static CaneMotion_Result_t Step(Imu_Sample_t sample, Grip_State_t grip)
{
    sample.timestamp_ms = s_next_sample_ms;
    s_next_sample_ms += SAMPLE_PERIOD_MS;
    return CaneMotion_Step(&s_motion, &sample, grip, s_alarm_policy);
}

static uint32_t Feed(Imu_Sample_t sample, Grip_State_t grip, uint32_t duration_ms)
{
    uint32_t events = 0U;
    for (uint32_t elapsed_ms = 0U; elapsed_ms < duration_ms; elapsed_ms += SAMPLE_PERIOD_MS)
    {
        events |= EVENT_BIT(Step(sample, grip).event);
    }
    return events;
}

static uint32_t Rotate_Quarter_Turn(void)
{
    uint32_t events = 0U;
    for (unsigned int degrees = 3U; degrees <= 90U; degrees += 3U)
    {
        const float radians = (float)degrees * 0.01745329252f;
        const Imu_Sample_t turning = { .ax = sinf(radians), .az = cosf(radians), .gy = -300.0f };
        events |= EVENT_BIT(Step(turning, GRIP_STATE_HELD).event);
    }
    return events;
}

static uint32_t Rotate_While_Accelerating_Down(unsigned int rotation_degrees)
{
    uint32_t events = 0U;
    for (unsigned int degrees = 1U; degrees <= rotation_degrees; ++degrees)
    {
        const float radians = (float)degrees * 0.01745329252f;
        const Imu_Sample_t rotating_down = {
            .ax = 0.35f * sinf(radians), .az = 0.35f * cosf(radians), .gy = -100.0f
        };
        events |= EVENT_BIT(Step(rotating_down, GRIP_STATE_HELD).event);
    }
    return events;
}

void test_CaneMotion_TiltedLiftAndBriefLowGDoNotAlarm(void)
{
    const Imu_Sample_t still = { .ax = 0.6f, .az = -0.8f };
    const Imu_Sample_t upward = { .ax = 1.8f, .az = -2.4f };
    const Imu_Sample_t low_g = { .ax = 0.06f, .az = -0.08f };
    (void)Feed(still, GRIP_STATE_HELD, 750U);

    CaneMotion_Result_t result = Step(upward, GRIP_STATE_HELD);
    TEST_ASSERT_TRUE(result.up_valid);
    TEST_ASSERT_FLOAT_WITHIN(0.02f, 2.0f, result.vertical_raw_g);
    TEST_ASSERT_FLOAT_WITHIN(0.02f, 0.0f, result.vertical_g);
    TEST_ASSERT_FLOAT_WITHIN(0.02f, 0.0f, result.horizontal_g);
    TEST_ASSERT_EQUAL(CANE_MOTION_EVENT_NONE, result.event);

    uint32_t events = EVENT_BIT(result.event);
    events |= Feed(upward, GRIP_STATE_HELD, 50U);
    events |= Feed(low_g, GRIP_STATE_RELEASED, 100U);
    events |= EVENT_BIT(Step(upward, GRIP_STATE_RELEASED).event); /* Abrupt catch. */
    events |= Feed(still, GRIP_STATE_HELD, 1600U);
    TEST_ASSERT_BITS_HIGH(EVENT_BIT(CANE_MOTION_EVENT_LIFT), events);
    TEST_ASSERT_BITS_LOW(ALARM_BITS | EVENT_BIT(CANE_MOTION_EVENT_SLIP), events);
}

void test_CaneMotion_SustainedReleasedDropStillAlarmsAfterLift(void)
{
    const Imu_Sample_t still = { .az = 1.0f };
    const Imu_Sample_t upward = { .az = 1.8f };
    const Imu_Sample_t freefall = {0};
    (void)Feed(still, GRIP_STATE_HELD, 750U);
    uint32_t events = Feed(upward, GRIP_STATE_HELD, 40U);
    TEST_ASSERT_BITS_HIGH(EVENT_BIT(CANE_MOTION_EVENT_LIFT), events);

    events = Feed(freefall, GRIP_STATE_RELEASED, 100U);
    TEST_ASSERT_BITS_LOW(ALARM_BITS, events);
    events = Feed(freefall, GRIP_STATE_RELEASED, 100U);
    TEST_ASSERT_BITS_HIGH(EVENT_BIT(CANE_MOTION_EVENT_DROP), events);
}

void test_CaneMotion_ShortDropAfterLiftCanConfirmWithTilt(void)
{
    const Imu_Sample_t still = { .az = 1.0f };
    const Imu_Sample_t upward = { .az = 3.0f };
    const Imu_Sample_t freefall = {0};
    const Imu_Sample_t tilted = { .ax = 1.0f };
    (void)Feed(still, GRIP_STATE_HELD, 750U);
    (void)Feed(upward, GRIP_STATE_HELD, 40U);

    uint32_t events = Feed(freefall, GRIP_STATE_RELEASED, 70U);
    events |= EVENT_BIT(Step(upward, GRIP_STATE_RELEASED).event);
    TEST_ASSERT_BITS_LOW(ALARM_BITS, events);

    events |= Rotate_Quarter_Turn();
    events |= Feed(tilted, GRIP_STATE_RELEASED, 350U);
    TEST_ASSERT_BITS_HIGH(EVENT_BIT(CANE_MOTION_EVENT_FALL), events);
}

void test_CaneMotion_ReleasedFreefallDoesNotRequireTiltOrImpact(void)
{
    const Imu_Sample_t still = { .az = 1.0f };
    const Imu_Sample_t freefall = {0};
    (void)Feed(still, GRIP_STATE_HELD, 750U);
    uint32_t drop_count = 0U;
    for (uint32_t elapsed_ms = 0U; elapsed_ms < 200U; elapsed_ms += SAMPLE_PERIOD_MS)
    {
        if (Step(freefall, GRIP_STATE_RELEASED).event == CANE_MOTION_EVENT_DROP)
        {
            ++drop_count;
        }
    }
    TEST_ASSERT_EQUAL_UINT32(1U, drop_count);
}

void test_CaneMotion_UpwardImpactCanConfirmReleasedDrop(void)
{
    const Imu_Sample_t still = { .az = 1.0f };
    const Imu_Sample_t freefall = {0};
    const Imu_Sample_t landing_impact = { .az = 3.0f };
    (void)Feed(still, GRIP_STATE_HELD, 750U);
    uint32_t events = Feed(freefall, GRIP_STATE_RELEASED, 40U);
    TEST_ASSERT_BITS_LOW(ALARM_BITS, events);
    TEST_ASSERT_EQUAL(CANE_MOTION_EVENT_DROP, Step(landing_impact, GRIP_STATE_RELEASED).event);
}

void test_CaneMotion_ShortUpwardSpikesCannotMaskShortReleasedDrop(void)
{
    const Imu_Sample_t still = { .az = 1.0f };
    const Imu_Sample_t spike = { .az = 2.1f };
    const Imu_Sample_t freefall = {0};
    const Imu_Sample_t landing_impact = { .az = 3.0f };

    for (uint32_t spike_count = 1U; spike_count <= 2U; ++spike_count)
    {
        setUp();
        (void)Feed(still, GRIP_STATE_HELD, 750U);
        CaneMotion_Result_t result = Step(spike, GRIP_STATE_HELD);
        TEST_ASSERT_FLOAT_WITHIN(0.02f, 1.1f, result.vertical_raw_g);
        TEST_ASSERT_FLOAT_WITHIN(0.02f, 0.0f, result.vertical_g);

        uint32_t events = EVENT_BIT(result.event);
        events |= Feed(spike, GRIP_STATE_HELD, (spike_count - 1U) * SAMPLE_PERIOD_MS);
        events |= Feed(freefall, GRIP_STATE_RELEASED, 40U);
        TEST_ASSERT_BITS_LOW(ALARM_BITS | EVENT_BIT(CANE_MOTION_EVENT_LIFT), events);
        TEST_ASSERT_EQUAL(CANE_MOTION_EVENT_DROP, Step(landing_impact, GRIP_STATE_RELEASED).event);
    }
}

void test_CaneMotion_QuietOppositeSampleCannotSeedWorldUp(void)
{
    const Imu_Sample_t opposite = { .az = 1.0f };
    const Imu_Sample_t still = { .az = -1.0f };
    const Imu_Sample_t upward = { .az = -1.8f };

    TEST_ASSERT_FALSE(Step(opposite, GRIP_STATE_HELD).up_valid);
    TEST_ASSERT_FALSE(Step(still, GRIP_STATE_HELD).up_valid);
    TEST_ASSERT_FALSE(Step(still, GRIP_STATE_HELD).up_valid);
    CaneMotion_Result_t result = Step(still, GRIP_STATE_HELD);
    TEST_ASSERT_TRUE(result.up_valid);
    TEST_ASSERT_FLOAT_WITHIN(0.02f, 0.0f, result.vertical_raw_g);

    uint32_t events = Feed(upward, GRIP_STATE_HELD, 40U);
    TEST_ASSERT_BITS_HIGH(EVENT_BIT(CANE_MOTION_EVENT_LIFT), events);
    TEST_ASSERT_BITS_LOW(ALARM_BITS | EVENT_BIT(CANE_MOTION_EVENT_SLIP), events);
}

void test_CaneMotion_AlternatingVerticalOutliersDoNotAccumulateEvidence(void)
{
    const Imu_Sample_t still = { .az = 1.0f };
    const Imu_Sample_t upward_spike = { .az = 1.8f, .gz = 100.0f };
    const Imu_Sample_t downward_spike = { .az = 0.35f, .gz = 100.0f };
    (void)Feed(still, GRIP_STATE_HELD, 750U);

    CaneMotion_Result_t first_dip = Step(downward_spike, GRIP_STATE_HELD);
    TEST_ASSERT_FLOAT_WITHIN(0.02f, -0.65f, first_dip.vertical_raw_g);
    TEST_ASSERT_FLOAT_WITHIN(0.02f, 0.0f, first_dip.vertical_g);

    uint32_t events = EVENT_BIT(first_dip.event);
    for (unsigned int frames = 0U; frames < 12U; ++frames)
    {
        events |= EVENT_BIT(Step(upward_spike, GRIP_STATE_HELD).event);
        events |= EVENT_BIT(Step(downward_spike, GRIP_STATE_HELD).event);
    }
    events |= Feed(still, GRIP_STATE_HELD, 100U);
    TEST_ASSERT_BITS_LOW(ALARM_BITS | EVENT_BIT(CANE_MOTION_EVENT_LIFT) |
                         EVENT_BIT(CANE_MOTION_EVENT_SLIP), events);
}

void test_CaneMotion_ObservesSlipInOppositeAndDiagonalDirections(void)
{
    const Imu_Sample_t still = { .az = 1.0f };
    const Imu_Sample_t sideways[] = {
        { .ax = 0.9f, .az = 1.0f },
        { .ax = -0.9f, .az = 1.0f },
        { .ay = 0.9f, .az = 1.0f },
        { .ay = -0.9f, .az = 1.0f },
        { .ax = -0.65f, .ay = 0.65f, .az = 1.0f }
    };

    for (size_t i = 0U; i < (sizeof(sideways) / sizeof(sideways[0])); ++i)
    {
        setUp();
        (void)Feed(still, GRIP_STATE_HELD, 750U);
        uint32_t events = Feed(sideways[i], GRIP_STATE_HELD, 80U);
        TEST_ASSERT_BITS_HIGH(EVENT_BIT(CANE_MOTION_EVENT_SLIP), events);

        events |= Feed(still, GRIP_STATE_HELD, 1500U);
        TEST_ASSERT_BITS_LOW(ALARM_BITS, events);
    }
}

void test_CaneMotion_SideSlipSurvivesBriefLowGAndSustainedTiltConfirmsFall(void)
{
    const Imu_Sample_t still = { .az = 1.0f };
    const Imu_Sample_t sideways = { .ax = 0.9f, .az = 1.0f };
    const Imu_Sample_t low_g = { .az = 0.2f };
    const Imu_Sample_t tilted = { .ax = 1.0f };
    (void)Feed(still, GRIP_STATE_HELD, 750U);
    uint32_t events = Feed(sideways, GRIP_STATE_HELD, 60U);
    TEST_ASSERT_BITS_HIGH(EVENT_BIT(CANE_MOTION_EVENT_SLIP), events);
    TEST_ASSERT_BITS_LOW(ALARM_BITS, events);

    events |= Feed(low_g, GRIP_STATE_HELD, 40U);
    events |= Rotate_Quarter_Turn();
    TEST_ASSERT_BITS_LOW(ALARM_BITS, events);
    events |= Feed(tilted, GRIP_STATE_HELD, 500U);
    TEST_ASSERT_BITS_HIGH(EVENT_BIT(CANE_MOTION_EVENT_FALL), events);
}

void test_CaneMotion_GyroTracksPureRotationWithoutReportingSlip(void)
{
    const Imu_Sample_t still = { .az = 1.0f };
    const Imu_Sample_t tilted = { .ax = 1.0f };
    (void)Feed(still, GRIP_STATE_HELD, 750U);
    uint32_t events = Rotate_Quarter_Turn();

    CaneMotion_Result_t result = Step(tilted, GRIP_STATE_HELD);
    TEST_ASSERT_TRUE(result.up_valid);
    TEST_ASSERT_FLOAT_WITHIN(0.02f, 0.0f, result.vertical_g);
    TEST_ASSERT_FLOAT_WITHIN(0.02f, 0.0f, result.horizontal_g);
    events |= EVENT_BIT(result.event);
    events |= Feed(tilted, GRIP_STATE_HELD, 500U);
    TEST_ASSERT_BITS_LOW(ALARM_BITS | EVENT_BIT(CANE_MOTION_EVENT_SLIP), events);
}

void test_CaneMotion_BriefVerticalDownwardMotionWhileHeldDoesNotAlarm(void)
{
    const Imu_Sample_t still = { .az = 1.0f };
    const Imu_Sample_t downward = { .az = 0.35f };
    (void)Feed(still, GRIP_STATE_HELD, 750U);
    uint32_t events = Feed(downward, GRIP_STATE_HELD, 100U);
    events |= Feed(still, GRIP_STATE_HELD, 1000U);
    TEST_ASSERT_BITS_LOW(ALARM_BITS | EVENT_BIT(CANE_MOTION_EVENT_SLIP), events);
}

void test_CaneMotion_DownwardAccelerationWithRotationIsSlipEvidence(void)
{
    const Imu_Sample_t still = { .az = 1.0f };
    (void)Feed(still, GRIP_STATE_HELD, 750U);

    uint32_t events = Rotate_While_Accelerating_Down(8U);
    TEST_ASSERT_BITS_HIGH(EVENT_BIT(CANE_MOTION_EVENT_SLIP), events);
    TEST_ASSERT_BITS_LOW(ALARM_BITS, events);
}

void test_CaneMotion_ReleaseAtImpactPreservesLowGFromHeldSlip(void)
{
    const Imu_Sample_t still = { .az = 1.0f };
    const Imu_Sample_t impact = { .az = 3.0f };
    (void)Feed(still, GRIP_STATE_HELD, 750U);

    uint32_t events = Rotate_While_Accelerating_Down(6U);
    TEST_ASSERT_BITS_HIGH(EVENT_BIT(CANE_MOTION_EVENT_SLIP), events);
    TEST_ASSERT_BITS_LOW(ALARM_BITS, events);
    TEST_ASSERT_EQUAL(CANE_MOTION_EVENT_DROP, Step(impact, GRIP_STATE_RELEASED).event);
    events = Feed(impact, GRIP_STATE_RELEASED, 30U);
    events |= Feed(still, GRIP_STATE_RELEASED, 1500U);
    TEST_ASSERT_BITS_LOW(ALARM_BITS, events);
}

void test_CaneMotion_MissingSampleAndTimestampGapResetEvidence(void)
{
    const Imu_Sample_t still = { .az = 1.0f };
    const Imu_Sample_t freefall = {0};
    (void)Feed(still, GRIP_STATE_HELD, 750U);
    uint32_t events = Feed(freefall, GRIP_STATE_RELEASED, 40U);
    CaneMotion_Result_t missing = CaneMotion_Step(&s_motion, NULL, GRIP_STATE_RELEASED, s_alarm_policy);
    TEST_ASSERT_EQUAL(CANE_MOTION_EVENT_NONE, missing.event);
    TEST_ASSERT_FALSE(missing.pending);
    events |= Feed(freefall, GRIP_STATE_RELEASED, 40U);
    s_next_sample_ms += 40U;
    events |= Feed(freefall, GRIP_STATE_RELEASED, 40U);
    TEST_ASSERT_BITS_LOW(ALARM_BITS, events);
}

void test_CaneMotion_SlipOnlyKeepsVerticalTelemetryWithoutArming(void)
{
    const Imu_Sample_t still = { .ax = 0.6f, .az = -0.8f };
    const Imu_Sample_t upward = { .ax = 1.8f, .az = -2.4f };
    s_alarm_policy = CANE_MOTION_ALARM_SLIP_ONLY;
    (void)Feed(still, GRIP_STATE_HELD, 100U);

    uint32_t events = Feed(upward, GRIP_STATE_HELD, 60U);
    CaneMotion_Result_t result = Step(upward, GRIP_STATE_HELD);
    TEST_ASSERT_EQUAL_UINT32(EVENT_BIT(CANE_MOTION_EVENT_NONE), events);
    TEST_ASSERT_EQUAL(CANE_MOTION_EVENT_NONE, result.event);
    TEST_ASSERT_FALSE(result.pending);
    TEST_ASSERT_FLOAT_WITHIN(0.02f, 2.0f, result.vertical_raw_g);
    TEST_ASSERT_FLOAT_WITHIN(0.02f, 2.0f, result.vertical_g);
    TEST_ASSERT_FLOAT_WITHIN(0.02f, 0.0f, result.horizontal_g);
}

void test_CaneMotion_SlipOnlyIgnoresDownwardAccelerationAndRotation(void)
{
    const Imu_Sample_t still = { .az = 1.0f };
    s_alarm_policy = CANE_MOTION_ALARM_SLIP_ONLY;
    (void)Feed(still, GRIP_STATE_HELD, 100U);

    uint32_t events = Rotate_While_Accelerating_Down(8U);
    TEST_ASSERT_EQUAL_UINT32(EVENT_BIT(CANE_MOTION_EVENT_NONE), events);
    TEST_ASSERT_EQUAL(CANE_MOTION_IDLE, s_motion.state);
}

void test_CaneMotion_SlipOnlyReportsRecoveredSideSlipWithoutAlarm(void)
{
    const Imu_Sample_t still = { .az = 1.0f };
    const Imu_Sample_t sideways = { .ay = -0.9f, .az = 1.0f };
    s_alarm_policy = CANE_MOTION_ALARM_SLIP_ONLY;
    (void)Feed(still, GRIP_STATE_HELD, 100U);
    TEST_ASSERT_EQUAL(CANE_MOTION_IDLE, s_motion.state);

    (void)Step(sideways, GRIP_STATE_HELD); /* The median needs a second lateral sample. */
    CaneMotion_Result_t candidate = Step(sideways, GRIP_STATE_HELD);
    TEST_ASSERT_EQUAL(CANE_MOTION_SLIP_START, candidate.state);
    TEST_ASSERT_EQUAL(CANE_MOTION_EVENT_NONE, candidate.event);
    TEST_ASSERT_TRUE(candidate.pending);

    uint32_t events = Feed(sideways, GRIP_STATE_HELD, 20U);
    TEST_ASSERT_EQUAL_UINT32(EVENT_BIT(CANE_MOTION_EVENT_NONE), events);
    CaneMotion_Result_t confirmed = Step(sideways, GRIP_STATE_HELD);
    TEST_ASSERT_EQUAL(CANE_MOTION_SLIP_TRACK, confirmed.state);
    TEST_ASSERT_EQUAL(CANE_MOTION_EVENT_SLIP, confirmed.event);
    TEST_ASSERT_TRUE(confirmed.pending);

    events = Feed(still, GRIP_STATE_HELD, 1600U);
    TEST_ASSERT_BITS_LOW(ALARM_BITS, events);
    TEST_ASSERT_EQUAL(CANE_MOTION_IDLE, s_motion.state);
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_CaneMotion_TiltedLiftAndBriefLowGDoNotAlarm);
    RUN_TEST(test_CaneMotion_SustainedReleasedDropStillAlarmsAfterLift);
    RUN_TEST(test_CaneMotion_ShortDropAfterLiftCanConfirmWithTilt);
    RUN_TEST(test_CaneMotion_ReleasedFreefallDoesNotRequireTiltOrImpact);
    RUN_TEST(test_CaneMotion_UpwardImpactCanConfirmReleasedDrop);
    RUN_TEST(test_CaneMotion_ShortUpwardSpikesCannotMaskShortReleasedDrop);
    RUN_TEST(test_CaneMotion_QuietOppositeSampleCannotSeedWorldUp);
    RUN_TEST(test_CaneMotion_AlternatingVerticalOutliersDoNotAccumulateEvidence);
    RUN_TEST(test_CaneMotion_ObservesSlipInOppositeAndDiagonalDirections);
    RUN_TEST(test_CaneMotion_SideSlipSurvivesBriefLowGAndSustainedTiltConfirmsFall);
    RUN_TEST(test_CaneMotion_GyroTracksPureRotationWithoutReportingSlip);
    RUN_TEST(test_CaneMotion_BriefVerticalDownwardMotionWhileHeldDoesNotAlarm);
    RUN_TEST(test_CaneMotion_DownwardAccelerationWithRotationIsSlipEvidence);
    RUN_TEST(test_CaneMotion_ReleaseAtImpactPreservesLowGFromHeldSlip);
    RUN_TEST(test_CaneMotion_MissingSampleAndTimestampGapResetEvidence);
    RUN_TEST(test_CaneMotion_SlipOnlyKeepsVerticalTelemetryWithoutArming);
    RUN_TEST(test_CaneMotion_SlipOnlyIgnoresDownwardAccelerationAndRotation);
    RUN_TEST(test_CaneMotion_SlipOnlyReportsRecoveredSideSlipWithoutAlarm);
    return UNITY_END();
}
