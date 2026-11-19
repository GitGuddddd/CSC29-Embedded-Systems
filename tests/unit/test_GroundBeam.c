#include "unity.h"
#include "app/GroundProjection/GroundBeam.h"
#include "app/GroundProjection/GroundBeam_Cfg.h"

#include <math.h>

static GroundBeam_t s_beam;

void setUp(void)
{
    GroundBeam_Init(&s_beam);
}

void tearDown(void)
{
}

static Imu_Sample_t Sample_AtAngle(float degrees)
{
    float radians = degrees * 0.01745329252f;
    return (Imu_Sample_t){ .ay = sinf(radians), .az = cosf(radians) };
}

void test_GroundBeam_RequiresAlignmentAndHandlesAnyMountingAxis(void)
{
    Imu_Sample_t reference = { .ax = 0.6f, .az = -0.8f };
    float cosine;
    TEST_ASSERT_EQUAL(GROUND_BEAM_UNALIGNED, GroundBeam_Update(&s_beam, &reference, &cosine));
    TEST_ASSERT_TRUE(isnan(cosine));
    TEST_ASSERT_TRUE(GroundBeam_Align(&s_beam, &reference));

    TEST_ASSERT_EQUAL(GROUND_BEAM_AVAILABLE, GroundBeam_Update(&s_beam, &reference, &cosine));
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 1.0f, cosine);
    TEST_ASSERT_EQUAL_UINT16(600U, GroundBeam_Project(600U, cosine));

    /* Tilt into a second plane while retaining the off-axis mounting. */
    Imu_Sample_t inclination = Sample_AtAngle((float)GROUND_BEAM_CUTOFF_DEG / 2.0f);
    Imu_Sample_t tilted = {
        .ax = reference.ax * inclination.az,
        .ay = inclination.ay,
        .az = reference.az * inclination.az
    };
    TEST_ASSERT_TRUE(GroundBeam_Align(&s_beam, &reference));
    TEST_ASSERT_EQUAL(GROUND_BEAM_AVAILABLE, GroundBeam_Update(&s_beam, &tilted, &cosine));
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, inclination.az, cosine);
    TEST_ASSERT_EQUAL_UINT16(520U, GroundBeam_Project(600U, 0.8660254f));
    TEST_ASSERT_EQUAL_UINT16(26U, GroundBeam_Project(30U, 0.8660254f));

    TEST_ASSERT_TRUE(GroundBeam_Align(&s_beam, &reference));
    Imu_Sample_t up = { .ax = -reference.ax, .az = -reference.az };
    TEST_ASSERT_EQUAL(GROUND_BEAM_OUT_OF_ANGLE, GroundBeam_Update(&s_beam, &up, &cosine));
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, -1.0f, cosine);
}

static GroundBeam_Status_t Rotate_Beam(float start_deg, float end_deg, uint32_t * const p_now_ms)
{
    GroundBeam_Status_t status = GROUND_BEAM_IMU_UNAVAILABLE;
    float delta_deg = end_deg - start_deg;
    for (uint32_t sample = 1U; sample <= 10U; ++sample)
    {
        Imu_Sample_t imu = Sample_AtAngle(start_deg + ((float)sample * 0.1f * delta_deg));
        imu.gx = delta_deg * 10.0f;
        *p_now_ms += 10U;
        imu.timestamp_ms = *p_now_ms;
        status = GroundBeam_Update(&s_beam, &imu, NULL);
    }
    return status;
}

void test_GroundBeam_UsesDifferentEntryAndResumeThresholds(void)
{
    Imu_Sample_t down = { .az = 1.0f };
    float in_band_deg = ((float)GROUND_BEAM_RESUME_DEG + (float)GROUND_BEAM_CUTOFF_DEG) / 2.0f;
    float too_steep_deg = (float)GROUND_BEAM_CUTOFF_DEG + 1.0f;
    Imu_Sample_t in_band = Sample_AtAngle(in_band_deg);
    uint32_t now_ms = 0U;
    TEST_ASSERT_TRUE(GroundBeam_Align(&s_beam, &down));
    TEST_ASSERT_EQUAL(GROUND_BEAM_OUT_OF_ANGLE, GroundBeam_Update(&s_beam, &in_band, NULL));
    TEST_ASSERT_EQUAL(GROUND_BEAM_AVAILABLE, Rotate_Beam(in_band_deg, 0.0f, &now_ms));
    TEST_ASSERT_EQUAL(GROUND_BEAM_AVAILABLE, Rotate_Beam(0.0f, in_band_deg, &now_ms));
    TEST_ASSERT_EQUAL(GROUND_BEAM_OUT_OF_ANGLE, Rotate_Beam(in_band_deg, too_steep_deg, &now_ms));
    TEST_ASSERT_EQUAL(GROUND_BEAM_OUT_OF_ANGLE, Rotate_Beam(too_steep_deg, in_band_deg, &now_ms));
    TEST_ASSERT_EQUAL(GROUND_BEAM_AVAILABLE, Rotate_Beam(in_band_deg, 0.0f, &now_ms));
}

void test_GroundBeam_RejectsAbsentDuplicateStaleAndInvalidSamples(void)
{
    Imu_Sample_t down = { .az = 1.0f, .timestamp_ms = 100U };
    Imu_Sample_t invalid = { .az = NAN };
    float cosine;
    TEST_ASSERT_TRUE(GroundBeam_Align(&s_beam, &down));
    TEST_ASSERT_EQUAL(GROUND_BEAM_AVAILABLE, GroundBeam_Update(&s_beam, &down, NULL));
    TEST_ASSERT_FALSE(GroundBeam_Align(&s_beam, &invalid));
    TEST_ASSERT_EQUAL(GROUND_BEAM_IMU_UNAVAILABLE, GroundBeam_Update(&s_beam, &down, &cosine));
    TEST_ASSERT_TRUE(isnan(cosine));
    TEST_ASSERT_EQUAL(GROUND_BEAM_IMU_UNAVAILABLE, GroundBeam_Update(&s_beam, NULL, &cosine));
    TEST_ASSERT_TRUE(isnan(cosine));

    /* A brief missed read leaves enough timing information to bridge with the gyro. */
    Imu_Sample_t moving = { .az = 1.5f, .gz = 100.0f, .timestamp_ms = 120U };
    TEST_ASSERT_EQUAL(GROUND_BEAM_AVAILABLE, GroundBeam_Update(&s_beam, &moving, &cosine));
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 1.0f, cosine);
    moving.timestamp_ms += GROUND_BEAM_SAMPLE_GAP_MS + 1U;
    TEST_ASSERT_EQUAL(GROUND_BEAM_IMU_UNAVAILABLE, GroundBeam_Update(&s_beam, &moving, NULL));
    down.timestamp_ms = moving.timestamp_ms + 10U;
    TEST_ASSERT_EQUAL(GROUND_BEAM_AVAILABLE, GroundBeam_Update(&s_beam, &down, NULL));

    invalid.timestamp_ms = down.timestamp_ms + 10U;
    TEST_ASSERT_EQUAL(GROUND_BEAM_IMU_UNAVAILABLE, GroundBeam_Update(&s_beam, &invalid, &cosine));
    TEST_ASSERT_TRUE(isnan(cosine));
    down.timestamp_ms = invalid.timestamp_ms + 10U;
    TEST_ASSERT_EQUAL(GROUND_BEAM_AVAILABLE, GroundBeam_Update(&s_beam, &down, NULL));

    Imu_Sample_t saturated = {
        .az = 1.0f, .gx = GROUND_BEAM_GYRO_SATURATION_DPS, .timestamp_ms = down.timestamp_ms + 10U
    };
    TEST_ASSERT_EQUAL(GROUND_BEAM_IMU_UNAVAILABLE, GroundBeam_Update(&s_beam, &saturated, NULL));
}

void test_GroundBeam_TracksGyroDuringConflictingAccelerationAndTimestampWrap(void)
{
    Imu_Sample_t down = { .az = 1.0f, .timestamp_ms = UINT32_MAX - 25U };
    TEST_ASSERT_TRUE(GroundBeam_Align(&s_beam, &down));
    TEST_ASSERT_EQUAL(GROUND_BEAM_AVAILABLE, GroundBeam_Update(&s_beam, &down, NULL));

    /* A 1 g acceleration with the wrong direction fails the plausibility check. */
    Imu_Sample_t moving = Sample_AtAngle(-60.0f);
    moving.gx = 100.0f;
    moving.timestamp_ms = down.timestamp_ms;
    GroundBeam_Status_t status = GROUND_BEAM_IMU_UNAVAILABLE;
    float cosine;
    for (uint32_t i = 0U; i < 40U; ++i)
    {
        moving.timestamp_ms += 10U;
        status = GroundBeam_Update(&s_beam, &moving, &cosine);
    }
    TEST_ASSERT_EQUAL(GROUND_BEAM_OUT_OF_ANGLE, status);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, Sample_AtAngle(40.0f).az, cosine);
    /* Rotating +X moves the fixed world's apparent acceleration toward +Y. */
    TEST_ASSERT_TRUE(s_beam.gravity[1] > 0.0f);

    moving.gx = -100.0f;
    for (uint32_t i = 0U; i < 20U; ++i)
    {
        moving.timestamp_ms += 10U;
        status = GroundBeam_Update(&s_beam, &moving, &cosine);
    }
    TEST_ASSERT_EQUAL(GROUND_BEAM_AVAILABLE, status);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, Sample_AtAngle(20.0f).az, cosine);
}

void test_GroundBeam_UsesConsistentAccelerationAtSustainedRotation(void)
{
    Imu_Sample_t steady = { .ax = 0.6f, .az = -0.8f };
    TEST_ASSERT_TRUE(GroundBeam_Align(&s_beam, &steady));
    TEST_ASSERT_EQUAL(GROUND_BEAM_AVAILABLE, GroundBeam_Update(&s_beam, &steady, NULL));
    /* Rotation around the beam with a steady gravity direction. */
    steady.gx = 350.0f * steady.ax;
    steady.gz = 350.0f * steady.az;
    for (uint32_t i = 0U; i < 100U; ++i)
    {
        float cosine;
        steady.timestamp_ms += 10U;
        TEST_ASSERT_EQUAL(GROUND_BEAM_AVAILABLE, GroundBeam_Update(&s_beam, &steady, &cosine));
        TEST_ASSERT_FLOAT_WITHIN(0.0001f, 1.0f, cosine);
    }
}

void test_GroundBeam_RejectsFreeFallAndBoundsGyroOnlyAge(void)
{
    Imu_Sample_t down = { .az = 1.0f };
    TEST_ASSERT_TRUE(GroundBeam_Align(&s_beam, &down));
    TEST_ASSERT_EQUAL(GROUND_BEAM_AVAILABLE, GroundBeam_Update(&s_beam, &down, NULL));
    Imu_Sample_t moving = { .az = 1.5f, .gz = 100.0f };
    for (uint32_t elapsed_ms = 10U; elapsed_ms <= GROUND_BEAM_GYRO_ONLY_LIMIT_MS; elapsed_ms += 10U)
    {
        moving.timestamp_ms = elapsed_ms;
        TEST_ASSERT_EQUAL(GROUND_BEAM_AVAILABLE, GroundBeam_Update(&s_beam, &moving, NULL));
    }
    moving.timestamp_ms += 10U;
    TEST_ASSERT_EQUAL(GROUND_BEAM_IMU_UNAVAILABLE, GroundBeam_Update(&s_beam, &moving, NULL));
    down.timestamp_ms = moving.timestamp_ms + 10U;
    TEST_ASSERT_EQUAL(GROUND_BEAM_AVAILABLE, GroundBeam_Update(&s_beam, &down, NULL));

    Imu_Sample_t free_fall = { .timestamp_ms = down.timestamp_ms + 10U };
    TEST_ASSERT_EQUAL(GROUND_BEAM_IMU_UNAVAILABLE, GroundBeam_Update(&s_beam, &free_fall, NULL));
    moving.timestamp_ms = free_fall.timestamp_ms + 10U;
    TEST_ASSERT_EQUAL(GROUND_BEAM_IMU_UNAVAILABLE,
                      GroundBeam_Update(&s_beam, &moving, NULL));
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_GroundBeam_RequiresAlignmentAndHandlesAnyMountingAxis);
    RUN_TEST(test_GroundBeam_UsesDifferentEntryAndResumeThresholds);
    RUN_TEST(test_GroundBeam_RejectsAbsentDuplicateStaleAndInvalidSamples);
    RUN_TEST(test_GroundBeam_TracksGyroDuringConflictingAccelerationAndTimestampWrap);
    RUN_TEST(test_GroundBeam_UsesConsistentAccelerationAtSustainedRotation);
    RUN_TEST(test_GroundBeam_RejectsFreeFallAndBoundsGyroOnlyAge);
    return UNITY_END();
}
