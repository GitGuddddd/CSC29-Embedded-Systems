#include "unity.h"
#include "app/GroundProjection/GroundProjection.h"
#include "app/GroundProjection/GroundProjection_Cfg.h"

void setUp(void)
{
}

void tearDown(void)
{
}

void test_GroundProjection_CalibrationAndLevel(void)
{
    Gp_Detector_t d;
    GroundProjection_Init(&d, 0U);
    GroundProjection_SetArmed(&d, true, 10U);
    GroundProjection_StartCalibration(&d, 20U);

    TEST_ASSERT_EQUAL(GP_STATE_CALIBRATING, d.state);

    for (uint32_t i = 0U; i < GP_CAL_SAMPLES; i++)
    {
        GroundProjection_Feed(&d, GP_RANGE_VALID, 1000U, 20U + i * 10U);
    }

    TEST_ASSERT_EQUAL(GP_STATE_LEVEL, d.state);
    TEST_ASSERT_EQUAL(1000U, d.baseline_mm);
}

void test_GroundProjection_Cliff_StepDown_Detection(void)
{
    Gp_Detector_t d;
    GroundProjection_Init(&d, 0U);
    GroundProjection_SetArmed(&d, true, 0U);
    GroundProjection_StartCalibration(&d, 0U);

    for (uint32_t i = 0U; i < GP_CAL_SAMPLES; i++)
    {
        GroundProjection_Feed(&d, GP_RANGE_VALID, 1000U, i * 10U);
    }

    /* Simulate a gradual drop: 1000mm -> 1180mm (> +150mm threshold) */
    uint32_t t = GP_CAL_SAMPLES * 10U;
    for (uint32_t i = 0U; i < 8U; i++)
    {
        t += 10U;
        GroundProjection_Feed(&d, GP_RANGE_VALID, 1180U, t);
    }

    TEST_ASSERT_EQUAL(GP_STATE_STEP_DOWN, d.state);
}

void test_GroundProjection_SevereCliff_FastPathBypass(void)
{
    Gp_Detector_t d;
    GroundProjection_Init(&d, 0U);
    GroundProjection_SetArmed(&d, true, 0U);
    GroundProjection_StartCalibration(&d, 0U);

    for (uint32_t i = 0U; i < GP_CAL_SAMPLES; i++)
    {
        GroundProjection_Feed(&d, GP_RANGE_VALID, 1000U, i * 10U);
    }

    /* Severe cliff drop: 1000mm -> 1350mm (> +250mm fast-path threshold) */
    /* Must trigger STEP_DOWN on the very first sample without waiting for 3 votes */
    uint32_t t = GP_CAL_SAMPLES * 10U + 10U;
    GroundProjection_Feed(&d, GP_RANGE_VALID, 1350U, t);
    GroundProjection_Feed(&d, GP_RANGE_VALID, 1350U, t + 10U);
    GroundProjection_Feed(&d, GP_RANGE_VALID, 1350U, t + 20U);
    GroundProjection_Feed(&d, GP_RANGE_VALID, 1350U, t + 30U);
    GroundProjection_Feed(&d, GP_RANGE_VALID, 1350U, t + 40U);

    TEST_ASSERT_EQUAL(GP_STATE_STEP_DOWN, d.state);
}

void test_GroundProjection_NoGround_SustainedVoting(void)
{
    Gp_Detector_t d;
    GroundProjection_Init(&d, 0U);
    GroundProjection_SetArmed(&d, true, 0U);
    GroundProjection_StartCalibration(&d, 0U);

    for (uint32_t i = 0U; i < GP_CAL_SAMPLES; i++)
    {
        GroundProjection_Feed(&d, GP_RANGE_VALID, 1000U, i * 10U);
    }
    TEST_ASSERT_EQUAL(GP_STATE_LEVEL, d.state);

    /* Feed sustained NO_RETURN samples to verify NO_GROUND transition */
    uint32_t t = GP_CAL_SAMPLES * 10U;
    GroundProjection_Feed(&d, GP_RANGE_NO_RETURN, 0U, t + 10U);
    TEST_ASSERT_EQUAL(GP_STATE_UNKNOWN, d.state);

    GroundProjection_Feed(&d, GP_RANGE_NO_RETURN, 0U, t + 20U);
    TEST_ASSERT_EQUAL(GP_STATE_UNKNOWN, d.state);

    GroundProjection_Feed(&d, GP_RANGE_NO_RETURN, 0U, t + 30U);
    TEST_ASSERT_EQUAL(GP_STATE_NO_GROUND, d.state);
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_GroundProjection_CalibrationAndLevel);
    RUN_TEST(test_GroundProjection_Cliff_StepDown_Detection);
    RUN_TEST(test_GroundProjection_SevereCliff_FastPathBypass);
    RUN_TEST(test_GroundProjection_NoGround_SustainedVoting);
    return UNITY_END();
}
