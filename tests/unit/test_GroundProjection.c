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

/* Real cadence: one ToF sample every 100 ms. Calibrate on 1000 mm, then refill the median window. */
#define TOF_PERIOD_MS (100U)

static uint32_t Calibrate_And_Settle(Gp_Detector_t * const p_d, uint16_t level_mm)
{
    uint32_t t = 0U;
    GroundProjection_Init(p_d, t);
    GroundProjection_SetArmed(p_d, true, t);
    GroundProjection_StartCalibration(p_d, t);

    for (uint32_t i = 0U; i < GP_CAL_SAMPLES; i++)
    {
        GroundProjection_Feed(p_d, GP_RANGE_VALID, level_mm, t);
        t += TOF_PERIOD_MS;
    }
    for (uint32_t i = 0U; i < 5U; i++)
    {
        GroundProjection_Feed(p_d, GP_RANGE_VALID, level_mm, t);
        t += TOF_PERIOD_MS;
    }
    return t;
}

/* Feeds range_mm every 100 ms until the detector reaches target; returns the time it did, or 0 if never */
static uint32_t Feed_Until(Gp_Detector_t * const p_d, uint16_t range_mm, uint32_t t, Gp_State_t target)
{
    for (uint32_t i = 0U; i < 20U; i++)
    {
        GroundProjection_Feed(p_d, GP_RANGE_VALID, range_mm, t);
        if (p_d->state == target)
        {
            return t;
        }
        t += TOF_PERIOD_MS;
    }
    return 0U;
}

/* Gap 9 / A10: after a reboot there is no baseline, so no step warning can fire and the user is prompted */
void test_GroundProjection_RebootWithoutCalibration_PromptsAndNeverWarns(void)
{
    Gp_Detector_t d;
    GroundProjection_Init(&d, 0U);
    GroundProjection_SetArmed(&d, true, 0U);

    uint32_t t = 0U;
    for (uint32_t i = 0U; i < 10U; i++)
    {
        GroundProjection_Feed(&d, GP_RANGE_VALID, 1000U, t);
        t += TOF_PERIOD_MS;
    }
    for (uint32_t i = 0U; i < 10U; i++)
    {
        GroundProjection_Feed(&d, GP_RANGE_VALID, 1400U, t);
        TEST_ASSERT_NOT_EQUAL(GP_STATE_STEP_DOWN, d.state);
        TEST_ASSERT_NOT_EQUAL(GP_STATE_STEP_UP, d.state);
        t += TOF_PERIOD_MS;
    }

    TEST_ASSERT_FALSE(d.is_calibrated);
    TEST_ASSERT_EQUAL(GP_STATE_UNKNOWN, d.state);
    TEST_ASSERT_EQUAL(ALERT_CALIBRATE_PROMPT, GroundProjection_GetHapticRequest(&d));
}

/* A cancelled recalibration (e.g. grip released) must keep the previous baseline, not wipe it */
void test_GroundProjection_CancelledRecalibration_KeepsOldBaseline(void)
{
    Gp_Detector_t d;
    uint32_t t = Calibrate_And_Settle(&d, 1000U);

    GroundProjection_StartCalibration(&d, t);
    TEST_ASSERT_EQUAL(GP_STATE_CALIBRATING, d.state);
    TEST_ASSERT_EQUAL(ALERT_NONE, GroundProjection_GetHapticRequest(&d));
    for (uint32_t i = 0U; i < 10U; i++)
    {
        t += TOF_PERIOD_MS;
        GroundProjection_Feed(&d, GP_RANGE_VALID, 1050U, t);
    }

    GroundProjection_CancelCalibration(&d, t);
    TEST_ASSERT_EQUAL(GP_STATE_UNKNOWN, d.state);
    TEST_ASSERT_TRUE(d.is_calibrated);
    TEST_ASSERT_EQUAL(1000U, d.baseline_mm);

    /* Warnings still work on the old baseline */
    for (uint32_t i = 0U; i < 5U; i++)
    {
        t += TOF_PERIOD_MS;
        GroundProjection_Feed(&d, GP_RANGE_VALID, 1000U, t);
    }
    TEST_ASSERT_NOT_EQUAL(0U, Feed_Until(&d, 1180U, t + TOF_PERIOD_MS, GP_STATE_STEP_DOWN));
}

/* A completed recalibration replaces the baseline */
void test_GroundProjection_CompletedRecalibration_ReplacesBaseline(void)
{
    Gp_Detector_t d;
    uint32_t t = Calibrate_And_Settle(&d, 1000U);

    GroundProjection_StartCalibration(&d, t);
    for (uint32_t i = 0U; i < GP_CAL_SAMPLES; i++)
    {
        t += TOF_PERIOD_MS;
        GroundProjection_Feed(&d, GP_RANGE_VALID, 1100U, t);
    }

    TEST_ASSERT_EQUAL(GP_STATE_LEVEL, d.state);
    TEST_ASSERT_EQUAL(1100U, d.baseline_mm);
}

/* Gap 9: a ToF that stops giving valid readings is reported within 500 ms and gets the caution cue */
void test_GroundProjection_TofFault_RequestsCautionWithin500ms(void)
{
    Gp_Detector_t d;
    uint32_t t = Calibrate_And_Settle(&d, 1000U);
    uint32_t last_valid_ms = t - TOF_PERIOD_MS;

    GroundProjection_Feed(&d, GP_RANGE_INVALID, 0U, t);
    TEST_ASSERT_EQUAL(GP_STATE_UNKNOWN, d.state);
    TEST_ASSERT_EQUAL(ALERT_NONE, GroundProjection_GetHapticRequest(&d));

    while (d.state != GP_STATE_SENSOR_FAULT)
    {
        t += TOF_PERIOD_MS;
        GroundProjection_Feed(&d, GP_RANGE_INVALID, 0U, t);
        TEST_ASSERT_TRUE((t - last_valid_ms) <= GP_STALE_TIMEOUT_MS);
    }
    TEST_ASSERT_EQUAL(ALERT_SENSOR_FAULT, GroundProjection_GetHapticRequest(&d));
}

/*
 * Gap 9 latency characterisation at 100 ms sampling, measured from the first
 * changed reading. Normal drops need the median (3rd sample) plus 3 votes:
 * 400 ms, which still misses NFR10. Drops > 250 mm take the fast path: 200 ms.
 */
void test_GroundProjection_StepDownLatency_400ms(void)
{
    Gp_Detector_t d;
    uint32_t t = Calibrate_And_Settle(&d, 1000U);

    uint32_t confirmed_ms = Feed_Until(&d, 1180U, t, GP_STATE_STEP_DOWN);
    TEST_ASSERT_TRUE(d.change_pending);
    TEST_ASSERT_EQUAL_UINT32(t, d.change_start_ms);
    TEST_ASSERT_EQUAL_UINT32(400U, confirmed_ms - d.change_start_ms);
    TEST_ASSERT_EQUAL(ALERT_CLIFF_WARNING, GroundProjection_GetHapticRequest(&d));
}

void test_GroundProjection_SevereDropLatency_200ms(void)
{
    Gp_Detector_t d;
    uint32_t t = Calibrate_And_Settle(&d, 1000U);

    uint32_t confirmed_ms = Feed_Until(&d, 1350U, t, GP_STATE_STEP_DOWN);
    TEST_ASSERT_EQUAL_UINT32(200U, confirmed_ms - d.change_start_ms);
}

void test_GroundProjection_StepUpLatency_400ms(void)
{
    Gp_Detector_t d;
    uint32_t t = Calibrate_And_Settle(&d, 1000U);

    uint32_t confirmed_ms = Feed_Until(&d, 900U, t, GP_STATE_STEP_UP);
    TEST_ASSERT_EQUAL_UINT32(400U, confirmed_ms - d.change_start_ms);
    TEST_ASSERT_EQUAL(ALERT_CLIFF_STEP_UP, GroundProjection_GetHapticRequest(&d));
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_GroundProjection_CalibrationAndLevel);
    RUN_TEST(test_GroundProjection_Cliff_StepDown_Detection);
    RUN_TEST(test_GroundProjection_SevereCliff_FastPathBypass);
    RUN_TEST(test_GroundProjection_NoGround_SustainedVoting);
    RUN_TEST(test_GroundProjection_RebootWithoutCalibration_PromptsAndNeverWarns);
    RUN_TEST(test_GroundProjection_CancelledRecalibration_KeepsOldBaseline);
    RUN_TEST(test_GroundProjection_CompletedRecalibration_ReplacesBaseline);
    RUN_TEST(test_GroundProjection_TofFault_RequestsCautionWithin500ms);
    RUN_TEST(test_GroundProjection_StepDownLatency_400ms);
    RUN_TEST(test_GroundProjection_SevereDropLatency_200ms);
    RUN_TEST(test_GroundProjection_StepUpLatency_400ms);
    return UNITY_END();
}
