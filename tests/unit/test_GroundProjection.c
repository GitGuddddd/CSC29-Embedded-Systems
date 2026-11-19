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

void test_GroundProjection_TiltCalibration_AveragesFinalRangeWindow(void)
{
    Gp_Detector_t d;
    uint32_t now_ms = 0U;
    GroundProjection_Init(&d, now_ms);
    GroundProjection_SetArmed(&d, true, now_ms);
    GroundProjection_StartCalibration(&d, now_ms);

    const Imu_Sample_t old_pose = { .az = 1.0f };
    for (uint32_t i = 0U; i < 5U; ++i)
    {
        GroundProjection_FeedWithImu(&d, GP_RANGE_VALID, 800U, now_ms, &old_pose);
        now_ms += TOF_PERIOD_MS;
    }

    /* The distance change starts a fresh window, with a different cane pose. */
    Imu_Sample_t pose = { .ax = 0.4f, .az = -0.9f };
    GroundProjection_FeedWithImu(&d, GP_RANGE_VALID, 1000U, now_ms, &pose);
    now_ms += TOF_PERIOD_MS;
    GroundProjection_FeedWithImu(&d, GP_RANGE_INVALID, 0U, now_ms, &old_pose);
    now_ms += TOF_PERIOD_MS;

    pose.ax = 0.45f;
    for (uint32_t i = 1U; i < GP_CAL_SAMPLES; ++i)
    {
        GroundProjection_FeedWithImu(&d, GP_RANGE_VALID, 1000U, now_ms, &pose);
        now_ms += TOF_PERIOD_MS;
    }

    TEST_ASSERT_EQUAL(GP_CAL_EVENT_DONE, GroundProjection_TakeCalEvent(&d));
    TEST_ASSERT_EQUAL_UINT16(1000U, d.baseline_mm);
    TEST_ASSERT_TRUE(d.is_tilt_calibrated);
    TEST_ASSERT_TRUE(d.cal_tilt_saved);
    float expected_x = (0.4f + 0.45f * (GP_CAL_SAMPLES - 1U)) / GP_CAL_SAMPLES;
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, expected_x, d.tilt_reference[0]);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.0f, d.tilt_reference[1]);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, -0.9f, d.tilt_reference[2]);
}

void test_GroundProjection_TiltCalibration_KeepsReferenceWithMissingSamplesOrCancellation(void)
{
    Gp_Detector_t d;
    uint32_t now_ms = 0U;
    GroundProjection_Init(&d, now_ms);
    GroundProjection_SetArmed(&d, true, now_ms);
    GroundProjection_StartCalibration(&d, now_ms);

    const Imu_Sample_t initial_pose = { .ay = 0.6f, .az = -0.8f };
    for (uint32_t i = 0U; i < GP_CAL_SAMPLES; ++i)
    {
        GroundProjection_FeedWithImu(&d, GP_RANGE_VALID, 1000U, now_ms, &initial_pose);
        now_ms += TOF_PERIOD_MS;
    }
    TEST_ASSERT_TRUE(d.cal_tilt_saved);

    GroundProjection_StartCalibration(&d, now_ms);
    const Imu_Sample_t second_pose = { .ax = 1.0f };
    for (uint32_t i = 0U; i < GP_CAL_SAMPLES; ++i)
    {
        const Imu_Sample_t * const sample = (i < GP_CAL_TILT_MIN_SAMPLES - 1U) ? &second_pose : NULL;
        GroundProjection_FeedWithImu(&d, GP_RANGE_VALID, 1100U, now_ms, sample);
        now_ms += TOF_PERIOD_MS;
    }
    TEST_ASSERT_EQUAL(GP_CAL_EVENT_DONE, GroundProjection_TakeCalEvent(&d));
    TEST_ASSERT_EQUAL_UINT16(1100U, d.baseline_mm);
    TEST_ASSERT_FALSE(d.cal_tilt_saved);

    GroundProjection_StartCalibration(&d, now_ms);
    GroundProjection_FeedWithImu(&d, GP_RANGE_VALID, 1200U, now_ms, &second_pose);
    GroundProjection_CancelCalibration(&d, now_ms + TOF_PERIOD_MS);
    TEST_ASSERT_EQUAL(GP_CAL_EVENT_CANCELLED, GroundProjection_TakeCalEvent(&d));
    TEST_ASSERT_EQUAL_UINT16(1100U, d.baseline_mm);
    TEST_ASSERT_FALSE(d.cal_tilt_saved);
    TEST_ASSERT_TRUE(d.is_tilt_calibrated);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.0f, d.tilt_reference[0]);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.6f, d.tilt_reference[1]);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, -0.8f, d.tilt_reference[2]);
}

void test_GroundProjection_TiltCalibration_RejectsAnInconsistentPose(void)
{
    Gp_Detector_t d;
    GroundProjection_Init(&d, 0U);
    GroundProjection_SetArmed(&d, true, 0U);
    GroundProjection_StartCalibration(&d, 0U);

    Imu_Sample_t pose = { .az = 1.0f };
    for (uint32_t i = 0U; i < GP_CAL_SAMPLES; ++i)
    {
        GroundProjection_FeedWithImu(&d, GP_RANGE_VALID, 1000U, i * TOF_PERIOD_MS, &pose);
        pose.az = -pose.az;
    }

    TEST_ASSERT_EQUAL(GP_CAL_EVENT_DONE, GroundProjection_TakeCalEvent(&d));
    TEST_ASSERT_EQUAL_UINT16(1000U, d.baseline_mm);
    TEST_ASSERT_FALSE(d.is_tilt_calibrated);
    TEST_ASSERT_FALSE(d.cal_tilt_saved);
}

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

void test_GroundProjection_PoseGate_UsesFreshSamplesOnResume(void)
{
    Gp_Detector_t d;
    uint32_t now_ms = Calibrate_And_Settle(&d, 1000U);
    GroundProjection_FeedProjected(&d, GP_RANGE_VALID, 1350U, now_ms, NULL);
    TEST_ASSERT_TRUE(d.change_pending);
    TEST_ASSERT_EQUAL_UINT8(5U, d.history_count);

    /* Rejected poses come from a responsive sensor, even across the stale timeout. */
    for (uint32_t elapsed_ms = 0U; elapsed_ms <= GP_STALE_TIMEOUT_MS; elapsed_ms += TOF_PERIOD_MS)
    {
        now_ms += TOF_PERIOD_MS;
        GroundProjection_FeedProjected(&d, GP_RANGE_POSE_UNAVAILABLE, 0U, now_ms, NULL);
        TEST_ASSERT_EQUAL(GP_STATE_POSE_UNAVAILABLE, d.state);
        TEST_ASSERT_EQUAL_UINT32(now_ms, d.last_sample_ms);
        TEST_ASSERT_EQUAL(ALERT_NONE, GroundProjection_GetHapticRequest(&d));
    }
    TEST_ASSERT_FALSE(d.change_pending);
    TEST_ASSERT_EQUAL_UINT8(0U, d.history_count);
    TEST_ASSERT_EQUAL_UINT16(1000U, d.baseline_mm);

    uint32_t first_fresh_ms = now_ms + TOF_PERIOD_MS;
    for (uint32_t i = 0U; i < 5U; ++i)
    {
        now_ms += TOF_PERIOD_MS;
        GroundProjection_FeedProjected(&d, GP_RANGE_VALID, 1350U, now_ms, NULL);
        TEST_ASSERT_EQUAL(i < 4U ? GP_STATE_UNKNOWN : GP_STATE_STEP_DOWN, d.state);
    }
    TEST_ASSERT_EQUAL_UINT32(first_fresh_ms, d.change_start_ms);
}

void test_GroundProjection_PoseGate_ResetsNoReturnVotes(void)
{
    Gp_Detector_t d;
    uint32_t now_ms = Calibrate_And_Settle(&d, 1000U);
    GroundProjection_FeedProjected(&d, GP_RANGE_NO_RETURN, 0U, now_ms, NULL);
    TEST_ASSERT_EQUAL_UINT8(1U, d.candidate_count);
    now_ms += TOF_PERIOD_MS;
    GroundProjection_FeedProjected(&d, GP_RANGE_POSE_UNAVAILABLE, 0U, now_ms, NULL);
    TEST_ASSERT_EQUAL_UINT8(0U, d.candidate_count);

    for (uint32_t i = 1U; i <= GP_CONFIRM_SAMPLES; ++i)
    {
        now_ms += TOF_PERIOD_MS;
        GroundProjection_FeedProjected(&d, GP_RANGE_NO_RETURN, 0U, now_ms, NULL);
        TEST_ASSERT_EQUAL(i < GP_CONFIRM_SAMPLES ? GP_STATE_UNKNOWN : GP_STATE_NO_GROUND, d.state);
    }
}

void test_GroundProjection_PoseGate_DiscardsPartialCalibrationWindow(void)
{
    Gp_Detector_t d;
    uint32_t now_ms = Calibrate_And_Settle(&d, 1000U);
    GroundProjection_StartCalibration(&d, now_ms);
    const Imu_Sample_t pose = { .az = 1.0f };
    GroundProjection_FeedProjected(&d, GP_RANGE_VALID, 30U, now_ms, &pose);
    TEST_ASSERT_EQUAL_UINT8(1U, d.cal_count);
    TEST_ASSERT_EQUAL_UINT8(1U, d.cal_tilt_count);

    now_ms += TOF_PERIOD_MS;
    GroundProjection_FeedProjected(&d, GP_RANGE_POSE_UNAVAILABLE, 0U, now_ms, NULL);
    TEST_ASSERT_EQUAL(GP_STATE_POSE_UNAVAILABLE, d.state);
    TEST_ASSERT_TRUE(d.is_calibrating);
    TEST_ASSERT_EQUAL_UINT8(0U, d.cal_count);
    TEST_ASSERT_EQUAL_UINT8(0U, d.cal_tilt_count);
    TEST_ASSERT_EQUAL_UINT16(1000U, d.baseline_mm);

    /* Only the raw ToF distance is bounded by GP_MIN_RANGE_MM before projection. */
    for (uint32_t i = 1U; i <= GP_CAL_SAMPLES; ++i)
    {
        now_ms += TOF_PERIOD_MS;
        GroundProjection_FeedProjected(&d, GP_RANGE_VALID, GP_MIN_RANGE_MM - 1U, now_ms, &pose);
        TEST_ASSERT_EQUAL(i < GP_CAL_SAMPLES ? GP_STATE_CALIBRATING : GP_STATE_LEVEL, d.state);
    }
    TEST_ASSERT_EQUAL(GP_CAL_EVENT_DONE, GroundProjection_TakeCalEvent(&d));
    TEST_ASSERT_EQUAL_UINT16(GP_MIN_RANGE_MM - 1U, d.baseline_mm);
}

void test_GroundProjection_PoseGate_CalibrationStillTimesOutAndDetectsLostSensor(void)
{
    Gp_Detector_t d;
    uint32_t start_ms = Calibrate_And_Settle(&d, 1000U);
    GroundProjection_StartCalibration(&d, start_ms);

    for (uint32_t elapsed_ms = TOF_PERIOD_MS; elapsed_ms <= GP_CAL_TIMEOUT_MS;
         elapsed_ms += TOF_PERIOD_MS)
    {
        GroundProjection_FeedProjected(&d, GP_RANGE_POSE_UNAVAILABLE, 0U, start_ms + elapsed_ms, NULL);
    }
    TEST_ASSERT_EQUAL(GP_CAL_EVENT_TIMEOUT, GroundProjection_TakeCalEvent(&d));
    TEST_ASSERT_EQUAL(GP_STATE_POSE_UNAVAILABLE, d.state);
    TEST_ASSERT_FALSE(d.is_calibrating);

    start_ms += GP_CAL_TIMEOUT_MS;
    GroundProjection_StartCalibration(&d, start_ms);
    GroundProjection_FeedProjected(&d, GP_RANGE_POSE_UNAVAILABLE, 0U, start_ms, NULL);
    GroundProjection_CancelCalibration(&d, start_ms);
    TEST_ASSERT_EQUAL(GP_CAL_EVENT_CANCELLED, GroundProjection_TakeCalEvent(&d));

    GroundProjection_StartCalibration(&d, start_ms);
    GroundProjection_FeedProjected(&d, GP_RANGE_POSE_UNAVAILABLE, 0U, start_ms, NULL);
    GroundProjection_FeedProjected(&d, GP_RANGE_INVALID, 0U, start_ms + GP_STALE_TIMEOUT_MS, NULL);
    TEST_ASSERT_EQUAL(GP_CAL_EVENT_SENSOR_FAULT, GroundProjection_TakeCalEvent(&d));
    TEST_ASSERT_EQUAL(GP_STATE_SENSOR_FAULT, d.state);
    TEST_ASSERT_FALSE(d.is_calibrating);
    TEST_ASSERT_EQUAL_UINT16(1000U, d.baseline_mm);
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
    TEST_ASSERT_EQUAL(GP_CAL_EVENT_CANCELLED, GroundProjection_TakeCalEvent(&d));
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

/* Week 6 section 6.3: a finished calibration is reported once, for the 1 s "saved" cue */
void test_GroundProjection_Calibration_ReportsDoneOnce(void)
{
    Gp_Detector_t d;
    GroundProjection_Init(&d, 0U);
    GroundProjection_SetArmed(&d, true, 0U);
    GroundProjection_StartCalibration(&d, 0U);

    for (uint32_t i = 0U; i < GP_CAL_SAMPLES; i++)
    {
        TEST_ASSERT_EQUAL(GP_CAL_EVENT_NONE, d.cal_event);
        GroundProjection_Feed(&d, GP_RANGE_VALID, 1000U, (i + 1U) * TOF_PERIOD_MS);
    }

    TEST_ASSERT_EQUAL(GP_CAL_EVENT_DONE, GroundProjection_TakeCalEvent(&d));
    TEST_ASSERT_EQUAL(GP_CAL_EVENT_NONE, GroundProjection_TakeCalEvent(&d));
}

/* Readings that never settle (cane moving) fail after 10 s and keep the old baseline */
void test_GroundProjection_CalibrationTimeout_KeepsOldBaseline(void)
{
    Gp_Detector_t d;
    uint32_t start_ms = Calibrate_And_Settle(&d, 1000U);
    GroundProjection_StartCalibration(&d, start_ms);

    uint32_t t = start_ms;
    while (d.state == GP_STATE_CALIBRATING)
    {
        t += TOF_PERIOD_MS;
        GroundProjection_Feed(&d, GP_RANGE_VALID, (((t / TOF_PERIOD_MS) % 2U) == 0U) ? 1000U : 1100U, t);
    }

    TEST_ASSERT_EQUAL_UINT32(GP_CAL_TIMEOUT_MS, t - start_ms);
    TEST_ASSERT_EQUAL(GP_CAL_EVENT_TIMEOUT, GroundProjection_TakeCalEvent(&d));
    TEST_ASSERT_TRUE(d.is_calibrated);
    TEST_ASSERT_EQUAL(1000U, d.baseline_mm);
}

/* With no baseline (e.g. boot auto-calibration on a moving cane), a timeout falls back to the prompt */
void test_GroundProjection_CalibrationTimeout_WithoutBaseline_PromptsAgain(void)
{
    Gp_Detector_t d;
    GroundProjection_Init(&d, 0U);
    GroundProjection_SetArmed(&d, true, 0U);
    GroundProjection_StartCalibration(&d, 0U);

    uint32_t t = 0U;
    while ((d.state == GP_STATE_CALIBRATING) && (t < (2U * GP_CAL_TIMEOUT_MS)))
    {
        t += TOF_PERIOD_MS;
        GroundProjection_Feed(&d, GP_RANGE_VALID, (((t / TOF_PERIOD_MS) % 2U) == 0U) ? 1000U : 1100U, t);
    }

    TEST_ASSERT_EQUAL(GP_CAL_EVENT_TIMEOUT, GroundProjection_TakeCalEvent(&d));
    TEST_ASSERT_FALSE(d.is_calibrated);
    TEST_ASSERT_EQUAL(ALERT_CALIBRATE_PROMPT, GroundProjection_GetHapticRequest(&d));
}

/* A ToF that stops answering during calibration fails it within 500 ms */
void test_GroundProjection_SensorLostDuringCalibration_ReportsSensorFault(void)
{
    Gp_Detector_t d;
    uint32_t start_ms = Calibrate_And_Settle(&d, 1000U);
    GroundProjection_StartCalibration(&d, start_ms);

    uint32_t t = start_ms;
    while (d.state == GP_STATE_CALIBRATING)
    {
        t += TOF_PERIOD_MS;
        GroundProjection_Feed(&d, GP_RANGE_INVALID, 0U, t);
    }

    TEST_ASSERT_EQUAL_UINT32(GP_STALE_TIMEOUT_MS, t - start_ms);
    TEST_ASSERT_EQUAL(GP_CAL_EVENT_SENSOR_FAULT, GroundProjection_TakeCalEvent(&d));
    TEST_ASSERT_EQUAL(1000U, d.baseline_mm);
}

/* Regression: boot calibration stamped with the real boot time completes on steady readings */
void test_GroundProjection_BootCalibration_StampedWithBootTime_Completes(void)
{
    Gp_Detector_t d;
    uint32_t boot_ms = 1500U;   /* Cane_Main runs this after its 1 s USB wait and sensor init */
    GroundProjection_Init(&d, boot_ms);
    GroundProjection_SetArmed(&d, true, boot_ms);
    GroundProjection_StartCalibration(&d, boot_ms);

    uint32_t t = boot_ms;
    for (uint32_t i = 0U; i < GP_CAL_SAMPLES; i++)
    {
        t += TOF_PERIOD_MS;
        GroundProjection_Feed(&d, GP_RANGE_VALID, 713U, t);
    }

    TEST_ASSERT_EQUAL(GP_STATE_LEVEL, d.state);
    TEST_ASSERT_EQUAL(GP_CAL_EVENT_DONE, GroundProjection_TakeCalEvent(&d));
    TEST_ASSERT_EQUAL(713U, d.baseline_mm);
}

/*
 * The old boot code stamped time 0, so the first sample at 1.5 s looked like a
 * 500 ms sensor dropout and the calibration silently died. It is now reported.
 */
void test_GroundProjection_CalibrationStampedAtZero_ReportsFailureInsteadOfSilence(void)
{
    Gp_Detector_t d;
    GroundProjection_Init(&d, 0U);
    GroundProjection_SetArmed(&d, true, 0U);
    GroundProjection_StartCalibration(&d, 0U);

    GroundProjection_Feed(&d, GP_RANGE_VALID, 713U, 1500U);

    TEST_ASSERT_EQUAL(GP_CAL_EVENT_SENSOR_FAULT, GroundProjection_TakeCalEvent(&d));
    TEST_ASSERT_FALSE(d.is_calibrated);
}

void test_GroundProjection_CancelledCalibration_DoesNotReplayOnNextGrip(void)
{
    Gp_Detector_t d;
    GroundProjection_Init(&d, 0U);
    GroundProjection_SetArmed(&d, true, 0U);
    GroundProjection_StartCalibration(&d, 0U);
    for (uint32_t i = 0U; i < GP_CAL_SAMPLES; i++)
    {
        GroundProjection_Feed(&d, GP_RANGE_VALID, 1000U, i * 10U);
    }
    TEST_ASSERT_EQUAL(GP_CAL_EVENT_DONE, GroundProjection_TakeCalEvent(&d));

    /* Recalibration started, then the user lets go (Cane_Main: cancel, then disarm). */
    GroundProjection_StartCalibration(&d, 1000U);
    GroundProjection_CancelCalibration(&d, 1100U);
    GroundProjection_SetArmed(&d, false, 1100U);

    /* Later grip re-arms without restarting calibration (already calibrated). */
    GroundProjection_SetArmed(&d, true, 60000U);
    TEST_ASSERT_EQUAL(GP_CAL_EVENT_NONE, GroundProjection_TakeCalEvent(&d));
    TEST_ASSERT_TRUE(d.is_calibrated);
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_GroundProjection_CalibrationAndLevel);
    RUN_TEST(test_GroundProjection_Cliff_StepDown_Detection);
    RUN_TEST(test_GroundProjection_SevereCliff_FastPathBypass);
    RUN_TEST(test_GroundProjection_NoGround_SustainedVoting);
    RUN_TEST(test_GroundProjection_TiltCalibration_AveragesFinalRangeWindow);
    RUN_TEST(test_GroundProjection_TiltCalibration_KeepsReferenceWithMissingSamplesOrCancellation);
    RUN_TEST(test_GroundProjection_TiltCalibration_RejectsAnInconsistentPose);
    RUN_TEST(test_GroundProjection_PoseGate_UsesFreshSamplesOnResume);
    RUN_TEST(test_GroundProjection_PoseGate_ResetsNoReturnVotes);
    RUN_TEST(test_GroundProjection_PoseGate_DiscardsPartialCalibrationWindow);
    RUN_TEST(test_GroundProjection_PoseGate_CalibrationStillTimesOutAndDetectsLostSensor);
    RUN_TEST(test_GroundProjection_RebootWithoutCalibration_PromptsAndNeverWarns);
    RUN_TEST(test_GroundProjection_CancelledRecalibration_KeepsOldBaseline);
    RUN_TEST(test_GroundProjection_CompletedRecalibration_ReplacesBaseline);
    RUN_TEST(test_GroundProjection_TofFault_RequestsCautionWithin500ms);
    RUN_TEST(test_GroundProjection_StepDownLatency_400ms);
    RUN_TEST(test_GroundProjection_SevereDropLatency_200ms);
    RUN_TEST(test_GroundProjection_StepUpLatency_400ms);
    RUN_TEST(test_GroundProjection_Calibration_ReportsDoneOnce);
    RUN_TEST(test_GroundProjection_CalibrationTimeout_KeepsOldBaseline);
    RUN_TEST(test_GroundProjection_CalibrationTimeout_WithoutBaseline_PromptsAgain);
    RUN_TEST(test_GroundProjection_SensorLostDuringCalibration_ReportsSensorFault);
    RUN_TEST(test_GroundProjection_BootCalibration_StampedWithBootTime_Completes);
    RUN_TEST(test_GroundProjection_CalibrationStampedAtZero_ReportsFailureInsteadOfSilence);
    RUN_TEST(test_GroundProjection_CancelledCalibration_DoesNotReplayOnNextGrip);
    return UNITY_END();
}
