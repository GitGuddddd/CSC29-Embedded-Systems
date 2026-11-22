#include "unity.h"
#include "app/AlertManager/AlertManager.h"
#include "app/AlertManager/AlertManager_Cfg.h"
#include "app/ThreatAssessment/ThreatAssessment.h" /* header only: cue constants */

void setUp(void)
{
    AlertManager_Init();
}

void tearDown(void)
{
}

void test_AlertManager_PitchMapping(void)
{
    /* Far end of the alert zone -> 1500Hz */
    TEST_ASSERT_EQUAL_UINT16(1500U, AlertManager_MapDistanceToPitch(ALERT_OBSTACLE_MAX_DIST_MM));

    /* 300mm -> 3500Hz */
    TEST_ASSERT_EQUAL_UINT16(3500U, AlertManager_MapDistanceToPitch(300U));

    /* Midpoint of 300..3000 mm -> 2500Hz */
    TEST_ASSERT_EQUAL_UINT16(2500U, AlertManager_MapDistanceToPitch(1650U));
}

void test_AlertManager_FallPreemptsObstacle(void)
{
    Alert_Output_t out;

    /* 1. Request obstacle alert at 1000mm */
    AlertManager_Arbitrate(ALERT_OBSTACLE_PROXIMITY, 1000U, 100U, &out);
    TEST_ASSERT_EQUAL(ALERT_OBSTACLE_PROXIMITY, out.active_alert);
    TEST_ASSERT_TRUE(out.buzzer_active);

    /* 2. Fall happens: should pre-empt obstacle immediately */
    AlertManager_Arbitrate(ALERT_FALL_ALARM, 0U, 200U, &out);
    TEST_ASSERT_EQUAL(ALERT_FALL_ALARM, out.active_alert);
    TEST_ASSERT_TRUE(out.fall_latched);
    TEST_ASSERT_TRUE(out.buzzer_active);

    /* 3. Even if obstacle is requested in next cycle, fall alarm remains latched */
    AlertManager_Arbitrate(ALERT_OBSTACLE_PROXIMITY, 800U, 300U, &out);
    TEST_ASSERT_EQUAL(ALERT_FALL_ALARM, out.active_alert);
}

void test_AlertManager_IncidentDoesNotSilentlyExpire_AndPauseDoesNotClear(void)
{
    Alert_Output_t out;

    /* Fall triggers at t = 1,000 ms */
    AlertManager_Arbitrate(ALERT_FALL_ALARM, 0U, 1000U, &out);
    TEST_ASSERT_EQUAL(ALERT_FALL_ALARM, out.active_alert);
    TEST_ASSERT_TRUE(out.buzzer_active);
    TEST_ASSERT_TRUE(out.fall_latched);

    /* The pattern repeats; elapsed time alone does not clear/pause the incident. */
    AlertManager_Arbitrate(ALERT_FALL_ALARM, 0U, 30000U, &out);
    TEST_ASSERT_EQUAL(ALERT_FALL_ALARM, out.active_alert);
    TEST_ASSERT_TRUE(out.buzzer_active);

    /* More than 30 seconds without acknowledgement is still an active alarm. */
    AlertManager_Arbitrate(ALERT_NONE, 0U, 31500U, &out);
    TEST_ASSERT_EQUAL(ALERT_FALL_ALARM, out.active_alert);
    TEST_ASSERT_TRUE(out.fall_latched);

    AlertManager_SetFallState(true, true, 31600U);
    AlertManager_Arbitrate(ALERT_SENSOR_FAULT, 0U, 31600U, &out);
    TEST_ASSERT_EQUAL(ALERT_FALL_MUTED, out.active_alert);
    TEST_ASSERT_TRUE(out.fall_latched);
    TEST_ASSERT_FALSE(out.buzzer_active);

    /* The mode manager's expired-pause decision starts a fresh audible burst. */
    AlertManager_SetFallState(true, false, 31700U);
    AlertManager_Arbitrate(ALERT_NONE, 0U, 31700U, &out);
    TEST_ASSERT_TRUE(out.buzzer_active);
    TEST_ASSERT_TRUE(out.fall_latched);

    AlertManager_SetFallState(false, false, 32000U);
    AlertManager_Arbitrate(ALERT_NONE, 0U, 32000U, &out);
    TEST_ASSERT_EQUAL(ALERT_NONE, out.active_alert);
    TEST_ASSERT_FALSE(out.fall_latched);
}

void test_AlertManager_TenPulses_AndInterBurstSilence(void)
{
    static const uint32_t starts_ms[] = {0U, 250U, 500U, 950U, 1200U,
                                        1850U, 2100U, 2350U, 2800U, 3050U};
    Alert_Output_t out;
    uint32_t start = UINT32_MAX - 100U;
    AlertManager_SetFallState(true, false, start);
    for (uint32_t i = 0U; i < 10U; ++i)
    {
        AlertManager_Arbitrate(ALERT_NONE, 0U, start + starts_ms[i], &out);
        TEST_ASSERT_TRUE(out.buzzer_active);
        AlertManager_Arbitrate(ALERT_NONE, 0U, start + starts_ms[i] + 150U, &out);
        TEST_ASSERT_FALSE(out.buzzer_active);
        TEST_ASSERT_TRUE(out.fall_latched);
    }
    AlertManager_Arbitrate(ALERT_NONE, 0U, start + 5690U, &out);
    TEST_ASSERT_FALSE(out.buzzer_active);
    AlertManager_Arbitrate(ALERT_NONE, 0U, start + 5700U, &out);
    TEST_ASSERT_TRUE(out.buzzer_active);
}

void test_AlertManager_TechnicalFaultHasTwoPulses(void)
{
    Alert_Output_t out;
    AlertManager_Arbitrate(ALERT_SENSOR_FAULT, 0U, 1000U, &out);
    TEST_ASSERT_TRUE(out.buzzer_active);
    AlertManager_Arbitrate(ALERT_SENSOR_FAULT, 0U, 1150U, &out);
    TEST_ASSERT_FALSE(out.buzzer_active);
    AlertManager_Arbitrate(ALERT_SENSOR_FAULT, 0U, 1350U, &out);
    TEST_ASSERT_TRUE(out.buzzer_active);
    AlertManager_Arbitrate(ALERT_SENSOR_FAULT, 0U, 1500U, &out);
    TEST_ASSERT_FALSE(out.buzzer_active);
    AlertManager_Arbitrate(ALERT_SENSOR_FAULT, 0U, 6000U, &out);
    TEST_ASSERT_TRUE(out.buzzer_active);
}

void test_AlertManager_DeliveryFallbackIsSparseAndLowDuty(void)
{
    Alert_Output_t out;
    AlertManager_Arbitrate(ALERT_FALL_DELIVERY_FAULT, 0U, 1000U, &out);
    TEST_ASSERT_EQUAL(ALERT_FALL_DELIVERY_FAULT, out.active_alert);
    TEST_ASSERT_TRUE(out.buzzer_active);
    TEST_ASSERT_EQUAL_UINT16(ALERT_DELIVERY_FREQ_HZ, out.generated_freq_hz);
    TEST_ASSERT_EQUAL_UINT8(ALERT_DELIVERY_DUTY_PERCENT, out.generated_duty_percent);
    AlertManager_Arbitrate(ALERT_FALL_DELIVERY_FAULT, 0U,
                           1000U + ALERT_DELIVERY_PULSE_MS, &out);
    TEST_ASSERT_FALSE(out.buzzer_active);
    AlertManager_Arbitrate(ALERT_FALL_DELIVERY_FAULT, 0U,
                           1000U + ALERT_DELIVERY_PERIOD_MS, &out);
    TEST_ASSERT_TRUE(out.buzzer_active);
}

void test_AlertManager_Synced_DefaultPlaysOnlyOnCane(void)
{
    Alert_Output_t out_glasses;
    Alert_Output_t out_cane;

    /* 1. Glasses-initiated obstacle plays on Glasses only */
    Alert_SyncInput_t in_glasses_obs = {
        .is_glasses_node = true,
        .cane_link_fresh = true,
        .cane_buzzer_fault = false,
        .delivery_fallback = false,
        .obstacle_active = true,
        .obstacle_dist_mm = 800U
    };
    AlertManager_ArbitrateSynced(&in_glasses_obs, 1000U, &out_glasses);
    TEST_ASSERT_TRUE(out_glasses.buzzer_active);
    TEST_ASSERT_EQUAL(ALERT_OBSTACLE_PROXIMITY, out_glasses.active_alert);

    Alert_SyncInput_t in_cane_obs = {
        .is_glasses_node = false,
        .cane_link_fresh = true,
        .cane_buzzer_fault = false,
        .obstacle_active = false,
        .obstacle_dist_mm = 0U
    };
    AlertManager_ArbitrateSynced(&in_cane_obs, 1000U, &out_cane);
    TEST_ASSERT_FALSE(out_cane.buzzer_active);

    /* 2. Glasses-initiated head fall plays on the Cane, not the Glasses */
    AlertManager_Init();
    Alert_SyncInput_t in_glasses_fall = {
        .is_glasses_node = true,
        .cane_link_fresh = true,
        .cane_buzzer_fault = false,
        .delivery_fallback = false,
        .local_fall_latched = true
    };
    AlertManager_ArbitrateSynced(&in_glasses_fall, 2000U, &out_glasses);
    TEST_ASSERT_FALSE(out_glasses.buzzer_active);
    TEST_ASSERT_EQUAL(ALERT_NONE, out_glasses.active_alert);

    AlertManager_Init();
    Alert_SyncInput_t in_cane_peer_fall = {
        .is_glasses_node = false,
        .cane_link_fresh = true,
        .cane_buzzer_fault = false,
        .local_fall_latched = false,
        .peer_fall_active = true
    };
    AlertManager_ArbitrateSynced(&in_cane_peer_fall, 2000U, &out_cane);
    TEST_ASSERT_TRUE(out_cane.buzzer_active);
    TEST_ASSERT_EQUAL(ALERT_FALL_ALARM, out_cane.active_alert);

    /* 3. Cane-initiated fall plays on Cane only */
    AlertManager_Init();
    Alert_SyncInput_t in_cane_local_fall = {
        .is_glasses_node = false,
        .cane_link_fresh = true,
        .cane_buzzer_fault = false,
        .local_fall_latched = true
    };
    AlertManager_ArbitrateSynced(&in_cane_local_fall, 3000U, &out_cane);
    TEST_ASSERT_TRUE(out_cane.buzzer_active);
    TEST_ASSERT_EQUAL(ALERT_FALL_ALARM, out_cane.active_alert);

    Alert_SyncInput_t in_glasses_peer_fall = {
        .is_glasses_node = true,
        .cane_link_fresh = true,
        .cane_buzzer_fault = false,
        .local_fall_latched = false,
        .peer_fall_active = true
    };
    AlertManager_ArbitrateSynced(&in_glasses_peer_fall, 3000U, &out_glasses);
    TEST_ASSERT_FALSE(out_glasses.buzzer_active);
}

void test_AlertManager_Synced_GlassesHeadFallFallbackCueWithoutReceipt(void)
{
    Alert_Output_t out;
    Alert_SyncInput_t in = {
        .is_glasses_node = true,
        .cane_link_fresh = false,
        .local_fall_latched = true,
        .delivery_fallback = true,
        .obstacle_active = true,
        .obstacle_dist_mm = 800U
    };

    /* No cane receipt: the sparse delivery cue, not the siren, and it
     * takes priority over obstacle beeps while the fall is latched.
     */
    AlertManager_ArbitrateSynced(&in, 1000U, &out);
    TEST_ASSERT_EQUAL(ALERT_FALL_DELIVERY_FAULT, out.active_alert);
    TEST_ASSERT_TRUE(out.buzzer_active);
    TEST_ASSERT_EQUAL_UINT16(ALERT_DELIVERY_FREQ_HZ, out.generated_freq_hz);

    /* Receipt confirmed: glasses go quiet, obstacle beeps stay suppressed. */
    in.delivery_fallback = false;
    in.cane_link_fresh = true;
    AlertManager_ArbitrateSynced(&in, 1100U, &out);
    TEST_ASSERT_FALSE(out.buzzer_active);
    TEST_ASSERT_EQUAL(ALERT_NONE, out.active_alert);
}

void test_AlertManager_Synced_CaneBuzzerFaultFailoverToGlasses(void)
{
    Alert_Output_t out_glasses;
    Alert_Output_t out_cane;

    /* 1. Head Fall failover */
    AlertManager_Init();
    Alert_SyncInput_t in_cane_fall = {
        .is_glasses_node = false,
        .cane_link_fresh = true,
        .cane_buzzer_fault = true,
        .peer_fall_active = true
    };
    AlertManager_ArbitrateSynced(&in_cane_fall, 1000U, &out_cane);
    TEST_ASSERT_FALSE(out_cane.buzzer_active);
    TEST_ASSERT_EQUAL_UINT16(0U, out_cane.generated_freq_hz);

    Alert_SyncInput_t in_glasses_fall = {
        .is_glasses_node = true,
        .cane_link_fresh = true,
        .cane_buzzer_fault = true,
        .delivery_fallback = true,
        .local_fall_latched = true
    };
    AlertManager_ArbitrateSynced(&in_glasses_fall, 1000U, &out_glasses);
    TEST_ASSERT_TRUE(out_glasses.buzzer_active);
    TEST_ASSERT_EQUAL(ALERT_FALL_ALARM, out_glasses.active_alert);

    /* 2. Cane Fall failover */
    AlertManager_Init();
    Alert_SyncInput_t in_cane_local_fall = {
        .is_glasses_node = false,
        .cane_link_fresh = true,
        .cane_buzzer_fault = true,
        .local_fall_latched = true
    };
    AlertManager_ArbitrateSynced(&in_cane_local_fall, 2000U, &out_cane);
    TEST_ASSERT_FALSE(out_cane.buzzer_active);

    Alert_SyncInput_t in_glasses_cane_fall = {
        .is_glasses_node = true,
        .cane_link_fresh = true,
        .cane_buzzer_fault = true,
        .peer_fall_active = true
    };
    AlertManager_ArbitrateSynced(&in_glasses_cane_fall, 2000U, &out_glasses);
    TEST_ASSERT_TRUE(out_glasses.buzzer_active);
    TEST_ASSERT_EQUAL(ALERT_FALL_ALARM, out_glasses.active_alert);

    /* 3. Obstacle Proximity failover */
    AlertManager_Init();
    Alert_SyncInput_t in_cane_obs = {
        .is_glasses_node = false,
        .cane_link_fresh = true,
        .cane_buzzer_fault = true,
        .obstacle_active = true,
        .obstacle_dist_mm = 600U
    };
    AlertManager_ArbitrateSynced(&in_cane_obs, 3000U, &out_cane);
    TEST_ASSERT_FALSE(out_cane.buzzer_active);

    Alert_SyncInput_t in_glasses_obs = {
        .is_glasses_node = true,
        .cane_link_fresh = true,
        .cane_buzzer_fault = true,
        .obstacle_active = true,
        .obstacle_dist_mm = 600U
    };
    AlertManager_ArbitrateSynced(&in_glasses_obs, 3000U, &out_glasses);
    TEST_ASSERT_TRUE(out_glasses.buzzer_active);
    TEST_ASSERT_EQUAL(ALERT_OBSTACLE_PROXIMITY, out_glasses.active_alert);

    /* 4. Sensor Fault failover */
    AlertManager_Init();
    Alert_SyncInput_t in_cane_fault = {
        .is_glasses_node = false,
        .cane_link_fresh = true,
        .cane_buzzer_fault = true,
        .sensor_fault = true
    };
    AlertManager_ArbitrateSynced(&in_cane_fault, 4000U, &out_cane);
    TEST_ASSERT_FALSE(out_cane.buzzer_active);

    Alert_SyncInput_t in_glasses_fault = {
        .is_glasses_node = true,
        .cane_link_fresh = true,
        .cane_buzzer_fault = true,
        .sensor_fault = true
    };
    AlertManager_ArbitrateSynced(&in_glasses_fault, 4000U, &out_glasses);
    TEST_ASSERT_TRUE(out_glasses.buzzer_active);
    TEST_ASSERT_EQUAL(ALERT_SENSOR_FAULT, out_glasses.active_alert);
}

void test_AlertManager_Synced_FaultRoutingPerNode_AndBothOnCaneOnly(void)
{
    Alert_Output_t out_cane;
    Alert_Output_t out_glasses;

    /* 1. Cane fault only -> Cane buzzes, Glasses silent */
    AlertManager_Init();
    Alert_SyncInput_t cane_only_cane = {
        .is_glasses_node = false,
        .cane_link_fresh = true,
        .sensor_fault = true,
        .peer_sensor_fault = false
    };
    AlertManager_ArbitrateSynced(&cane_only_cane, 1000U, &out_cane);
    TEST_ASSERT_TRUE(out_cane.buzzer_active);
    TEST_ASSERT_EQUAL(ALERT_SENSOR_FAULT, out_cane.active_alert);

    AlertManager_Init();
    Alert_SyncInput_t cane_only_glasses = {
        .is_glasses_node = true,
        .cane_link_fresh = true,
        .sensor_fault = false,
        .peer_sensor_fault = true
    };
    AlertManager_ArbitrateSynced(&cane_only_glasses, 1000U, &out_glasses);
    TEST_ASSERT_FALSE(out_glasses.buzzer_active);

    /* 2. Glasses fault only -> Glasses buzzes, Cane silent */
    AlertManager_Init();
    Alert_SyncInput_t glasses_only_cane = {
        .is_glasses_node = false,
        .cane_link_fresh = true,
        .sensor_fault = false,
        .peer_sensor_fault = true
    };
    AlertManager_ArbitrateSynced(&glasses_only_cane, 2000U, &out_cane);
    TEST_ASSERT_FALSE(out_cane.buzzer_active);

    AlertManager_Init();
    Alert_SyncInput_t glasses_only_glasses = {
        .is_glasses_node = true,
        .cane_link_fresh = true,
        .sensor_fault = true,
        .peer_sensor_fault = false
    };
    AlertManager_ArbitrateSynced(&glasses_only_glasses, 2000U, &out_glasses);
    TEST_ASSERT_TRUE(out_glasses.buzzer_active);
    TEST_ASSERT_EQUAL(ALERT_SENSOR_FAULT, out_glasses.active_alert);

    /* 3. Both have fault -> ONLY Cane buzzes, Glasses silent */
    AlertManager_Init();
    Alert_SyncInput_t both_cane = {
        .is_glasses_node = false,
        .cane_link_fresh = true,
        .sensor_fault = true,
        .peer_sensor_fault = true
    };
    AlertManager_ArbitrateSynced(&both_cane, 3000U, &out_cane);
    TEST_ASSERT_TRUE(out_cane.buzzer_active);
    TEST_ASSERT_EQUAL(ALERT_SENSOR_FAULT, out_cane.active_alert);

    AlertManager_Init();
    Alert_SyncInput_t both_glasses = {
        .is_glasses_node = true,
        .cane_link_fresh = true,
        .sensor_fault = true,
        .peer_sensor_fault = true
    };
    AlertManager_ArbitrateSynced(&both_glasses, 3000U, &out_glasses);
    TEST_ASSERT_FALSE(out_glasses.buzzer_active);
}

void test_AlertManager_CalibrationCuesAreDistinctOneShots(void)
{
    const Alert_SyncInput_t cane = {0};
    Alert_Output_t out;
    uint32_t start = UINT32_MAX - 50U;

    AlertManager_PlayCalibrationCue(ALERT_CAL_STARTED, start);
    AlertManager_ArbitrateSynced(&cane, start + ALERT_CALIBRATION_PULSE_MS - 1U, &out);
    TEST_ASSERT_TRUE(out.buzzer_active);
    TEST_ASSERT_EQUAL(ALERT_CAL_STARTED, out.active_alert);
    TEST_ASSERT_EQUAL_UINT16(ALERT_CAL_STARTED_FREQ_HZ, out.generated_freq_hz);

    AlertManager_ArbitrateSynced(&cane, start + ALERT_CALIBRATION_PULSE_MS, &out);
    TEST_ASSERT_FALSE(out.buzzer_active);

    AlertManager_PlayCalibrationCue(ALERT_CAL_DONE, 200U);
    AlertManager_ArbitrateSynced(&cane, 200U, &out);
    TEST_ASSERT_TRUE(out.buzzer_active);
    TEST_ASSERT_EQUAL(ALERT_CAL_DONE, out.active_alert);
    TEST_ASSERT_EQUAL_UINT16(ALERT_CAL_DONE_FREQ_HZ, out.generated_freq_hz);

    AlertManager_PlayCalibrationCue(ALERT_CAL_FAILED, 210U);
    AlertManager_ArbitrateSynced(&cane, 210U, &out);
    TEST_ASSERT_TRUE(out.buzzer_active);
    TEST_ASSERT_EQUAL(ALERT_CAL_FAILED, out.active_alert);
    TEST_ASSERT_EQUAL_UINT16(ALERT_CAL_FAILED_FREQ_HZ, out.generated_freq_hz);
}

void test_AlertManager_CalibrationCueYieldsToSafetyAlerts(void)
{
    Alert_SyncInput_t cane = { .sensor_fault = true };
    Alert_Output_t out;

    AlertManager_PlayCalibrationCue(ALERT_CAL_STARTED, 10U);
    AlertManager_ArbitrateSynced(&cane, 10U, &out);
    TEST_ASSERT_EQUAL(ALERT_SENSOR_FAULT, out.active_alert);

    cane.sensor_fault = false;
    AlertManager_ArbitrateSynced(&cane, 20U, &out);
    TEST_ASSERT_FALSE(out.buzzer_active);

    cane.local_fall_latched = true;
    AlertManager_PlayCalibrationCue(ALERT_CAL_DONE, 30U);
    AlertManager_ArbitrateSynced(&cane, 30U, &out);
    TEST_ASSERT_EQUAL(ALERT_FALL_ALARM, out.active_alert);

    cane.local_fall_latched = false;
    cane.peer_fall_active = true;
    AlertManager_PlayCalibrationCue(ALERT_CAL_DONE, 40U);
    AlertManager_ArbitrateSynced(&cane, 40U, &out);
    TEST_ASSERT_EQUAL(ALERT_FALL_ALARM, out.active_alert);

    AlertManager_Init();
    cane.peer_fall_active = false;
    cane.cane_buzzer_fault = true;
    AlertManager_PlayCalibrationCue(ALERT_CAL_FAILED, 50U);
    AlertManager_ArbitrateSynced(&cane, 50U, &out);
    TEST_ASSERT_FALSE(out.buzzer_active);
}

/* ---- FR9 obstacle direction-cue beep pattern (ported from putra-branch) ---- */

/* Count rising edges of buzzer_active over [0, span_ms) sampled every 10 ms. */
static uint32_t Count_Beeps(uint8_t cue, uint16_t dist_mm, uint32_t span_ms, uint32_t * p_on_ms)
{
    uint32_t beeps = 0U;
    uint32_t on_ms = 0U;
    bool prev = false;
    Alert_Output_t out;

    AlertManager_Init();
    AlertManager_SetObstacleCue(cue);
    for (uint32_t t = 0U; t < span_ms; t += 10U)
    {
        AlertManager_Arbitrate(ALERT_OBSTACLE_PROXIMITY, dist_mm, t, &out);
        if (out.buzzer_active)
        {
            on_ms += 10U;
            if (!prev)
            {
                beeps++;
            }
        }
        prev = out.buzzer_active;
    }
    if (p_on_ms != NULL)
    {
        *p_on_ms = on_ms;
    }
    return beeps;
}

void test_AlertManager_CueBitsMatchThreatAssessment(void)
{
    TEST_ASSERT_EQUAL_UINT8(THREAT_CUE_STOP, ALERT_CUE_STOP);
    TEST_ASSERT_EQUAL_UINT8(THREAT_CUE_SIDE_SHIFT, ALERT_CUE_SIDE_SHIFT);
    TEST_ASSERT_EQUAL_UINT8(THREAT_LANE_CENTER, ALERT_CUE_SIDE_CENTER);
    TEST_ASSERT_EQUAL_UINT8(THREAT_LANE_LEFT, ALERT_CUE_SIDE_LEFT);
    TEST_ASSERT_EQUAL_UINT8(THREAT_LANE_RIGHT, ALERT_CUE_SIDE_RIGHT);
    TEST_ASSERT_EQUAL_UINT8(THREAT_CUE_SIDE_ALL_BLOCKED, ALERT_CUE_SIDE_ALL_BLOCKED);
}

void test_AlertManager_ObstacleCenterCueIsOneBeep(void)
{
    uint32_t on_ms = 0U;
    /* 1000 mm: pause 150 + (1000-300)*300/1700 ~ 326 ms; cycle = 300 + 326 ms. */
    TEST_ASSERT_EQUAL_UINT32(4U, Count_Beeps(0U, 1000U, 2000U, &on_ms));
    TEST_ASSERT_EQUAL_UINT32(4U * ALERT_OBSTACLE_PULSE_MS, on_ms);
}

void test_AlertManager_ObstacleLeftIsTwoBeepsRightIsThree(void)
{
    const uint8_t left = (uint8_t)(ALERT_CUE_SIDE_LEFT << ALERT_CUE_SIDE_SHIFT);
    const uint8_t right = (uint8_t)(ALERT_CUE_SIDE_RIGHT << ALERT_CUE_SIDE_SHIFT);
    uint32_t on_ms = 0U;

    /* At the far end of the zone the cycle is burst + 450 ms pause: LEFT 600+450, RIGHT 900+450. */
    TEST_ASSERT_EQUAL_UINT32(2U, Count_Beeps(left, ALERT_OBSTACLE_MAX_DIST_MM, 1000U, &on_ms));
    TEST_ASSERT_EQUAL_UINT32(300U, on_ms);
    TEST_ASSERT_EQUAL_UINT32(3U, Count_Beeps(right, ALERT_OBSTACLE_MAX_DIST_MM, 1300U, &on_ms));
    TEST_ASSERT_EQUAL_UINT32(450U, on_ms);
}

void test_AlertManager_AllBlockedIsFastFourBeepBurst(void)
{
    uint32_t on_ms = 0U;
    const uint8_t blocked = (uint8_t)(ALERT_CUE_SIDE_ALL_BLOCKED << ALERT_CUE_SIDE_SHIFT);
    /* 2000 mm: burst = 4 x (110 on + 90 off) = 800 ms, then a 450 ms pause. */
    TEST_ASSERT_EQUAL_UINT32(4U, Count_Beeps(blocked, 2000U, 1000U, &on_ms));
    TEST_ASSERT_EQUAL_UINT32(4U * ALERT_OBSTACLE_BLOCKED_PULSE_MS, on_ms);
}

void test_AlertManager_ObstaclePauseShrinksWithDistance(void)
{
    const uint8_t left = (uint8_t)(ALERT_CUE_SIDE_LEFT << ALERT_CUE_SIDE_SHIFT);
    /* Beeps in a fixed 4 s window: more repeats = shorter pause. */
    uint32_t far_beeps = Count_Beeps(left, 2000U, 4000U, NULL);   /* cycle 1050 ms */
    uint32_t near_beeps = Count_Beeps(left, 300U, 4000U, NULL);   /* cycle 750 ms */
    TEST_ASSERT_TRUE(near_beeps > far_beeps);
}

static Alert_Output_t Obstacle_Step(uint8_t cue, uint16_t dist_mm, uint32_t t_ms);

void test_AlertManager_StopIsContinuousAtMaxPitchAtAnyDistance(void)
{
    const uint16_t dists[] = {2000U, 1000U, 300U};
    for (uint32_t k = 0U; k < 3U; k++)
    {
        uint32_t on_ms = 0U;
        AlertManager_Init();
        Alert_Output_t out = {0};
        AlertManager_SetObstacleCue(ALERT_CUE_STOP);
        for (uint32_t t = 0U; t < 1000U; t += 10U)
        {
            AlertManager_Arbitrate(ALERT_OBSTACLE_PROXIMITY, dists[k], t, &out);
            on_ms += out.buzzer_active ? 10U : 0U;
        }
        TEST_ASSERT_EQUAL_UINT32(1000U, on_ms);
        TEST_ASSERT_EQUAL_UINT16(ALERT_BUZZER_MAX_FREQ_HZ, out.generated_freq_hz);
        TEST_ASSERT_EQUAL_UINT8(ALERT_STANDARD_DUTY_PERCENT, out.generated_duty_percent);
    }
}

void test_AlertManager_StopStaysContinuousAndKeepsPitch(void)
{
    Alert_Output_t out = {0};
    AlertManager_Init();
    AlertManager_SetObstacleCue(ALERT_CUE_STOP);
    for (uint32_t t = 0U; t < 8000U; t += 10U)
    {
        AlertManager_Arbitrate(ALERT_OBSTACLE_PROXIMITY, 800U, t, &out);
        TEST_ASSERT_TRUE(out.buzzer_active);   /* never pulses, however long STOP lasts */
        TEST_ASSERT_EQUAL_UINT16(ALERT_BUZZER_MAX_FREQ_HZ, out.generated_freq_hz);
    }
}

void test_AlertManager_NoFreeLaneStretchesItsPauseWhenItKeepsPlaying(void)
{
    const uint8_t blocked = (uint8_t)(ALERT_CUE_SIDE_ALL_BLOCKED << ALERT_CUE_SIDE_SHIFT);
    uint32_t first = 0U;
    uint32_t later = 0U;
    bool prev = false;

    AlertManager_Init();
    for (uint32_t t = 0U; t < 10000U; t += 10U)
    {
        const bool on = Obstacle_Step(blocked, 1000U, t).buzzer_active;
        if (on && !prev)
        {
            if (t < ALERT_OBSTACLE_BLOCKED_REPEAT_MS) { first++; } else { later++; }
        }
        prev = on;
    }
    /* 5 s window each: fewer bursts once the pause has been stretched. */
    TEST_ASSERT_TRUE(later < first);
}

void test_AlertManager_StopLeavesAfterDwellNotInstantly(void)
{
    /* STOP for 1 s, then the requested cue becomes centre (1-beep burst). */
    Alert_Output_t out = {0};
    for (uint32_t t = 0U; t < 1000U; t += 10U)
    {
        out = Obstacle_Step(ALERT_CUE_STOP, 1200U, t);
    }
    for (uint32_t t = 1000U; t < 1000U + ALERT_OBSTACLE_CUE_DWELL_MS - 10U; t += 10U)
    {
        out = Obstacle_Step(0U, 1200U, t);
        TEST_ASSERT_TRUE(out.buzzer_active);                             /* still STOP */
        TEST_ASSERT_EQUAL_UINT16(ALERT_BUZZER_MAX_FREQ_HZ, out.generated_freq_hz);
    }
    uint32_t silent = 0U;
    for (uint32_t t = 1000U + ALERT_OBSTACLE_CUE_DWELL_MS; t < 3000U; t += 10U)
    {
        silent += Obstacle_Step(0U, 1200U, t).buzzer_active ? 0U : 1U;
    }
    TEST_ASSERT_TRUE(silent > 0U); /* then the 1-beep pattern has gaps */
}

void test_AlertManager_ObstacleLoudnessStaysAtEarSafeDuty(void)
{
    Alert_Output_t out;
    AlertManager_SetObstacleCue((uint8_t)(ALERT_CUE_STOP | (ALERT_CUE_SIDE_RIGHT << ALERT_CUE_SIDE_SHIFT)));
    AlertManager_Arbitrate(ALERT_OBSTACLE_PROXIMITY, 400U, 0U, &out);
    TEST_ASSERT_EQUAL_UINT8(ALERT_STANDARD_DUTY_PERCENT, out.generated_duty_percent);
}

void test_AlertManager_Synced_GlassesPassesObstacleCueToPattern(void)
{
    Alert_Output_t out;
    Alert_SyncInput_t in = {
        .is_glasses_node = true,
        .obstacle_active = true,
        .obstacle_dist_mm = 1000U,
        .obstacle_cue = (uint8_t)(ALERT_CUE_SIDE_LEFT << ALERT_CUE_SIDE_SHIFT)
    };
    AlertManager_ArbitrateSynced(&in, 0U, &out);
    TEST_ASSERT_TRUE(out.buzzer_active);          /* beep 1 on */
    AlertManager_ArbitrateSynced(&in, 200U, &out);
    TEST_ASSERT_FALSE(out.buzzer_active);         /* gap */
    AlertManager_ArbitrateSynced(&in, 310U, &out);
    TEST_ASSERT_TRUE(out.buzzer_active);          /* beep 2 on */
}

/* ---- Steadiness while walking: cue dwell, distance smoothing, latched pause ---- */

#define CUE_LEFT   ((uint8_t)(ALERT_CUE_SIDE_LEFT << ALERT_CUE_SIDE_SHIFT))
#define CUE_RIGHT  ((uint8_t)(ALERT_CUE_SIDE_RIGHT << ALERT_CUE_SIDE_SHIFT))

static Alert_Output_t Obstacle_Step(uint8_t cue, uint16_t dist_mm, uint32_t t_ms)
{
    Alert_Output_t out;
    AlertManager_SetObstacleCue(cue);
    AlertManager_Arbitrate(ALERT_OBSTACLE_PROXIMITY, dist_mm, t_ms, &out);
    return out;
}

/* Run the obstacle renderer for 3 s and return the on/off sample per 10 ms. */
static void Record_Pattern(bool * p_on, uint32_t n, uint8_t (*cue_at)(uint32_t t_ms))
{
    AlertManager_Init();
    for (uint32_t i = 0U; i < n; i++)
    {
        p_on[i] = Obstacle_Step(cue_at(i * 10U), 1200U, i * 10U).buzzer_active;
    }
}

static uint8_t Cue_Right(uint32_t t) { (void)t; return CUE_RIGHT; }
static uint8_t Cue_RightWithLeftBlip(uint32_t t) { return ((t >= 300U) && (t < 500U)) ? CUE_LEFT : CUE_RIGHT; }
static uint8_t Cue_RightThenLeft(uint32_t t) { return (t >= 300U) ? CUE_LEFT : CUE_RIGHT; }

void test_AlertManager_CueFlickerShorterThanDwellKeepsPattern(void)
{
    static bool ref[300];
    static bool blip[300];
    Record_Pattern(ref, 300U, Cue_Right);
    Record_Pattern(blip, 300U, Cue_RightWithLeftBlip);
    for (uint32_t i = 0U; i < 300U; i++)
    {
        TEST_ASSERT_EQUAL_INT(ref[i], blip[i]); /* a 200 ms flicker changes nothing */
    }
}

void test_AlertManager_CueChangeAppliesAfterDwell(void)
{
    static bool ref[300];
    static bool changed[300];
    Record_Pattern(ref, 300U, Cue_Right);
    Record_Pattern(changed, 300U, Cue_RightThenLeft);

    const uint32_t apply_idx = (300U + ALERT_OBSTACLE_CUE_DWELL_MS) / 10U;
    for (uint32_t i = 0U; i < apply_idx; i++)
    {
        TEST_ASSERT_EQUAL_INT(ref[i], changed[i]); /* unchanged during the dwell */
    }
    uint32_t diffs = 0U;
    for (uint32_t i = apply_idx; i < 300U; i++)
    {
        diffs += (ref[i] != changed[i]) ? 1U : 0U;
    }
    TEST_ASSERT_TRUE(diffs > 0U); /* then the 2-beep pattern replaces the 3-beep one */
}

void test_AlertManager_StopRisingAppliesImmediatelyAndRestartsCycle(void)
{
    /* LEFT at 2000 mm: burst 0-600 ms, pause until 1050 ms. At 700 ms it is silent. */
    TEST_ASSERT_TRUE(Obstacle_Step(CUE_LEFT, 2000U, 0U).buzzer_active);
    for (uint32_t t = 10U; t < 700U; t += 10U)
    {
        (void)Obstacle_Step(CUE_LEFT, 2000U, t);
    }
    TEST_ASSERT_FALSE(Obstacle_Step(CUE_LEFT, 2000U, 700U).buzzer_active);
    /* STOP appears: it plays at once (continuous), without waiting for the hold time. */
    TEST_ASSERT_TRUE(Obstacle_Step(ALERT_CUE_STOP, 2000U, 710U).buzzer_active);
    TEST_ASSERT_TRUE(Obstacle_Step(ALERT_CUE_STOP, 2000U, 800U).buzzer_active);
}

void test_AlertManager_PitchFollowsDistanceThroughLowPass(void)
{
    Alert_Output_t out = Obstacle_Step(0U, 2000U, 0U);
    TEST_ASSERT_EQUAL_UINT16(AlertManager_MapDistanceToPitch(2000U), out.generated_freq_hz);

    out = Obstacle_Step(0U, 300U, 10U); /* distance jumps to the nearest limit */
    TEST_ASSERT_TRUE(out.generated_freq_hz < AlertManager_MapDistanceToPitch(300U)); /* not instant */
    TEST_ASSERT_TRUE(out.generated_freq_hz > AlertManager_MapDistanceToPitch(2000U));

    uint16_t prev = out.generated_freq_hz;
    for (uint32_t t = 20U; t < 2000U; t += 10U)
    {
        out = Obstacle_Step(0U, 300U, t);
        TEST_ASSERT_TRUE(out.generated_freq_hz >= prev); /* monotonic, no overshoot */
        prev = out.generated_freq_hz;
    }
    TEST_ASSERT_UINT16_WITHIN(40U, AlertManager_MapDistanceToPitch(300U), out.generated_freq_hz);
}

void test_AlertManager_BeepsNeverTruncatedWhileDistanceSweeps(void)
{
    /* Distance sweeps 2000 -> 300 mm in 4 s with a LEFT cue: every beep must stay a full 150 ms. */
    uint32_t run_ms = 0U;
    uint32_t beeps = 0U;
    bool prev = false;

    for (uint32_t t = 0U; t <= 4000U; t += 10U)
    {
        const uint16_t d = (uint16_t)(2000U - ((1700U * t) / 4000U));
        const bool on = Obstacle_Step(CUE_LEFT, d, t).buzzer_active;
        if (on)
        {
            run_ms += 10U;
        }
        else if (prev)
        {
            TEST_ASSERT_EQUAL_UINT32(ALERT_OBSTACLE_PULSE_MS, run_ms);
            run_ms = 0U;
        }
        if (on && !prev)
        {
            beeps++;
        }
        prev = on;
    }
    TEST_ASSERT_TRUE(beeps >= 6U);
}

/* Changing between two beep patterns must not run the old burst into the new one
 * (bench/simulation case: a 2-beep "left free" burst followed straight away by the new
 * pattern's first beep was heard as a 3-beep "right free"). For every moment of change:
 * no beep is cut or merged, and the new pattern starts after a clear silence. */
void test_AlertManager_PatternChangeNeverMergesBursts(void)
{
    for (uint32_t change_ms = 0U; change_ms < 1100U; change_ms += 10U)
    {
        AlertManager_Init();
        const uint32_t apply_ms = change_ms + ALERT_OBSTACLE_CUE_DWELL_MS;
        bool prev_on = false;
        uint32_t on_start = 0U;
        uint32_t off_start = 0U;
        bool seen_off = false;
        bool checked_new = false;
        for (uint32_t t = 0U; t < 4000U; t += 10U)
        {
            const uint8_t cue = (t >= change_ms) ? 0U : CUE_LEFT;
            const bool on = Obstacle_Step(cue, 2000U, t).buzzer_active;
            if (on && !prev_on)
            {
                if (seen_off)
                {
                    TEST_ASSERT_TRUE((t - off_start) >= ALERT_OBSTACLE_BEEP_GAP_MS);
                }
                if (!checked_new && (t >= apply_ms) && seen_off)
                {
                    TEST_ASSERT_TRUE((t - off_start) >= ALERT_OBSTACLE_SWITCH_GAP_MS);
                    checked_new = true;
                }
                on_start = t;
            }
            if (!on && prev_on)
            {
                TEST_ASSERT_EQUAL_UINT32(ALERT_OBSTACLE_PULSE_MS, t - on_start);
                off_start = t;
                seen_off = true;
            }
            prev_on = on;
        }
        TEST_ASSERT_TRUE(checked_new);
    }
}

/* Counted bursts keep a pause clearly longer than the in-burst gap, even close up.
 * The 1-beep centre pattern keeps its original distance-based pause. */
static uint32_t Silence_After_Burst(uint8_t cue, uint32_t beeps)
{
    AlertManager_Init();
    uint32_t count = 0U;
    uint32_t off_at = 0U;
    bool prev_on = false;
    for (uint32_t t = 0U; t < 3000U; t += 10U)
    {
        const bool on = Obstacle_Step(cue, ALERT_OBSTACLE_MIN_DIST_MM, t).buzzer_active;
        if (!on && prev_on)
        {
            off_at = t;
            count++;
        }
        if (on && !prev_on && (count == beeps))
        {
            return t - off_at;
        }
        prev_on = on;
    }
    return 0U;
}

void test_AlertManager_MultiBeepPauseHasFloor(void)
{
    TEST_ASSERT_EQUAL_UINT32(ALERT_OBSTACLE_BEEP_GAP_MS + ALERT_OBSTACLE_MULTI_PAUSE_MIN_MS,
                             Silence_After_Burst(CUE_RIGHT, 3U));
    TEST_ASSERT_EQUAL_UINT32(ALERT_OBSTACLE_BEEP_GAP_MS + ALERT_OBSTACLE_MULTI_PAUSE_MIN_MS,
                             Silence_After_Burst(CUE_LEFT, 2U));
    TEST_ASSERT_EQUAL_UINT32(ALERT_OBSTACLE_BEEP_GAP_MS + ALERT_OBSTACLE_PAUSE_MIN_MS,
                             Silence_After_Burst(0U, 1U)); /* centre: unchanged */
}

/* An obstacle that clears mid-beep lets that beep play its full length (no blip);
 * a STOP tone still ends at once. */
void test_AlertManager_LastBeepFinishesWhenObstacleClears(void)
{
    AlertManager_Init();
    Alert_Output_t out;
    AlertManager_Arbitrate(ALERT_OBSTACLE_PROXIMITY, 2000U, 0U, &out);   /* beep starts at 0 ms */
    TEST_ASSERT_TRUE(out.buzzer_active);
    AlertManager_Arbitrate(ALERT_OBSTACLE_PROXIMITY, 2000U, 40U, &out);
    for (uint32_t t = 50U; t < ALERT_OBSTACLE_PULSE_MS; t += 10U)
    {
        AlertManager_Arbitrate(ALERT_NONE, 0U, t, &out);                 /* cleared at 50 ms */
        TEST_ASSERT_TRUE(out.buzzer_active);
    }
    AlertManager_Arbitrate(ALERT_NONE, 0U, ALERT_OBSTACLE_PULSE_MS, &out);
    TEST_ASSERT_FALSE(out.buzzer_active);
    AlertManager_Arbitrate(ALERT_NONE, 0U, 1000U, &out);
    TEST_ASSERT_FALSE(out.buzzer_active);                                 /* no new beeps afterwards */

    AlertManager_Init();
    AlertManager_SetObstacleCue(ALERT_CUE_STOP);
    AlertManager_Arbitrate(ALERT_OBSTACLE_PROXIMITY, 800U, 0U, &out);
    TEST_ASSERT_TRUE(out.buzzer_active);
    AlertManager_Arbitrate(ALERT_NONE, 0U, 10U, &out);
    TEST_ASSERT_FALSE(out.buzzer_active);                                 /* STOP ends immediately */
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_AlertManager_PitchMapping);
    RUN_TEST(test_AlertManager_FallPreemptsObstacle);
    RUN_TEST(test_AlertManager_IncidentDoesNotSilentlyExpire_AndPauseDoesNotClear);
    RUN_TEST(test_AlertManager_TenPulses_AndInterBurstSilence);
    RUN_TEST(test_AlertManager_TechnicalFaultHasTwoPulses);
    RUN_TEST(test_AlertManager_DeliveryFallbackIsSparseAndLowDuty);
    RUN_TEST(test_AlertManager_Synced_DefaultPlaysOnlyOnCane);
    RUN_TEST(test_AlertManager_Synced_GlassesHeadFallFallbackCueWithoutReceipt);
    RUN_TEST(test_AlertManager_Synced_CaneBuzzerFaultFailoverToGlasses);
    RUN_TEST(test_AlertManager_Synced_FaultRoutingPerNode_AndBothOnCaneOnly);
    RUN_TEST(test_AlertManager_CalibrationCuesAreDistinctOneShots);
    RUN_TEST(test_AlertManager_CalibrationCueYieldsToSafetyAlerts);
    RUN_TEST(test_AlertManager_CueBitsMatchThreatAssessment);
    RUN_TEST(test_AlertManager_ObstacleCenterCueIsOneBeep);
    RUN_TEST(test_AlertManager_ObstacleLeftIsTwoBeepsRightIsThree);
    RUN_TEST(test_AlertManager_AllBlockedIsFastFourBeepBurst);
    RUN_TEST(test_AlertManager_ObstaclePauseShrinksWithDistance);
    RUN_TEST(test_AlertManager_StopIsContinuousAtMaxPitchAtAnyDistance);
    RUN_TEST(test_AlertManager_StopStaysContinuousAndKeepsPitch);
    RUN_TEST(test_AlertManager_NoFreeLaneStretchesItsPauseWhenItKeepsPlaying);
    RUN_TEST(test_AlertManager_StopLeavesAfterDwellNotInstantly);
    RUN_TEST(test_AlertManager_ObstacleLoudnessStaysAtEarSafeDuty);
    RUN_TEST(test_AlertManager_Synced_GlassesPassesObstacleCueToPattern);
    RUN_TEST(test_AlertManager_CueFlickerShorterThanDwellKeepsPattern);
    RUN_TEST(test_AlertManager_CueChangeAppliesAfterDwell);
    RUN_TEST(test_AlertManager_StopRisingAppliesImmediatelyAndRestartsCycle);
    RUN_TEST(test_AlertManager_PitchFollowsDistanceThroughLowPass);
    RUN_TEST(test_AlertManager_BeepsNeverTruncatedWhileDistanceSweeps);
    RUN_TEST(test_AlertManager_PatternChangeNeverMergesBursts);
    RUN_TEST(test_AlertManager_MultiBeepPauseHasFloor);
    RUN_TEST(test_AlertManager_LastBeepFinishesWhenObstacleClears);
    return UNITY_END();
}
