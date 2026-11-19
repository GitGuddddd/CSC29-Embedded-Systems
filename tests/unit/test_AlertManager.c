#include "unity.h"
#include "app/AlertManager/AlertManager.h"
#include "app/AlertManager/AlertManager_Cfg.h"

void setUp(void)
{
    AlertManager_Init();
}

void tearDown(void)
{
}

void test_AlertManager_PitchMapping(void)
{
    /* 2000mm -> 1500Hz */
    TEST_ASSERT_EQUAL_UINT16(1500U, AlertManager_MapDistanceToPitch(2000U));

    /* 300mm -> 3500Hz */
    TEST_ASSERT_EQUAL_UINT16(3500U, AlertManager_MapDistanceToPitch(300U));

    /* 1150mm (midpoint) -> 2500Hz */
    TEST_ASSERT_EQUAL_UINT16(2500U, AlertManager_MapDistanceToPitch(1150U));
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

    /* 2. Glasses-initiated head-fall plays on Glasses only */
    AlertManager_Init();
    Alert_SyncInput_t in_glasses_fall = {
        .is_glasses_node = true,
        .cane_link_fresh = true,
        .cane_buzzer_fault = false,
        .delivery_fallback = false,
        .local_fall_latched = true
    };
    AlertManager_ArbitrateSynced(&in_glasses_fall, 2000U, &out_glasses);
    TEST_ASSERT_TRUE(out_glasses.buzzer_active);
    TEST_ASSERT_EQUAL(ALERT_FALL_ALARM, out_glasses.active_alert);

    Alert_SyncInput_t in_cane_peer_fall = {
        .is_glasses_node = false,
        .cane_link_fresh = true,
        .cane_buzzer_fault = false,
        .local_fall_latched = false,
        .peer_fall_active = true
    };
    AlertManager_ArbitrateSynced(&in_cane_peer_fall, 2000U, &out_cane);
    TEST_ASSERT_FALSE(out_cane.buzzer_active);

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
    TEST_ASSERT_FALSE(out.buzzer_active);

    cane.peer_fall_active = false;
    cane.cane_buzzer_fault = true;
    AlertManager_PlayCalibrationCue(ALERT_CAL_FAILED, 50U);
    AlertManager_ArbitrateSynced(&cane, 50U, &out);
    TEST_ASSERT_FALSE(out.buzzer_active);
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
    RUN_TEST(test_AlertManager_Synced_CaneBuzzerFaultFailoverToGlasses);
    RUN_TEST(test_AlertManager_Synced_FaultRoutingPerNode_AndBothOnCaneOnly);
    RUN_TEST(test_AlertManager_CalibrationCuesAreDistinctOneShots);
    RUN_TEST(test_AlertManager_CalibrationCueYieldsToSafetyAlerts);
    return UNITY_END();
}
