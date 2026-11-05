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

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_AlertManager_PitchMapping);
    RUN_TEST(test_AlertManager_FallPreemptsObstacle);
    RUN_TEST(test_AlertManager_IncidentDoesNotSilentlyExpire_AndPauseDoesNotClear);
    RUN_TEST(test_AlertManager_TenPulses_AndInterBurstSilence);
    RUN_TEST(test_AlertManager_TechnicalFaultHasTwoPulses);
    RUN_TEST(test_AlertManager_DeliveryFallbackIsSparseAndLowDuty);
    return UNITY_END();
}
