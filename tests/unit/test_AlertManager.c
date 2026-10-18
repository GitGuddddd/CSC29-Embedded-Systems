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

void test_AlertManager_30s_MutedTransition_And_ExplicitReset(void)
{
    Alert_Output_t out;

    /* Fall triggers at t = 1,000 ms */
    AlertManager_Arbitrate(ALERT_FALL_ALARM, 0U, 1000U, &out);
    TEST_ASSERT_EQUAL(ALERT_FALL_ALARM, out.active_alert);
    TEST_ASSERT_TRUE(out.buzzer_active);
    TEST_ASSERT_TRUE(out.fall_latched);

    /* At t = 30,000 ms (29s later): Still sounding siren */
    AlertManager_Arbitrate(ALERT_FALL_ALARM, 0U, 30000U, &out);
    TEST_ASSERT_EQUAL(ALERT_FALL_ALARM, out.active_alert);
    TEST_ASSERT_TRUE(out.buzzer_active);

    /* At t = 31,500 ms (> 30s elapsed): Mutes buzzer, but stays latched (SYS_FALL_MUTED) */
    AlertManager_Arbitrate(ALERT_NONE, 0U, 31500U, &out);
    TEST_ASSERT_EQUAL(ALERT_FALL_MUTED, out.active_alert);
    TEST_ASSERT_FALSE(out.buzzer_active);
    TEST_ASSERT_TRUE(out.fall_latched);

    /* Explicit reset occurs via user button press / upright 3s hold */
    AlertManager_AcknowledgeReset();
    AlertManager_Arbitrate(ALERT_NONE, 0U, 32000U, &out);
    TEST_ASSERT_EQUAL(ALERT_NONE, out.active_alert);
    TEST_ASSERT_FALSE(out.fall_latched);
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_AlertManager_PitchMapping);
    RUN_TEST(test_AlertManager_FallPreemptsObstacle);
    RUN_TEST(test_AlertManager_30s_MutedTransition_And_ExplicitReset);
    return UNITY_END();
}
