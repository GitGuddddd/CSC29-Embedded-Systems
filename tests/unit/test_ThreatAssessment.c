#include "unity.h"
#include "app/ThreatAssessment/ThreatAssessment.h"
#include "app/ThreatAssessment/ThreatAssessment_Cfg.h"

void setUp(void)
{
    ThreatAssessment_Init();
}

void tearDown(void)
{
}

void test_ThreatAssessment_IgnoresDepartingTargets(void)
{
    Radar_Target_t targets[1];
    targets[0].is_valid = true;
    targets[0].distance_mm = 1500U;
    targets[0].speed_cms = -20; /* Departing fast (-20 < -10) */

    Threat_Result_t res;
    ThreatAssessment_ProcessTargets(targets, 1U, GRIP_STATE_HELD, &res);

    TEST_ASSERT_FALSE(res.is_threat_active);
}

void test_ThreatAssessment_Hysteresis_EnterAndClear(void)
{
    Radar_Target_t targets[1];
    targets[0].is_valid = true;
    targets[0].speed_cms = 15; /* Approaching */

    Threat_Result_t res;

    /* 1. At 2050mm (Outside enter threshold of 2000mm) -> Inactive */
    targets[0].distance_mm = 2050U;
    ThreatAssessment_ProcessTargets(targets, 1U, GRIP_STATE_HELD, &res);
    TEST_ASSERT_FALSE(res.is_threat_active);

    /* 2. Moves closer: 1950mm (< 2000mm) -> Active */
    targets[0].distance_mm = 1950U;
    ThreatAssessment_ProcessTargets(targets, 1U, GRIP_STATE_HELD, &res);
    TEST_ASSERT_TRUE(res.is_threat_active);

    /* 3. Moves back to 2050mm (In hysteresis zone: > 2000mm but <= 2100mm) -> Remains Active! */
    targets[0].distance_mm = 2050U;
    ThreatAssessment_ProcessTargets(targets, 1U, GRIP_STATE_HELD, &res);
    TEST_ASSERT_TRUE(res.is_threat_active);

    /* 4. Moves past clear threshold: 2150mm (> 2100mm) -> Inactive */
    targets[0].distance_mm = 2150U;
    ThreatAssessment_ProcessTargets(targets, 1U, GRIP_STATE_HELD, &res);
    TEST_ASSERT_FALSE(res.is_threat_active);
}

void test_ThreatAssessment_FR6_GripGating(void)
{
    Radar_Target_t targets[1];
    targets[0].is_valid = true;
    targets[0].distance_mm = 1200U;
    targets[0].speed_cms = 10;

    Threat_Result_t res;

    /* Active while held */
    ThreatAssessment_ProcessTargets(targets, 1U, GRIP_STATE_HELD, &res);
    TEST_ASSERT_TRUE(res.is_threat_active);

    /* Suppressed when explicitly confirmed RELEASED */
    ThreatAssessment_ProcessTargets(targets, 1U, GRIP_STATE_RELEASED, &res);
    TEST_ASSERT_FALSE(res.is_threat_active);

    /* Fail-safe active when UNKNOWN (link dropout) */
    ThreatAssessment_ProcessTargets(targets, 1U, GRIP_STATE_UNKNOWN, &res);
    TEST_ASSERT_TRUE(res.is_threat_active);
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_ThreatAssessment_IgnoresDepartingTargets);
    RUN_TEST(test_ThreatAssessment_Hysteresis_EnterAndClear);
    RUN_TEST(test_ThreatAssessment_FR6_GripGating);
    return UNITY_END();
}
