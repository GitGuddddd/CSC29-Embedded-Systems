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

void test_ThreatAssessment_TtcIsBoundedInWireUnits(void)
{
    Radar_Target_t target = {.is_valid = true, .distance_mm = 2000U, .speed_cms = 1};
    Threat_Result_t result;
    ThreatAssessment_ProcessTargets(&target, 1U, GRIP_STATE_HELD, &result);
    TEST_ASSERT_TRUE(result.is_threat_active); /* Unavailable TTC does not suppress the obstacle. */
    TEST_ASSERT_EQUAL_UINT16(BLE_TTC_NO_THREAT, result.ttc_20ms);

    target.distance_mm = 4094U;
    target.speed_cms = 5;
    ThreatAssessment_ProcessTargets(&target, 1U, GRIP_STATE_HELD, &result);
    TEST_ASSERT_EQUAL_UINT16(BLE_TTC_MAX_TICKS, result.ttc_20ms);
    TEST_ASSERT_EQUAL_UINT32(81880U, (uint32_t)result.ttc_20ms * BLE_TTC_STEP_MS);

    target.distance_mm = 819U;
    target.speed_cms = 1;
    ThreatAssessment_ProcessTargets(&target, 1U, GRIP_STATE_HELD, &result);
    TEST_ASSERT_EQUAL_UINT16(BLE_TTC_NO_THREAT, result.ttc_20ms); /* 4095 is reserved. */
}

void test_ThreatAssessment_TtcMatchesWideArithmeticAcrossBoundaryInputs(void)
{
    static const uint16_t distances[] = {1U, 819U, 1000U, 2000U, 4094U, 4095U, 6000U, 65534U};
    static const int16_t speeds[] = {0, 1, 2, 5, 10, 100, 32767};
    for (uint32_t d = 0U; d < sizeof(distances) / sizeof(distances[0]); ++d)
    {
        for (uint32_t s = 0U; s < sizeof(speeds) / sizeof(speeds[0]); ++s)
        {
            Radar_Target_t target = {.is_valid = true, .distance_mm = distances[d], .speed_cms = speeds[s]};
            Threat_Result_t result;
            ThreatAssessment_ProcessTargets(&target, 1U, GRIP_STATE_HELD, &result);
            uint16_t expected = BLE_TTC_NO_THREAT;
            if (speeds[s] > 0)
            {
                uint64_t milliseconds = ((uint64_t)distances[d] * 100U) / (uint16_t)speeds[s];
                uint64_t ticks = milliseconds / BLE_TTC_STEP_MS;
                if (ticks <= BLE_TTC_MAX_TICKS) expected = (uint16_t)ticks;
            }
            TEST_ASSERT_EQUAL_UINT16(expected, result.ttc_20ms);
        }
    }
}

void test_ThreatAssessment_ClearUsesUnavailableSentinels(void)
{
    Threat_Result_t result;
    ThreatAssessment_ProcessTargets(NULL, 0U, GRIP_STATE_HELD, &result);
    TEST_ASSERT_FALSE(result.is_threat_active);
    TEST_ASSERT_EQUAL_UINT16(UINT16_MAX, result.nearest_distance_mm);
    TEST_ASSERT_EQUAL_UINT16(BLE_TTC_NO_THREAT, result.ttc_20ms);
    Radar_Target_t target = {.is_valid = true, .distance_mm = 1500U, .speed_cms = 10};
    ThreatAssessment_ProcessTargets(&target, 1U, GRIP_STATE_RELEASED, &result);
    TEST_ASSERT_FALSE(result.is_threat_active);
    TEST_ASSERT_EQUAL_UINT16(BLE_TTC_NO_THREAT, result.ttc_20ms);
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_ThreatAssessment_IgnoresDepartingTargets);
    RUN_TEST(test_ThreatAssessment_Hysteresis_EnterAndClear);
    RUN_TEST(test_ThreatAssessment_FR6_GripGating);
    RUN_TEST(test_ThreatAssessment_TtcIsBoundedInWireUnits);
    RUN_TEST(test_ThreatAssessment_TtcMatchesWideArithmeticAcrossBoundaryInputs);
    RUN_TEST(test_ThreatAssessment_ClearUsesUnavailableSentinels);
    return UNITY_END();
}
