#include "unity.h"
#include "app/ThreatAssessment/ThreatAssessment.h"
#include "app/AlertManager/AlertManager.h"
#include "rte/Rte.h"

void setUp(void)
{
    ThreatAssessment_Init();
    AlertManager_Init();
}

void tearDown(void)
{
}

void test_Integration_ThreatToAlertPipeline(void)
{
    /* Simulate radar target entering obstacle zone */
    Radar_Target_t targets[1];
    targets[0].is_valid = true;
    targets[0].distance_mm = 1150U; /* Middle distance */
    targets[0].speed_cms = 20;

    Threat_Result_t threat;
    ThreatAssessment_ProcessTargets(targets, 1U, GRIP_STATE_HELD, &threat);

    TEST_ASSERT_TRUE(threat.is_threat_active);
    TEST_ASSERT_EQUAL_UINT16(1150U, threat.nearest_distance_mm);

    /* Feed threat into AlertManager */
    Alert_Output_t alert_out;
    AlertManager_Arbitrate(ALERT_OBSTACLE_PROXIMITY, threat.nearest_distance_mm, 1000U, &alert_out);

    TEST_ASSERT_EQUAL(ALERT_OBSTACLE_PROXIMITY, alert_out.active_alert);
    TEST_ASSERT_TRUE(alert_out.buzzer_active);
    TEST_ASSERT_EQUAL_UINT16(2500U, alert_out.generated_freq_hz);
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_Integration_ThreatToAlertPipeline);
    return UNITY_END();
}
