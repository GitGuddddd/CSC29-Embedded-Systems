#include "unity.h"
#include "services/IncidentRetention/IncidentRetention.h"

void setUp(void)
{
    IncidentRetention_TestPowerCycle();
}

void tearDown(void)
{
}

static void SimulateWatchdogRestart(IncidentRetention_Role_t role)
{
    IncidentRetention_TestSetWatchdogReset(true);
    IncidentRetention_TestRestartRuntime();
    IncidentRetention_Init(role);
}

void test_ActiveIncidentSurvivesEnabledWatchdogReset(void)
{
    IncidentRetention_Record_t restored;
    IncidentRetention_Init(INCIDENT_RETENTION_ROLE_GLASSES);
    IncidentRetention_Store(true, 0xF1020304U);
    SimulateWatchdogRestart(INCIDENT_RETENTION_ROLE_GLASSES);
    TEST_ASSERT_TRUE(IncidentRetention_Restore(&restored));
    TEST_ASSERT_EQUAL_HEX32(0xF1020304U, restored.event_id);

    /* The record remains valid across a second watchdog reset before clearance. */
    SimulateWatchdogRestart(INCIDENT_RETENTION_ROLE_GLASSES);
    TEST_ASSERT_TRUE(IncidentRetention_Restore(&restored));
}

void test_ClearPreventsIncidentRevival(void)
{
    IncidentRetention_Record_t restored;
    IncidentRetention_Init(INCIDENT_RETENTION_ROLE_CANE);
    IncidentRetention_Store(true, 7U);
    SimulateWatchdogRestart(INCIDENT_RETENTION_ROLE_CANE);
    TEST_ASSERT_TRUE(IncidentRetention_Restore(&restored));
    IncidentRetention_Store(false, 7U);
    SimulateWatchdogRestart(INCIDENT_RETENTION_ROLE_CANE);
    TEST_ASSERT_FALSE(IncidentRetention_Restore(&restored));
}

void test_NonWatchdogResetDiscardsRetainedIncident(void)
{
    IncidentRetention_Record_t restored;
    IncidentRetention_Init(INCIDENT_RETENTION_ROLE_CANE);
    IncidentRetention_Store(true, 11U);
    IncidentRetention_TestSetWatchdogReset(false);
    IncidentRetention_TestRestartRuntime();
    IncidentRetention_Init(INCIDENT_RETENTION_ROLE_CANE);
    TEST_ASSERT_FALSE(IncidentRetention_Restore(&restored));
    TEST_ASSERT_EQUAL_HEX32(0U, IncidentRetention_TestReadWord(0U));
}

void test_WrongFirmwareRoleCannotRestoreRecord(void)
{
    IncidentRetention_Record_t restored;
    IncidentRetention_Init(INCIDENT_RETENTION_ROLE_CANE);
    IncidentRetention_Store(true, 12U);
    SimulateWatchdogRestart(INCIDENT_RETENTION_ROLE_GLASSES);
    TEST_ASSERT_FALSE(IncidentRetention_Restore(&restored));
}

void test_CorruptOrTornPayloadRestoresConservativeActiveIncident(void)
{
    IncidentRetention_Record_t restored;
    IncidentRetention_Init(INCIDENT_RETENTION_ROLE_GLASSES);
    IncidentRetention_Store(true, 0x12345678U);
    IncidentRetention_TestCorruptWord(2U, 0U);
    SimulateWatchdogRestart(INCIDENT_RETENTION_ROLE_GLASSES);
    TEST_ASSERT_TRUE(IncidentRetention_Restore(&restored));
    TEST_ASSERT_EQUAL_UINT32(0U, restored.event_id);

    IncidentRetention_Store(true, 99U);
    IncidentRetention_TestCorruptWord(3U, 0U); /* Payload update never completed. */
    SimulateWatchdogRestart(INCIDENT_RETENTION_ROLE_GLASSES);
    TEST_ASSERT_TRUE(IncidentRetention_Restore(&restored));
    TEST_ASSERT_EQUAL_UINT32(0U, restored.event_id);

    IncidentRetention_Store(false, 0U); /* An absent active commit remains clear. */
    SimulateWatchdogRestart(INCIDENT_RETENTION_ROLE_GLASSES);
    TEST_ASSERT_FALSE(IncidentRetention_Restore(&restored));
}

void test_EventCounterWrapIsRetainedWithoutClearingIncident(void)
{
    IncidentRetention_Record_t restored;
    IncidentRetention_Init(INCIDENT_RETENTION_ROLE_CANE);
    IncidentRetention_Store(true, UINT32_MAX);
    SimulateWatchdogRestart(INCIDENT_RETENTION_ROLE_CANE);
    TEST_ASSERT_TRUE(IncidentRetention_Restore(&restored));
    TEST_ASSERT_EQUAL_UINT32(UINT32_MAX, restored.event_id);

    IncidentRetention_Store(true, 0U);
    TEST_ASSERT_EQUAL_HEX32(UINT32_MAX, IncidentRetention_TestReadWord(2U));
    SimulateWatchdogRestart(INCIDENT_RETENTION_ROLE_CANE);
    TEST_ASSERT_TRUE(IncidentRetention_Restore(&restored));
    TEST_ASSERT_EQUAL_UINT32(0U, restored.event_id);
}

void test_InvalidRoleCannotCreateOrRestoreRecord(void)
{
    IncidentRetention_Record_t restored;
    IncidentRetention_Init((IncidentRetention_Role_t)0);
    IncidentRetention_Store(true, 42U);
    IncidentRetention_TestSetWatchdogReset(true);
    IncidentRetention_TestRestartRuntime();
    IncidentRetention_Init((IncidentRetention_Role_t)0);
    TEST_ASSERT_FALSE(IncidentRetention_Restore(&restored));
    TEST_ASSERT_EQUAL_HEX32(0U, IncidentRetention_TestReadWord(0U));
}

void test_NullAndUninitializedAccessFailClosed(void)
{
    IncidentRetention_Record_t restored;
    TEST_ASSERT_FALSE(IncidentRetention_Restore(&restored));
    IncidentRetention_Store(true, 1U);
    IncidentRetention_Init(INCIDENT_RETENTION_ROLE_CANE);
    TEST_ASSERT_FALSE(IncidentRetention_Restore(NULL));
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_ActiveIncidentSurvivesEnabledWatchdogReset);
    RUN_TEST(test_ClearPreventsIncidentRevival);
    RUN_TEST(test_NonWatchdogResetDiscardsRetainedIncident);
    RUN_TEST(test_WrongFirmwareRoleCannotRestoreRecord);
    RUN_TEST(test_CorruptOrTornPayloadRestoresConservativeActiveIncident);
    RUN_TEST(test_EventCounterWrapIsRetainedWithoutClearingIncident);
    RUN_TEST(test_InvalidRoleCannotCreateOrRestoreRecord);
    RUN_TEST(test_NullAndUninitializedAccessFailClosed);
    return UNITY_END();
}
