#include "unity.h"
#include "app/FallDetection/FallDetection.h"
#include "app/FallDetection/FallDetection_Cfg.h"

void setUp(void)
{
    FallDetection_Init(&FallConfig_Head);
}

void tearDown(void)
{
}

void test_FallDetection_InitialState_IsIdle(void)
{
    TEST_ASSERT_EQUAL_STRING("IDLE", FallDetection_GetStateName());
}

void test_FallDetection_HeadDrop_Sequence_TriggersFall(void)
{
    Imu_Sample_t s = {0};
    uint32_t now = 0U;
    bool fall_alert = false;

    /* Upright standing baseline */
    s.ax = 0.0f; s.ay = 0.0f; s.az = 1.0f;
    FallDetection_Step(&s, now);

    /* Freefall: 0g */
    now += 10U;
    s.az = 0.0f;
    FallDetection_Step(&s, now);
    TEST_ASSERT_EQUAL_STRING("FREEFALL", FallDetection_GetStateName());

    /* Maintain freefall > 50ms */
    now += 60U;
    FallDetection_Step(&s, now);

    /* Impact: 3.0g */
    now += 10U;
    s.ax = 3.0f;
    FallDetection_Step(&s, now);
    TEST_ASSERT_EQUAL_STRING("SETTLING", FallDetection_GetStateName());

    /* Settle for 800ms */
    now += 800U;
    FallDetection_Step(&s, now);
    TEST_ASSERT_EQUAL_STRING("CHECK_STILL", FallDetection_GetStateName());

    /* Still on ground, tilted 90 degrees (ax = 1.0g, az = 0.0g) */
    s.ax = 1.0f; s.ay = 0.0f; s.az = 0.0f;
    for (uint8_t i = 0U; i < 50U; i++)
    {
        now += 10U;
        fall_alert = FallDetection_Step(&s, now);
    }

    TEST_ASSERT_TRUE(fall_alert);
    TEST_ASSERT_EQUAL_STRING("IDLE", FallDetection_GetStateName());
}

void test_FallDetection_Reset_ReturnsToIdle(void)
{
    Imu_Sample_t s = {0};
    uint32_t now = 0U;

    /* Transition into FREEFALL */
    s.ax = 0.0f; s.ay = 0.0f; s.az = 0.0f;
    FallDetection_Step(&s, now);
    TEST_ASSERT_EQUAL_STRING("FREEFALL", FallDetection_GetStateName());

    /* Reset */
    FallDetection_Reset();
    TEST_ASSERT_EQUAL_STRING("IDLE", FallDetection_GetStateName());
}

/* Drives one cane drop: upright, free fall, impact, settle, optional rolling, then rest. */
static bool Run_CaneDrop(uint32_t roll_ms, float rest_ax, float rest_az)
{
    Imu_Sample_t s = {0};
    uint32_t now = 0U;
    bool fall = false;

    FallDetection_Init(&FallConfig_Cane);
    for (uint32_t i = 0U; i < 300U; i++) { s.az = 1.0f; now += 10U; fall |= FallDetection_Step(&s, now); }
    for (uint32_t i = 0U; i < 30U; i++)  { s.az = 0.05f; now += 10U; fall |= FallDetection_Step(&s, now); }
    s.ax = 2.0f; s.az = 2.0f; s.gx = 200.0f; now += 10U; fall |= FallDetection_Step(&s, now);
    for (uint32_t i = 0U; i < 59U; i++)  { s.ax = 0.9f; s.az = 0.3f; s.gx = 50.0f; now += 10U; fall |= FallDetection_Step(&s, now); }
    for (uint32_t i = 0U; i < (roll_ms / 10U); i++)
    {
        s.ax = 1.0f; s.az = 0.05f; s.gx = 60.0f; now += 10U; fall |= FallDetection_Step(&s, now);
    }
    for (uint32_t i = 0U; i < 300U; i++)
    {
        s.ax = rest_ax; s.az = rest_az; s.gx = 0.0f; now += 10U; fall |= FallDetection_Step(&s, now);
    }
    return fall;
}

void test_FallDetection_CaneRollsAfterImpact_StillConfirmsFall(void)
{
    TEST_ASSERT_TRUE(Run_CaneDrop(500U, 1.0f, 0.0f));
    TEST_ASSERT_EQUAL_STRING("IDLE", FallDetection_GetStateName());
}

void test_FallDetection_CaneLandsUpright_IsNotAFall(void)
{
    TEST_ASSERT_FALSE(Run_CaneDrop(500U, 0.0f, 1.0f));
}

void test_FallDetection_NeverComesToRest_TimesOutWithoutFall(void)
{
    /* Motion lasting past the timeout must abandon the incident. */
    Imu_Sample_t s = {0};
    uint32_t now = 0U;
    FallDetection_Init(&FallConfig_Cane);
    s.ax = 3.0f; now += 10U; (void)FallDetection_Step(&s, now);          /* impact */
    TEST_ASSERT_EQUAL(FALL_STATE_SETTLING, FallDetection_GetState());
    bool fall = false;
    for (uint32_t i = 0U; i < ((FALL_STILL_TIMEOUT_MS + 1000U) / 10U); i++)
    {
        s.ax = 1.0f; s.az = 0.0f; s.gx = 90.0f; now += 10U; fall |= FallDetection_Step(&s, now);
    }
    TEST_ASSERT_FALSE(fall);
    TEST_ASSERT_EQUAL(FALL_STATE_IDLE, FallDetection_GetState());
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_FallDetection_InitialState_IsIdle);
    RUN_TEST(test_FallDetection_HeadDrop_Sequence_TriggersFall);
    RUN_TEST(test_FallDetection_Reset_ReturnsToIdle);
    RUN_TEST(test_FallDetection_CaneRollsAfterImpact_StillConfirmsFall);
    RUN_TEST(test_FallDetection_CaneLandsUpright_IsNotAFall);
    RUN_TEST(test_FallDetection_NeverComesToRest_TimesOutWithoutFall);
    return UNITY_END();
}
