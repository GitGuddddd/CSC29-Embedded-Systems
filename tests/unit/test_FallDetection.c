#include "unity.h"
#include "app/FallDetection/FallDetection.h"

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

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_FallDetection_InitialState_IsIdle);
    RUN_TEST(test_FallDetection_HeadDrop_Sequence_TriggersFall);
    return UNITY_END();
}
