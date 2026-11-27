#include "unity.h"
#include "drivers/TouchSensor/TouchSensor.h"

void setUp(void) { TouchSensor_Init(NULL); }
void tearDown(void) { }

void test_Debounce_UsesElapsedTime_NotNumberOfRtosIterations(void)
{
    Grip_State_t state;
    TEST_ASSERT_FALSE(TouchSensor_Update(NULL, true, 100U, &state));
    TEST_ASSERT_FALSE(TouchSensor_Update(NULL, true, 110U, &state));
    TEST_ASSERT_TRUE(TouchSensor_Update(NULL, true, 120U, &state));
    TEST_ASSERT_EQUAL(GRIP_STATE_HELD, state);
    TEST_ASSERT_FALSE(TouchSensor_Update(NULL, true, 60000U, &state));
    TEST_ASSERT_EQUAL(GRIP_STATE_HELD, state);
}

void test_BounceCannotManufactureReleaseBeforeHold(void)
{
    Grip_State_t state;
    (void)TouchSensor_Update(NULL, true, 100U, &state);
    (void)TouchSensor_Update(NULL, true, 120U, &state);
    (void)TouchSensor_Update(NULL, false, 130U, &state);
    (void)TouchSensor_Update(NULL, true, 140U, &state);
    TEST_ASSERT_FALSE(TouchSensor_Update(NULL, true, 160U, &state));
    TEST_ASSERT_EQUAL(GRIP_STATE_HELD, state);
    (void)TouchSensor_Update(NULL, false, 170U, &state);
    TEST_ASSERT_TRUE(TouchSensor_Update(NULL, false, 190U, &state));
    TEST_ASSERT_EQUAL(GRIP_STATE_RELEASED, state);
}

void test_Debounce_AcrossTimestampWrap(void)
{
    Grip_State_t state;
    (void)TouchSensor_Update(NULL, true, UINT32_MAX - 9U, &state);
    TEST_ASSERT_TRUE(TouchSensor_Update(NULL, true, 10U, &state));
    TEST_ASSERT_EQUAL(GRIP_STATE_HELD, state);
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_Debounce_UsesElapsedTime_NotNumberOfRtosIterations);
    RUN_TEST(test_BounceCannotManufactureReleaseBeforeHold);
    RUN_TEST(test_Debounce_AcrossTimestampWrap);
    return UNITY_END();
}
