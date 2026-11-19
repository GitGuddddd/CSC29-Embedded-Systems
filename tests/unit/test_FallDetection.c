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

void test_FallDetection_HeadDirectImpactWithoutFreefall_IsIgnored(void)
{
    Imu_Sample_t s = {0};
    uint32_t now = 0U;

    /* Upright standing baseline */
    s.ax = 0.0f;
    s.ay = 0.0f;
    s.az = 1.0f;
    (void)FallDetection_Step(&s, now);
    TEST_ASSERT_EQUAL_STRING("IDLE", FallDetection_GetStateName());

    /* Direct impact spike: 3.0g without prior free-fall */
    now += 10U;
    s.ax = 3.0f;
    s.ay = 0.0f;
    s.az = 0.0f;
    bool fall_alert = FallDetection_Step(&s, now);

    /* Head-mounted profile requires free-fall before impact; stays in IDLE */
    TEST_ASSERT_FALSE(fall_alert);
    TEST_ASSERT_EQUAL(FALL_STATE_IDLE, FallDetection_GetState());
    TEST_ASSERT_EQUAL_STRING("IDLE", FallDetection_GetStateName());
}

void test_FallDetection_DynamicMotionDoesNotCorruptGravityReference(void)
{
    Imu_Sample_t s = {0};
    uint32_t now = 0U;
    bool fall_alert = false;

    /* Upright standing baseline */
    s.ax = 0.0f;
    s.ay = 0.0f;
    s.az = 1.0f;
    (void)FallDetection_Step(&s, now);
    TEST_ASSERT_EQUAL_STRING("IDLE", FallDetection_GetStateName());

    /* Dynamic motion in IDLE: sensor experiences lateral acceleration (ax = 1.0g, az = 0.0g)
     * accompanied by angular rotation (gx = 60.0 dps > 30.0 dps stillness threshold).
     * Since the sensor is not still, s_ref must remain upright (s_ref_z = 1.0f, s_ref_x = 0.0f).
     */
    for (uint32_t i = 0U; i < 200U; i++)
    {
        now += 10U;
        s.ax = 1.0f;
        s.ay = 0.0f;
        s.az = 0.0f;
        s.gx = 60.0f;
        s.gy = 0.0f;
        s.gz = 0.0f;
        (void)FallDetection_Step(&s, now);
        TEST_ASSERT_EQUAL_STRING("IDLE", FallDetection_GetStateName());
    }

    /* Subsequent genuine fall sequence:
     * Freefall: 0g for 70 ms
     */
    now += 10U;
    s.ax = 0.0f;
    s.ay = 0.0f;
    s.az = 0.0f;
    s.gx = 0.0f;
    (void)FallDetection_Step(&s, now);
    TEST_ASSERT_EQUAL_STRING("FREEFALL", FallDetection_GetStateName());

    now += 60U;
    (void)FallDetection_Step(&s, now);

    /* Impact: 3.0g */
    now += 10U;
    s.ax = 3.0f;
    (void)FallDetection_Step(&s, now);
    TEST_ASSERT_EQUAL_STRING("SETTLING", FallDetection_GetStateName());

    /* Settling for 800 ms */
    now += 800U;
    (void)FallDetection_Step(&s, now);
    TEST_ASSERT_EQUAL_STRING("CHECK_STILL", FallDetection_GetStateName());

    /* Rest on ground, tilted 90 degrees (ax = 1.0g, az = 0.0g) */
    s.ax = 1.0f;
    s.ay = 0.0f;
    s.az = 0.0f;
    for (uint8_t i = 0U; i < 50U; i++)
    {
        now += 10U;
        fall_alert = FallDetection_Step(&s, now);
    }

    /* If s_ref had shifted to [1.0, 0.0, 0.0] during dynamic motion, tilt angle would
     * be ~0 deg and fall_alert would be false. With s_ref uncorrupted, fall is confirmed. */
    TEST_ASSERT_TRUE(fall_alert);
    TEST_ASSERT_EQUAL_STRING("IDLE", FallDetection_GetStateName());
}

/* Feeds one sample every 10 ms for duration_ms; returns true if any step reported a fall. */
static bool Feed(Imu_Sample_t * const p_s, uint32_t * const p_now, uint32_t duration_ms)
{
    bool fall = false;

    for (uint32_t i = 0U; i < (duration_ms / 10U); i++)
    {
        *p_now += 10U;
        fall |= FallDetection_Step(p_s, *p_now);
    }
    return fall;
}

/* Sets the accelerometer (g) and gyro x-axis (dps) of a sample. */
static void Set(Imu_Sample_t * const p_s, float ax, float ay, float az, float gx)
{
    p_s->ax = ax;
    p_s->ay = ay;
    p_s->az = az;
    p_s->gx = gx;
    p_s->gy = 0.0f;
    p_s->gz = 0.0f;
}

/* Fast head move: free fall, impact, then settling motion; ends upright and still. */
static bool Run_FastMoveEndingUpright(Imu_Sample_t * const p_s, uint32_t * const p_now)
{
    bool fall = false;

    Set(p_s, 0.0f, 0.0f, 0.3f, 80.0f);           /* 0.3 g dip for 70 ms */
    fall |= Feed(p_s, p_now, 70U);
    Set(p_s, 0.0f, 0.0f, 2.8f, 120.0f);          /* 2.8 g impact */
    fall |= Feed(p_s, p_now, 10U);
    Set(p_s, 0.0f, 0.2f, 1.1f, 60.0f);           /* moving while settling */
    fall |= Feed(p_s, p_now, 800U);
    Set(p_s, 0.0f, 0.0f, 1.0f, 0.0f);            /* upright and still */
    fall |= Feed(p_s, p_now, 2000U);
    return fall;
}

/* The false alarm from the lean/bend trial log: a held bend, then a fast return
 * upright that dips into free fall. The reference must not adopt the bent pose.
 */
void test_FallDetection_HeadHeldBendThenFastReturn_IsNotAFall(void)
{
    Imu_Sample_t s = {0};
    uint32_t now = 0U;
    bool fall = false;

    Set(&s, 0.0f, 0.0f, 1.0f, 0.0f);             /* worn upright, 40 s */
    fall |= Feed(&s, &now, 40000U);
    Set(&s, 0.0f, -0.9f, 0.436f, 0.0f);          /* held bend (~64 deg), 6 s */
    fall |= Feed(&s, &now, 6000U);
    fall |= Run_FastMoveEndingUpright(&s, &now);

    TEST_ASSERT_FALSE(fall);
    TEST_ASSERT_EQUAL(FALL_STATE_IDLE, FallDetection_GetState());
}

/* Glasses boot lying on their side, then wake (Reset) and are worn upright.
 * Reset must restart fast reference learning, or the upright head still looks
 * tilted against the side-lying reference and a fast move reports a fall.
 */
void test_FallDetection_HeadResetRelearnsWornPoseQuickly(void)
{
    Imu_Sample_t s = {0};
    uint32_t now = 0U;
    bool fall = false;

    Set(&s, 1.0f, 0.0f, 0.0f, 0.0f);             /* on the table, 40 s */
    fall |= Feed(&s, &now, 40000U);
    FallDetection_Reset();                        /* wake from standby */
    Set(&s, 0.0f, 0.0f, 1.0f, 0.0f);             /* worn upright, 5 s */
    fall |= Feed(&s, &now, 5000U);
    fall |= Run_FastMoveEndingUpright(&s, &now);

    TEST_ASSERT_FALSE(fall);
}

/* Cane switched on lying flat, then woken (Reset) and held upright.
 * A drop that ends flat must still be detected after the cane relearns upright.
 */
void test_FallDetection_CaneResetRelearnsHeldPoseQuickly(void)
{
    Imu_Sample_t s = {0};
    uint32_t now = 0U;
    bool fall = false;

    FallDetection_Init(&FallConfig_Cane);
    Set(&s, 1.0f, 0.0f, 0.0f, 0.0f);             /* flat at switch-on, 40 s */
    (void)Feed(&s, &now, 40000U);
    FallDetection_Reset();                        /* IMU wakes after parking */
    Set(&s, 0.0f, 0.0f, 1.0f, 0.0f);             /* held upright, still, 5 s */
    (void)Feed(&s, &now, 5000U);

    Set(&s, 0.0f, 0.0f, 0.05f, 50.0f);           /* free fall, 300 ms */
    fall |= Feed(&s, &now, 300U);
    Set(&s, 2.0f, 0.0f, 2.0f, 200.0f);           /* impact */
    fall |= Feed(&s, &now, 10U);
    Set(&s, 0.9f, 0.0f, 0.3f, 50.0f);            /* settling */
    fall |= Feed(&s, &now, 600U);
    Set(&s, 1.0f, 0.0f, 0.0f, 0.0f);             /* comes to rest flat */
    fall |= Feed(&s, &now, 2000U);

    TEST_ASSERT_TRUE(fall);
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
    RUN_TEST(test_FallDetection_HeadDirectImpactWithoutFreefall_IsIgnored);
    RUN_TEST(test_FallDetection_DynamicMotionDoesNotCorruptGravityReference);
    RUN_TEST(test_FallDetection_HeadHeldBendThenFastReturn_IsNotAFall);
    RUN_TEST(test_FallDetection_HeadResetRelearnsWornPoseQuickly);
    RUN_TEST(test_FallDetection_CaneResetRelearnsHeldPoseQuickly);
    return UNITY_END();
}
