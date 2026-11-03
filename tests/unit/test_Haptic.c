#include "unity.h"
#include "drivers/Haptic/Haptic.h"
#include "drivers/Haptic/Haptic_Cfg.h"

/*
 * \file test_Haptic.c
 * \brief Cane motor arbitration, grip gating and cue distinctness
 */

#define STEP_MS (10U)

typedef struct
{
    uint32_t pulses;
    uint32_t longest_pulse_ms;
} Pulse_Stats_t;

void setUp(void)
{
    Haptic_Init(&Haptic_Config_Cane);
    Haptic_SetGripHeld(true);
}

void tearDown(void)
{
}

/* Steps the player from start_ms for window_ms and counts motor pulses */
static Pulse_Stats_t Measure(uint32_t start_ms, uint32_t window_ms)
{
    Pulse_Stats_t s = { 0U, 0U };
    uint32_t on_ms = 0U;
    bool was_on = false;

    for (uint32_t t = start_ms; t < (start_ms + window_ms); t += STEP_MS)
    {
        Haptic_Step(t);
        bool on = Haptic_IsMotorOn();
        if (on)
        {
            on_ms += STEP_MS;
            if (!was_on)
            {
                s.pulses++;
            }
        }
        else
        {
            if (on_ms > s.longest_pulse_ms)
            {
                s.longest_pulse_ms = on_ms;
            }
            on_ms = 0U;
        }
        was_on = on;
    }
    if (on_ms > s.longest_pulse_ms)
    {
        s.longest_pulse_ms = on_ms;
    }
    return s;
}

static Pulse_Stats_t Measure_One_Period(Alert_Type_t cue, uint32_t period_ms)
{
    Haptic_Init(&Haptic_Config_Cane);
    Haptic_SetGripHeld(true);
    Haptic_Request(HAPTIC_SRC_GROUND, cue);
    return Measure(0U, period_ms);
}

/* Version C gap: every cue must differ in pulse count or pulse length */
void test_Haptic_CuesAreDistinct(void)
{
    Pulse_Stats_t down = Measure_One_Period(ALERT_CLIFF_WARNING, HAPTIC_CFG_CLIFF_PERIOD_MS);
    Pulse_Stats_t up = Measure_One_Period(ALERT_CLIFF_STEP_UP, HAPTIC_CFG_STEPUP_PERIOD_MS);
    Pulse_Stats_t caution = Measure_One_Period(ALERT_SENSOR_FAULT, HAPTIC_CFG_CAUTION_PERIOD_MS);
    Pulse_Stats_t link = Measure_One_Period(ALERT_LINK_LOSS, HAPTIC_CFG_LINKLOSS_PERIOD_MS);
    Pulse_Stats_t prompt = Measure_One_Period(ALERT_CALIBRATE_PROMPT, HAPTIC_CFG_CALPROMPT_PERIOD_MS);

    TEST_ASSERT_EQUAL_UINT32(3U, down.pulses);
    TEST_ASSERT_EQUAL_UINT32(2U, up.pulses);
    TEST_ASSERT_EQUAL_UINT32(1U, caution.pulses);
    TEST_ASSERT_EQUAL_UINT32(2U, link.pulses);
    TEST_ASSERT_EQUAL_UINT32(4U, prompt.pulses);

    /* STEP_UP and LINK_LOSS share a count, so they must differ in length */
    TEST_ASSERT_EQUAL_UINT32(HAPTIC_CFG_STEPUP_PULSE_MS, up.longest_pulse_ms);
    TEST_ASSERT_EQUAL_UINT32(HAPTIC_CFG_LINKLOSS_PULSE_MS, link.longest_pulse_ms);
}

/* Tribunal 1 (12:50PM, no change): step warnings pre-empt the link-loss cue, which resumes afterwards */
void test_Haptic_StepWarningPreemptsLinkLoss(void)
{
    Haptic_Request(HAPTIC_SRC_LINK, ALERT_LINK_LOSS);
    Haptic_Step(0U);
    TEST_ASSERT_EQUAL(ALERT_LINK_LOSS, Haptic_GetActivePattern());

    Haptic_Request(HAPTIC_SRC_GROUND, ALERT_CLIFF_WARNING);
    Haptic_Step(100U);
    TEST_ASSERT_EQUAL(ALERT_CLIFF_WARNING, Haptic_GetActivePattern());
    TEST_ASSERT_TRUE(Haptic_IsMotorOn());

    Haptic_Request(HAPTIC_SRC_GROUND, ALERT_CLIFF_STEP_UP);
    Haptic_Step(200U);
    TEST_ASSERT_EQUAL(ALERT_CLIFF_STEP_UP, Haptic_GetActivePattern());

    Haptic_Request(HAPTIC_SRC_GROUND, ALERT_NONE);
    Haptic_Step(300U);
    TEST_ASSERT_EQUAL(ALERT_LINK_LOSS, Haptic_GetActivePattern());
}

/* Regression: level ground used to call Haptic_Abort() and chop the link-loss cue */
void test_Haptic_LevelGroundDoesNotCutLinkLoss(void)
{
    Haptic_Request(HAPTIC_SRC_LINK, ALERT_LINK_LOSS);

    for (uint32_t t = 0U; t < HAPTIC_CFG_LINKLOSS_PULSE_MS; t += STEP_MS)
    {
        Haptic_Request(HAPTIC_SRC_GROUND, ALERT_NONE);
        Haptic_Step(t);
        TEST_ASSERT_TRUE(Haptic_IsMotorOn());
    }
}

/* Version C gap: caution must not mask a confirmed hazard */
void test_Haptic_HazardOutranksCaution(void)
{
    Haptic_Request(HAPTIC_SRC_GROUND, ALERT_SENSOR_FAULT);
    Haptic_Request(HAPTIC_SRC_LINK, ALERT_LINK_LOSS);
    Haptic_Step(0U);
    TEST_ASSERT_EQUAL(ALERT_SENSOR_FAULT, Haptic_GetActivePattern());

    Haptic_Request(HAPTIC_SRC_GROUND, ALERT_CLIFF_WARNING);
    Haptic_Step(10U);
    TEST_ASSERT_EQUAL(ALERT_CLIFF_WARNING, Haptic_GetActivePattern());
}

/* The calibration prompt never hides a link-loss warning */
void test_Haptic_CalibrationPromptIsLowestPriority(void)
{
    Haptic_Request(HAPTIC_SRC_GROUND, ALERT_CALIBRATE_PROMPT);
    Haptic_Step(0U);
    TEST_ASSERT_EQUAL(ALERT_CALIBRATE_PROMPT, Haptic_GetActivePattern());

    Haptic_Request(HAPTIC_SRC_LINK, ALERT_LINK_LOSS);
    Haptic_Step(10U);
    TEST_ASSERT_EQUAL(ALERT_LINK_LOSS, Haptic_GetActivePattern());
}

/* Test plan (Benny/Clifton): releasing the grip stops all vibration; regripping resumes it */
void test_Haptic_GripReleaseSilencesEverything(void)
{
    Haptic_Request(HAPTIC_SRC_GROUND, ALERT_CLIFF_WARNING);
    Haptic_Request(HAPTIC_SRC_LINK, ALERT_LINK_LOSS);
    Haptic_Step(0U);
    TEST_ASSERT_TRUE(Haptic_IsMotorOn());

    Haptic_SetGripHeld(false);
    Pulse_Stats_t s = Measure(10U, 3000U);
    TEST_ASSERT_EQUAL_UINT32(0U, s.pulses);
    TEST_ASSERT_EQUAL(ALERT_NONE, Haptic_GetActivePattern());

    Haptic_SetGripHeld(true);
    Haptic_Step(3100U);
    TEST_ASSERT_EQUAL(ALERT_CLIFF_WARNING, Haptic_GetActivePattern());
    TEST_ASSERT_TRUE(Haptic_IsMotorOn());
}

/* Re-posting the same cue every ToF poll must not restart it and cut pulses short */
void test_Haptic_RepeatedRequestDoesNotRestartPattern(void)
{
    Haptic_Request(HAPTIC_SRC_GROUND, ALERT_CLIFF_WARNING);
    Haptic_Step(0U);

    /* 200 ms is in the gap after the first 150 ms pulse */
    Haptic_Request(HAPTIC_SRC_GROUND, ALERT_CLIFF_WARNING);
    Haptic_Step(200U);
    TEST_ASSERT_FALSE(Haptic_IsMotorOn());

    Haptic_Step(260U);
    TEST_ASSERT_TRUE(Haptic_IsMotorOn());
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_Haptic_CuesAreDistinct);
    RUN_TEST(test_Haptic_StepWarningPreemptsLinkLoss);
    RUN_TEST(test_Haptic_LevelGroundDoesNotCutLinkLoss);
    RUN_TEST(test_Haptic_HazardOutranksCaution);
    RUN_TEST(test_Haptic_CalibrationPromptIsLowestPriority);
    RUN_TEST(test_Haptic_GripReleaseSilencesEverything);
    RUN_TEST(test_Haptic_RepeatedRequestDoesNotRestartPattern);
    return UNITY_END();
}
