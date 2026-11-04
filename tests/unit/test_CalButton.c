#include "unity.h"
#include "drivers/CalButton/CalButton.h"
#include "drivers/CalButton/CalButton_Cfg.h"

/*
 * \file test_CalButton.c
 * \brief GP20 calibration button: debounce, 2 s hold, one event per press
 */

#define POLL_MS     (50U)       /* Cane_Main polls the button every 50 ms */
#define PRESSED     (false)     /* Active-low */
#define RELEASED    (true)

void setUp(void)
{
    CalButton_Init(&CalButton_Config_Cane);
}

void tearDown(void)
{
}

/* Polls the button at a fixed level; returns how many hold events fired and when the first one did */
static uint32_t Poll(bool level, uint32_t start_ms, uint32_t duration_ms, uint32_t * const p_first_ms)
{
    uint32_t events = 0U;
    for (uint32_t t = start_ms; t < (start_ms + duration_ms); t += POLL_MS)
    {
        if (CalButton_Update(level, t) == CALBTN_EVENT_GROUND_HOLD)
        {
            if ((events == 0U) && (p_first_ms != NULL))
            {
                *p_first_ms = t;
            }
            events++;
        }
    }
    return events;
}

void test_CalButton_Hold2s_FiresOnceAt2s(void)
{
    uint32_t first_ms = 0U;
    TEST_ASSERT_EQUAL_UINT32(1U, Poll(PRESSED, 0U, 10000U, &first_ms));
    TEST_ASSERT_EQUAL_UINT32(CALBTN_CFG_GROUND_HOLD_MS, first_ms);
}

void test_CalButton_ShortPress_NoEvent(void)
{
    TEST_ASSERT_EQUAL_UINT32(0U, Poll(PRESSED, 0U, 1500U, NULL));
    TEST_ASSERT_EQUAL_UINT32(0U, Poll(RELEASED, 1500U, 3000U, NULL));
}

/* Pull-up idle level never counts as a press */
void test_CalButton_NotPressed_NoEvent(void)
{
    TEST_ASSERT_EQUAL_UINT32(0U, Poll(RELEASED, 0U, 10000U, NULL));
}

/* Contact bounce faster than the debounce time is ignored */
void test_CalButton_Bounce_Ignored(void)
{
    uint32_t events = 0U;
    for (uint32_t t = 0U; t < 5000U; t += 10U)
    {
        bool level = (((t / 10U) % 2U) == 0U) ? PRESSED : RELEASED;
        if (CalButton_Update(level, t) == CALBTN_EVENT_GROUND_HOLD)
        {
            events++;
        }
    }
    TEST_ASSERT_EQUAL_UINT32(0U, events);
}

/* A new press after a release starts a new calibration */
void test_CalButton_ReleaseRearms(void)
{
    TEST_ASSERT_EQUAL_UINT32(1U, Poll(PRESSED, 0U, 2500U, NULL));
    TEST_ASSERT_EQUAL_UINT32(0U, Poll(RELEASED, 2500U, 500U, NULL));
    TEST_ASSERT_EQUAL_UINT32(1U, Poll(PRESSED, 3000U, 2500U, NULL));
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_CalButton_Hold2s_FiresOnceAt2s);
    RUN_TEST(test_CalButton_ShortPress_NoEvent);
    RUN_TEST(test_CalButton_NotPressed_NoEvent);
    RUN_TEST(test_CalButton_Bounce_Ignored);
    RUN_TEST(test_CalButton_ReleaseRearms);
    return UNITY_END();
}
