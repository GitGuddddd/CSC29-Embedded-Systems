/** @file mock_buzzer.c
 *
 * @brief Host-test buzzer: records the last tone and how often it changed.
 *
 * @par
 * COPYRIGHT NOTICE: (c) 2026 Group 29. All rights reserved.
 */

#include <stdint.h>

#include "buzzer.h"
#include "mock_buzzer.h"

static uint32_t g_last_tone_hz = 0u;
static uint32_t g_tone_calls = 0u;

void
buzzer_init (uint pin)
{
    (void) pin;
}

void
buzzer_tone (uint32_t freq_hz)
{
    g_last_tone_hz = freq_hz;
    g_tone_calls++;
}

void
mock_buzzer_reset (void)
{
    g_last_tone_hz = 0u;
    g_tone_calls = 0u;
}

uint32_t
mock_buzzer_last_tone_hz (void)
{
    return g_last_tone_hz;
}

uint32_t
mock_buzzer_tone_calls (void)
{
    return g_tone_calls;
}

/*** end of file ***/
