#include "Haptic.h"
#include "Haptic_Cfg.h"
#include "hal/Hal_Pwm.h"
#include <stddef.h>

/*
 * \file Haptic.c
 * \brief Non-blocking pattern player and arbiter for the cane vibration motor
 * Compliance: Barr-C:2018
 */

typedef struct
{
    uint8_t pulses;
    uint16_t pulse_ms;
    uint16_t gap_ms;
    uint16_t period_ms;
} Haptic_Pattern_t;

static uint8_t s_haptic_pin = 15U;
static Alert_Type_t s_requests[HAPTIC_SRC_COUNT];
static bool s_grip_held = false;
static Alert_Type_t s_active_pattern = ALERT_NONE;
static uint32_t s_pattern_start_ms = 0U;
static bool s_motor_on = false;

static Alert_Type_t s_oneshot = ALERT_NONE;
static uint32_t s_oneshot_start_ms = 0U;
static uint32_t s_oneshot_len_ms = 0U;

static bool Get_Pattern(Alert_Type_t type, Haptic_Pattern_t * const p_out)
{
    switch (type)
    {
        case ALERT_CLIFF_WARNING:
            *p_out = (Haptic_Pattern_t){ HAPTIC_CFG_CLIFF_PULSES, HAPTIC_CFG_CLIFF_PULSE_MS,
                                         HAPTIC_CFG_CLIFF_GAP_MS, HAPTIC_CFG_CLIFF_PERIOD_MS };
            return true;
        case ALERT_CLIFF_STEP_UP:
            *p_out = (Haptic_Pattern_t){ HAPTIC_CFG_STEPUP_PULSES, HAPTIC_CFG_STEPUP_PULSE_MS,
                                         HAPTIC_CFG_STEPUP_GAP_MS, HAPTIC_CFG_STEPUP_PERIOD_MS };
            return true;
        case ALERT_SENSOR_FAULT:
            *p_out = (Haptic_Pattern_t){ HAPTIC_CFG_CAUTION_PULSES, HAPTIC_CFG_CAUTION_PULSE_MS,
                                         HAPTIC_CFG_CAUTION_GAP_MS, HAPTIC_CFG_CAUTION_PERIOD_MS };
            return true;
        case ALERT_LINK_LOSS:
            *p_out = (Haptic_Pattern_t){ HAPTIC_CFG_LINKLOSS_PULSES, HAPTIC_CFG_LINKLOSS_PULSE_MS,
                                         HAPTIC_CFG_LINKLOSS_GAP_MS, HAPTIC_CFG_LINKLOSS_PERIOD_MS };
            return true;
        case ALERT_CALIBRATE_PROMPT:
            *p_out = (Haptic_Pattern_t){ HAPTIC_CFG_CALPROMPT_PULSES, HAPTIC_CFG_CALPROMPT_PULSE_MS,
                                         HAPTIC_CFG_CALPROMPT_GAP_MS, HAPTIC_CFG_CALPROMPT_PERIOD_MS };
            return true;
        case ALERT_CAL_STARTED:
            *p_out = (Haptic_Pattern_t){ HAPTIC_CFG_CALSTART_PULSES, HAPTIC_CFG_CALSTART_PULSE_MS,
                                         HAPTIC_CFG_CALSTART_GAP_MS, HAPTIC_CFG_CALSTART_PERIOD_MS };
            return true;
        case ALERT_CAL_DONE:
            *p_out = (Haptic_Pattern_t){ HAPTIC_CFG_CALDONE_PULSES, HAPTIC_CFG_CALDONE_PULSE_MS,
                                         HAPTIC_CFG_CALDONE_GAP_MS, HAPTIC_CFG_CALDONE_PERIOD_MS };
            return true;
        case ALERT_CAL_FAILED:
            *p_out = (Haptic_Pattern_t){ HAPTIC_CFG_CALFAIL_PULSES, HAPTIC_CFG_CALFAIL_PULSE_MS,
                                         HAPTIC_CFG_CALFAIL_GAP_MS, HAPTIC_CFG_CALFAIL_PERIOD_MS };
            return true;
        default:
            return false;
    }
}

/* Higher value wins. Cues the cane cannot play rank 0. */
static uint8_t Get_Priority(Alert_Type_t type)
{
    switch (type)
    {
        case ALERT_CLIFF_WARNING:    return 7U;
        case ALERT_CLIFF_STEP_UP:    return 6U;
        case ALERT_CAL_STARTED:
        case ALERT_CAL_DONE:
        case ALERT_CAL_FAILED:       return 5U;
        case ALERT_SENSOR_FAULT:     return 4U;
        case ALERT_LINK_LOSS:        return 3U;
        case ALERT_CALIBRATE_PROMPT: return 2U;
        default:                     return 0U;
    }
}

static void Set_Motor(bool on)
{
    if (on == s_motor_on)
    {
        return;
    }

    s_motor_on = on;
    if (on)
    {
        Hal_Pwm_SetFrequency(s_haptic_pin, HAPTIC_CFG_PWM_FREQ_HZ, HAPTIC_CFG_PWM_DUTY_PCT);
    }
    else
    {
        Hal_Pwm_Stop(s_haptic_pin);
    }
}

void Haptic_Init(const Haptic_Config_t * const p_cfg)
{
    if (p_cfg != NULL)
    {
        s_haptic_pin = p_cfg->gpio_pin;
    }
    Hal_Pwm_Init(s_haptic_pin, HAPTIC_CFG_PWM_FREQ_HZ, 0U);
    Hal_Pwm_Stop(s_haptic_pin);

    for (uint32_t i = 0U; i < (uint32_t)HAPTIC_SRC_COUNT; ++i)
    {
        s_requests[i] = ALERT_NONE;
    }
    s_grip_held = false;
    s_active_pattern = ALERT_NONE;
    s_pattern_start_ms = 0U;
    s_motor_on = false;
    s_oneshot = ALERT_NONE;
    s_oneshot_start_ms = 0U;
    s_oneshot_len_ms = 0U;
}

void Haptic_Request(Haptic_Source_t source, Alert_Type_t pattern_type)
{
    if (source < HAPTIC_SRC_COUNT)
    {
        s_requests[source] = pattern_type;
    }
}

void Haptic_PlayOnce(Alert_Type_t pattern_type, uint32_t now_ms)
{
    Haptic_Pattern_t pat;
    if (!Get_Pattern(pattern_type, &pat))
    {
        return;
    }

    /* A new one-shot replaces any unfinished one */
    s_oneshot = pattern_type;
    s_oneshot_start_ms = now_ms;
    s_oneshot_len_ms = ((uint32_t)pat.pulses * pat.pulse_ms) +
                       ((uint32_t)(pat.pulses - 1U) * pat.gap_ms);
}

void Haptic_SetGripHeld(bool held)
{
    s_grip_held = held;
}

void Haptic_Step(uint32_t now_ms)
{
    Alert_Type_t winner = ALERT_NONE;
    uint8_t best = 0U;

    /* A one-shot cue expires after it has played once, whether or not it was felt */
    if ((s_oneshot != ALERT_NONE) &&
        ((uint32_t)(now_ms - s_oneshot_start_ms) >= s_oneshot_len_ms))
    {
        s_oneshot = ALERT_NONE;
    }

    if (s_grip_held)
    {
        for (uint32_t i = 0U; i < (uint32_t)HAPTIC_SRC_COUNT; ++i)
        {
            uint8_t prio = Get_Priority(s_requests[i]);
            if (prio > best)
            {
                best = prio;
                winner = s_requests[i];
            }
        }

        if (Get_Priority(s_oneshot) > best)
        {
            best = Get_Priority(s_oneshot);
            winner = s_oneshot;
        }
    }

    /* Restart only when the winning cue changes, so a repeated request never cuts a pulse short */
    if (winner != s_active_pattern)
    {
        s_active_pattern = winner;
        s_pattern_start_ms = now_ms;
    }

    Haptic_Pattern_t pat;
    if (!Get_Pattern(s_active_pattern, &pat))
    {
        Set_Motor(false);
        return;
    }

    /* A one-shot keeps its own timeline, so pre-empting it never replays it from the start */
    uint32_t start_ms = (s_active_pattern == s_oneshot) ? s_oneshot_start_ms : s_pattern_start_ms;
    uint32_t t = (uint32_t)(now_ms - start_ms) % pat.period_ms;
    uint32_t slot_ms = (uint32_t)pat.pulse_ms + (uint32_t)pat.gap_ms;
    bool on = ((t / slot_ms) < pat.pulses) && ((t % slot_ms) < pat.pulse_ms);

    Set_Motor(on);
}

Alert_Type_t Haptic_GetActivePattern(void)
{
    return s_active_pattern;
}

bool Haptic_IsMotorOn(void)
{
    return s_motor_on;
}
