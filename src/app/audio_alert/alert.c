#include "alert.h"
#include "buzzer.h"
#include "feature_cfg.h"   /* BRING-UP EDIT (Putra, 2 Nov): see ENABLE_OBSTACLE_BEEP below */

/* BRING-UP EDIT (Putra, 2 Nov): was a plain "#define ENABLE_OBSTACLE_BEEP 0".
 * The default is still 0; config/feature_cfg.h can set it to 1 so the radar
 * alerts can be heard. Brian owns this file and decides the final value. */
#ifndef ENABLE_OBSTACLE_BEEP
#define ENABLE_OBSTACLE_BEEP  0   /* 0 = no obstacle beeps on the glasses for now */
#endif
#define ENABLE_FAULT_BEEP     1

#define FAULT_TONE_HZ  1000u
#define OBST_MIN_MM    200
#define OBST_MAX_MM    2000

/* BRING-UP EDIT (Putra, 8 Nov): distance now rides on loudness instead of
 * pulse rate (DRAFT, not bench-verified: duty_pct is linear in distance,
 * untested whether that reads as a smooth loudness ramp on the real piezo,
 * see docs/deviations.md OPEN-14). 50 is buzzer_tone's loudest; see
 * buzzer.h. */
#define ALERT_MIN_DUTY_PCT   (10u)
#define ALERT_MAX_DUTY_PCT   (50u)

/* BRING-UP EDIT (Putra, 8 Nov): fixed beep timing for LEFT/RIGHT, no longer
 * distance-derived now that distance is loudness. DRAFT, untuned. */
#define ALERT_BEEP_PULSE_MS  (150u)
#define ALERT_BEEP_GAP_MS    (150u)

/* BRING-UP EDIT (Putra, 11 Nov): the pause AFTER a full beep burst now also
 * rides on distance (DRAFT, not bench-verified), same linear scaling as
 * loudness below - the closer the obstacle, the shorter the pause, so the
 * same beep pattern repeats faster instead of only getting louder. Beep
 * *count* (2 vs 3, i.e. direction) is unchanged by this: only the gap after
 * a complete burst shrinks. See docs/deviations.md OPEN-21. */
#define ALERT_BEEP_PAUSE_MIN_MS  (150u)
#define ALERT_BEEP_PAUSE_MAX_MS  (450u)

static uint32_t fault_mask = 0u;
static int32_t  obst_mm = -1;
static uint32_t last_freq = UINT32_MAX;
static uint32_t last_duty = UINT32_MAX;
/* BRING-UP EDIT (Putra, 8 Nov): FR9 cue byte, see alert_set_cue() below. */
static uint8_t  cue_bits = 0u;

/* Only touch the PWM hardware when the tone or its loudness actually
 * changes. */
static void set_tone (uint32_t freq, uint32_t duty_pct)
{
    if ((freq != last_freq) || (duty_pct != last_duty))
    {
        last_freq = freq;
        last_duty = duty_pct;
        buzzer_tone_vol(freq, (uint8_t) duty_pct);
    }
}

void alert_init (uint pin)
{
    buzzer_init(pin);
    set_tone(0u, 0u);
}

void alert_set_fault (alert_fault_t source, bool active)
{
    if (active)
    {
        fault_mask |= (uint32_t)source;
    }
    else
    {
        fault_mask &= ~(uint32_t)source;
    }
}

void alert_set_obstacle (int32_t dist_mm)
{
    obst_mm = dist_mm;
}

/* BRING-UP EDIT (Putra, 8 Nov): stores the FR9 cue byte for alert_update()
 * to read. Threat sends 0 (no STOP bit, side CENTER) while cue_bits is at
 * its reset value, so a cue byte Threat never sends (fault, clear, or before
 * the first obstacle) behaves exactly as the pre-cue code did. Brian owns
 * the final sound design; see the cue handling in alert_update() below. */
void alert_set_cue (uint8_t cue)
{
    cue_bits = cue;
}

static uint32_t fault_freq (uint32_t now_ms)
{
    uint32_t phase = now_ms % 1000u;
    uint32_t freq = 0u;

    if ((fault_mask & (uint32_t)ALERT_FAULT_IMU) != 0u)
    {
        /* IMU fault: one long 1 kHz tone per second (matches the 1 Hz LED). */
        if (phase < 500u)
        {
            freq = FAULT_TONE_HZ;
        }
    }
    else
    {
        /* Radar fault (draft pattern): two short 1 kHz blips per second. */
        if ((phase < 100u) || ((phase >= 200u) && (phase < 300u)))
        {
            freq = FAULT_TONE_HZ;
        }
    }
    return freq;
}

void alert_update (uint32_t now_ms)
{
    uint32_t freq = 0u;
    uint32_t duty = 0u;

    if ((ENABLE_FAULT_BEEP != 0) && (fault_mask != 0u))
    {
        freq = fault_freq(now_ms);
        duty = ALERT_MAX_DUTY_PCT;
    }
    else if ((ENABLE_OBSTACLE_BEEP != 0) && (obst_mm > 0) && (obst_mm <= OBST_MAX_MM))
    {
        /* BRING-UP EDIT (Putra, 8 Nov), FR9 cue byte, redesigned so side is
         * always on instead of only near STOP (Putra's call, 8 Nov): pitch
         * still tracks distance as before, but distance's old job of
         * speeding up the pulse now speaks through loudness instead, so the
         * pulse timing is free to mean side (confirmed design B, 8 Nov):
         *   - CENTER: one continuous tone, louder when close. No pulsing.
         *   - LEFT / RIGHT: a burst of 2 or 3 short pulses (fixed timing,
         *     so beep count still means direction), then a pause that
         *     shrinks as distance shrinks (11 Nov, see ALERT_BEEP_PAUSE_*
         *     below) - closer repeats the same pattern faster, on top of
         *     the loudness/pitch ramp distance already gives.
         *   - STOP: forces the loudest volume AND the shortest pause
         *     regardless of distance, on top of whichever of the above the
         *     side already selected (so STOP LEFT is a loud, fast 2-beep
         *     burst, not a different pattern).
         * Brian owns the final sound design. */
        uint32_t d      = (uint32_t)((obst_mm < OBST_MIN_MM) ? OBST_MIN_MM : obst_mm);
        uint32_t tone   = 1000u + (((uint32_t)OBST_MAX_MM - d) * 2u);
        uint32_t vol    = ALERT_MIN_DUTY_PCT +
                          ((((uint32_t)OBST_MAX_MM - d) *
                            (ALERT_MAX_DUTY_PCT - ALERT_MIN_DUTY_PCT)) /
                           ((uint32_t)OBST_MAX_MM - (uint32_t)OBST_MIN_MM));
        uint32_t side     = ((uint32_t) cue_bits >> ALERT_CUE_SIDE_SHIFT) &
                            ALERT_CUE_SIDE_MASK;
        bool     b_stop   = (0u != ((uint32_t) cue_bits & ALERT_CUE_STOP));
        bool     b_on;
        /* BRING-UP EDIT (Putra, 11 Nov): pause shrinks as d shrinks, same
         * linear scaling as vol above; STOP forces the shortest pause (on
         * top of whichever side pattern is already playing), same way it
         * forces the loudest volume. */
        uint32_t pause_ms = ALERT_BEEP_PAUSE_MAX_MS -
                            ((((uint32_t) OBST_MAX_MM - d) *
                              (ALERT_BEEP_PAUSE_MAX_MS -
                               ALERT_BEEP_PAUSE_MIN_MS)) /
                             ((uint32_t) OBST_MAX_MM - (uint32_t) OBST_MIN_MM));

        if (b_stop)
        {
            vol = ALERT_MAX_DUTY_PCT;
            pause_ms = ALERT_BEEP_PAUSE_MIN_MS;
        }

        if (ALERT_CUE_SIDE_CENTER == side)
        {
            b_on = true;
        }
        else
        {
            uint32_t beeps    = (ALERT_CUE_SIDE_LEFT == side) ? 2u : 3u;
            uint32_t slot_ms  = ALERT_BEEP_PULSE_MS + ALERT_BEEP_GAP_MS;
            uint32_t burst_ms = slot_ms * beeps;
            uint32_t cycle_ms = burst_ms + pause_ms;
            uint32_t phase    = now_ms % cycle_ms;

            b_on = (phase < burst_ms) &&
                   ((phase % slot_ms) < ALERT_BEEP_PULSE_MS);
        }

        if (b_on)
        {
            freq = tone;
            duty = vol;
        }
    }
    else
    {
        /* Nothing to play. */
    }
    set_tone(freq, duty);
}

/*** end of file ***/
