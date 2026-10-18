#include "alert.h"
#include "buzzer.h"

#define ENABLE_OBSTACLE_BEEP  0   /* 0 = no obstacle beeps on the glasses for now */
#define ENABLE_FAULT_BEEP     1

#define FAULT_TONE_HZ  1000u
#define OBST_MIN_MM    200
#define OBST_MAX_MM    2000

static uint32_t fault_mask = 0u;
static int32_t  obst_mm = -1;
static uint32_t last_freq = UINT32_MAX;

/* Only touch the PWM hardware when the tone actually changes. */
static void set_tone (uint32_t freq)
{
    if (freq != last_freq)
    {
        last_freq = freq;
        buzzer_tone(freq);
    }
}

void alert_init (uint pin)
{
    buzzer_init(pin);
    set_tone(0u);
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

    if ((ENABLE_FAULT_BEEP != 0) && (fault_mask != 0u))
    {
        freq = fault_freq(now_ms);
    }
    else if ((ENABLE_OBSTACLE_BEEP != 0) && (obst_mm > 0) && (obst_mm <= OBST_MAX_MM))
    {
        /* Closer = higher pitch and faster beeping. */
        uint32_t d      = (uint32_t)((obst_mm < OBST_MIN_MM) ? OBST_MIN_MM : obst_mm);
        uint32_t tone   = 1000u + (((uint32_t)OBST_MAX_MM - d) * 2u);
        uint32_t period = 150u + ((d * 400u) / (uint32_t)OBST_MAX_MM);

        if ((now_ms % period) < ((period * 6u) / 10u))
        {
            freq = tone;
        }
    }
    else
    {
        /* Nothing to play. */
    }
    set_tone(freq);
}
