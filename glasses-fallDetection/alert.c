/**
 * @file    alert.c
 * @brief   Audible alerts of the glasses node: fault tones and obstacle
 *          beeps.
 *
 * The stored values use the fewest bits that hold their range: the fault
 * mask is 8 bits, the obstacle distance 16 bits and the last tone 16 bits.
 */

#include "alert.h"

#include "buzzer.h"
#include "hardware/sync.h"

/* Feature switches. */
#define ENABLE_OBSTACLE_BEEP  0   /* 0 means no obstacle beeps for now. */
#define ENABLE_FAULT_BEEP     1

/* Fault tones. */
#define FAULT_TONE_HZ         1000u
#define FAULT_PERIOD_MS       1000u
#define IMU_TONE_ON_MS        500u
#define RADAR_BLIP1_END_MS    100u
#define RADAR_BLIP2_START_MS  200u
#define RADAR_BLIP2_END_MS    300u

/* Obstacle beeps. The distances are signed because 0 or less means none. */
#define OBST_MIN_MM           200
#define OBST_MAX_MM           2000
#define OBST_TONE_BASE_HZ     1000u
#define OBST_TONE_SLOPE       2u
#define OBST_PERIOD_BASE_MS   150u
#define OBST_PERIOD_SPAN_MS   400u
#define OBST_DUTY_NUM         6u
#define OBST_DUTY_DEN         10u

/* Marks that no tone has been sent to the buzzer yet. */
#define TONE_UNKNOWN          0xFFFFu

static volatile uint8_t  g_fault_mask = 0u;
static volatile int16_t  g_obst_mm    = -1;
static uint16_t          g_last_freq  = TONE_UNKNOWN;

static void     set_tone (uint16_t freq);
static uint32_t fault_bit (alert_fault_t source);
static uint16_t fault_freq (uint32_t fault_mask, uint32_t now_ms);
static uint16_t obstacle_freq (int32_t dist_mm, uint32_t now_ms);

/**
 * @brief Sets up the buzzer on the given GPIO pin and silences it.
 * @param pin  GPIO number of the buzzer.
 */
void alert_init (uint32_t pin)
{
    buzzer_init(pin);
    set_tone(0u);
}

/**
 * @brief Sets or clears one fault source.
 *
 * The mask is changed inside a critical section, because the new value
 * depends on the old one and a callback may change it too.
 *
 * @param source    Fault source to change.
 * @param b_active  true to set the fault, false to clear it.
 */
void alert_set_fault (alert_fault_t source, bool b_active)
{
    uint32_t irq_state = save_and_disable_interrupts();
    uint32_t mask      = g_fault_mask;

    if (b_active)
    {
        mask |= fault_bit(source);
    }
    else
    {
        mask &= ~fault_bit(source);
    }

    /* Cast: every alert_fault_t value fits in 8 bits, so the mask does too. */
    g_fault_mask = (uint8_t)mask;

    restore_interrupts(irq_state);
}

/**
 * @brief Sets the obstacle distance.
 *
 * The value is stored in 16 bits, so it is limited to the range of an
 * int16_t first.
 *
 * @param dist_mm  Distance in mm. 0 or less means no obstacle.
 */
void alert_set_obstacle (int32_t dist_mm)
{
    int32_t limited = dist_mm;

    if (limited > INT16_MAX)
    {
        limited = INT16_MAX;
    }
    else if (limited < INT16_MIN)
    {
        limited = INT16_MIN;
    }
    else
    {
        /* The value already fits in 16 bits. */
    }

    /* Cast: the value was limited to the range of an int16_t above. */
    g_obst_mm = (int16_t)limited;
}

/**
 * @brief  Returns the active fault sources.
 * @return Bit mask of alert_fault_t values.
 */
uint8_t alert_get_fault_mask (void)
{
    return g_fault_mask;
}

/**
 * @brief Chooses the tone for the current time and sends it to the buzzer.
 *
 * Faults take priority over obstacle beeps.
 *
 * @param now_ms  Milliseconds since boot.
 */
void alert_update (uint32_t now_ms)
{
    uint16_t freq = 0u;
    uint32_t mask = g_fault_mask;
    int32_t  dist = g_obst_mm;

    if ((0 != ENABLE_FAULT_BEEP) && (0u != mask))
    {
        freq = fault_freq(mask, now_ms);
    }
    else if ((0 != ENABLE_OBSTACLE_BEEP) && (dist > 0) &&
             (dist <= OBST_MAX_MM))
    {
        freq = obstacle_freq(dist, now_ms);
    }
    else
    {
        /* Nothing to play. */
    }

    set_tone(freq);
}

/**
 * @brief Sends a tone to the buzzer, but only when it has changed.
 * @param freq  Tone in Hz, or 0 for silence.
 */
static void set_tone (uint16_t freq)
{
    if (freq != g_last_freq)
    {
        g_last_freq = freq;
        buzzer_tone(freq);
    }
}

/**
 * @brief  Converts a fault source into its mask bit.
 * @param  source  Fault source.
 * @return The bit of the source in the fault mask.
 */
static uint32_t fault_bit (alert_fault_t source)
{
    /* Cast: the enumerators are small positive constants, so the value is
     * not changed by the conversion. */
    return (uint32_t)source;
}

/**
 * @brief  Chooses the fault tone for the current time.
 * @param  fault_mask  Active fault sources, not zero.
 * @param  now_ms      Milliseconds since boot.
 * @return Tone in Hz, or 0 for silence.
 */
static uint16_t fault_freq (uint32_t fault_mask, uint32_t now_ms)
{
    uint32_t phase = now_ms % FAULT_PERIOD_MS;
    uint16_t freq  = 0u;

    if (0u != (fault_mask & fault_bit(ALERT_FAULT_IMU)))
    {
        /* IMU fault: one long tone per second, like the 1 Hz LED. */
        if (phase < IMU_TONE_ON_MS)
        {
            freq = FAULT_TONE_HZ;
        }
    }
    else if (0u != (fault_mask & fault_bit(ALERT_FAULT_RADAR)))
    {
        /* Radar fault (draft pattern): two short blips per second. */
        if ((phase < RADAR_BLIP1_END_MS) ||
            ((phase >= RADAR_BLIP2_START_MS) && (phase < RADAR_BLIP2_END_MS)))
        {
            freq = FAULT_TONE_HZ;
        }
    }
    else
    {
        /* TODO: Define tones for the TOF and BLE loss faults. */
    }

    return freq;
}

/**
 * @brief  Chooses the obstacle beep for the current time.
 *
 * A closer obstacle gives a higher pitch and faster beeping.
 *
 * @param  dist_mm  Distance in mm, from 1 to OBST_MAX_MM.
 * @param  now_ms   Milliseconds since boot.
 * @return Tone in Hz, or 0 for silence.
 */
static uint16_t obstacle_freq (int32_t dist_mm, uint32_t now_ms)
{
    int32_t  clamped = dist_mm;
    uint32_t dist;
    uint32_t tone;
    uint32_t period;
    uint16_t freq = 0u;

    if (clamped < OBST_MIN_MM)
    {
        clamped = OBST_MIN_MM;
    }

    /* Cast: clamped is at least OBST_MIN_MM here, so it is positive. */
    dist   = (uint32_t)clamped;
    tone   = OBST_TONE_BASE_HZ + (((uint32_t)OBST_MAX_MM - dist) *
                                  OBST_TONE_SLOPE);
    period = OBST_PERIOD_BASE_MS + ((dist * OBST_PERIOD_SPAN_MS) /
                                    (uint32_t)OBST_MAX_MM);

    if ((now_ms % period) < ((period * OBST_DUTY_NUM) / OBST_DUTY_DEN))
    {
        /* Cast: the tone is at most 1000 + (2000 - 200) * 2 = 4600 Hz. */
        freq = (uint16_t)tone;
    }

    return freq;
}

/* end of alert.c */

