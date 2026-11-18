/**
 * @file    buzzer.c
 * @brief   PWM driver for the buzzer.
 *
 * The system clock is divided down to a 1 MHz PWM counter, so the output
 * frequency is 1,000,000 / (wrap + 1).
 */

#include "buzzer.h"

#include "hardware/clocks.h"
#include "hardware/pwm.h"
#include "pico/stdlib.h"

#define PWM_COUNTER_HZ       1000000u
#define PWM_COUNTER_HZ_F     1000000.0f
#define TONE_SILENT_BELOW_HZ 20u
#define PWM_LEVEL_OFF        0u

static uint32_t g_pin   = 0u;
static uint32_t g_slice = 0u;

/**
 * @brief Sets up PWM on the given GPIO pin. The buzzer starts silent.
 * @param pin  GPIO number of the buzzer.
 */
void buzzer_init (uint32_t pin)
{
    float divider;

    g_pin = pin;
    gpio_set_function(pin, GPIO_FUNC_PWM);
    g_slice = pwm_gpio_to_slice_num(pin);

    /* Cast: the system clock is at most a few hundred MHz, which a float
     * holds with negligible rounding for a clock divider. */
    divider = (float)clock_get_hz(clk_sys) / PWM_COUNTER_HZ_F;
    pwm_set_clkdiv(g_slice, divider);
    pwm_set_gpio_level(pin, PWM_LEVEL_OFF);
    pwm_set_enabled(g_slice, true);
}

/**
 * @brief Plays a square wave with 50 percent duty, which is the loudest.
 * @param freq_hz  Frequency in Hz. Below 20 Hz means silent.
 */
void buzzer_tone (uint32_t freq_hz)
{
    uint32_t wrap;

    if (freq_hz < TONE_SILENT_BELOW_HZ)
    {
        pwm_set_gpio_level(g_pin, PWM_LEVEL_OFF);
    }
    else
    {
        wrap = (PWM_COUNTER_HZ / freq_hz) - 1u;

        /* Casts: freq_hz is at least 20 Hz, so wrap is at most 49999 and
         * fits in 16 bits. */
        pwm_set_wrap(g_slice, (uint16_t)wrap);
        pwm_set_gpio_level(g_pin, (uint16_t)((wrap + 1u) / 2u));
    }
}

/* end of buzzer.c */

