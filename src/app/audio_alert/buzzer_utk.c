/** @file buzzer_utk.c
 *
 * @brief BRING-UP ONLY: buzzer driver for the course template (uT-Kernel).
 *
 * @par
 * COPYRIGHT NOTICE: (c) 2026 Group 29. All rights reserved.
 *
 * Implements the buzzer.h interface of Brian's alert code with the template's
 * own PWM calls, because the template build does not link the Pico SDK
 * hardware_pwm and clocks modules that Brian's buzzer.c uses. The PWM counter
 * runs at 1 MHz (system clock 125 MHz, divider 125), so the output frequency
 * is 1 000 000 / (wrap + 1), the same as buzzer.c. Brian owns the audio driver
 * and may replace this file.
 *
 * This file only builds inside the course template (it is exported with the
 * other sources); the host tests use tests/harness/mock_buzzer.c instead.
 */

#include <stdint.h>

#include <sys/machine.h>
#include <tk/tkernel.h>
#include <bsp/libbsp.h>

// buzzer.h is NOT included on purpose: it pulls in the Pico SDK pico/types.h,
// whose size_t clashes with the template's tkernel.h. The three prototypes
// below match buzzer.h (uint is unsigned int); alert.c sees the real header.
void buzzer_init (unsigned int pin);
void buzzer_tone (uint32_t freq_hz);
void buzzer_tone_vol (uint32_t freq_hz, uint8_t duty_pct);

#define BUZZER_UTK_SYS_CLOCK_HZ    (125000000u)
#define BUZZER_UTK_COUNTER_HZ      (1000000u)
#define BUZZER_UTK_DIV_FRAC_BITS   (4u)
#define BUZZER_UTK_OFF_BELOW_HZ    (20u)
#define BUZZER_UTK_SLICE_STRIDE    (0x14u)

static unsigned int s_pin = 0u;

// The slice number of a GPIO pin: two pins per slice, eight slices.
static unsigned int
buzzer_utk_slice (unsigned int pin)
{
    return ((pin >> 1) & 0x07u);
}

/*!
 * @brief Set up the PWM slice of the buzzer pin and leave it silent.
 *
 * @param[in] pin  GPIO number of the passive buzzer (GP18 on the Maker Pi Pico).
 */
void
buzzer_init (unsigned int pin)
{
    const uint32_t div = (BUZZER_UTK_SYS_CLOCK_HZ / BUZZER_UTK_COUNTER_HZ)
                         << BUZZER_UTK_DIV_FRAC_BITS;
    const uint32_t div_reg = PWM_BASE + PWM_CHx_DIV +
                             (buzzer_utk_slice(pin) * BUZZER_UTK_SLICE_STRIDE);

    s_pin = pin;

    // The template releases the PWM block from reset only when USE_PTMR is 1
    // (kernel/sysdepend/pico_rp2040/hw_setting.c). It is 0 in this build, so
    // without this the PWM registers ignore every write and the pin stays idle.
    set_w(RESETS_RESET, RESETS_RESET_PWM);
    clr_w(RESETS_RESET, RESETS_RESET_PWM);
    while (0u == (in_w(RESETS_RESET_DONE) & RESETS_RESET_PWM))
    {
        // Wait until the PWM block is out of reset.
    }

    (void) pwm_set_pin(pin);
    out_w(div_reg, div);
    (void) pwm_set_cc(pin, 0u);
    (void) pwm_set_enabled(pin, TRUE);
}

/*!
 * @brief Play a tone, or stop.
 *
 * @param[in] freq_hz  Tone in Hz; below 20 Hz means silent.
 */
void
buzzer_tone (uint32_t freq_hz)
{
    // 50 % duty is the loudest on a passive buzzer.
    buzzer_tone_vol(freq_hz, 50u);
}

/*!
 * @brief Play a tone at a given loudness, or stop.
 *
 * @par
 * BRING-UP EDIT (Putra, 8 Nov): FR9's distance-as-loudness cue needs duty
 * control; buzzer.h previously only exposed the fixed 50 % (loudest) tone.
 * Brian owns the final driver; this keeps the original wrap/cc math and
 * only parameterises the duty that used to be hardcoded.
 *
 * @param[in] freq_hz   Tone in Hz; below 20 Hz means silent.
 * @param[in] duty_pct  0-50; 0 is silent, 50 is loudest (see buzzer_tone).
 */
void
buzzer_tone_vol (uint32_t freq_hz, uint8_t duty_pct)
{
    uint32_t wrap;

    if ((freq_hz < BUZZER_UTK_OFF_BELOW_HZ) || (0u == duty_pct))
    {
        (void) pwm_set_cc(s_pin, 0u);
        return;
    }

    wrap = (BUZZER_UTK_COUNTER_HZ / freq_hz) - 1u;
    (void) pwm_set_wrap(s_pin, wrap);
    (void) pwm_set_cc(s_pin, ((wrap + 1u) * (uint32_t) duty_pct) / 100u);
}

/*** end of file ***/
