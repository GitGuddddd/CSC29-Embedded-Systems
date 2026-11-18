/**
 * @file    buzzer.h
 * @brief   PWM driver for the buzzer.
 */

#ifndef BUZZER_H
#define BUZZER_H

#include <stdint.h>

/* Sets up PWM on the given GPIO pin. The buzzer starts silent. */
void buzzer_init (uint32_t pin);

/* Plays a square wave of the given frequency in Hz. A frequency below
 * 20 Hz, including 0, means silent. */
void buzzer_tone (uint32_t freq_hz);

#endif /* BUZZER_H */

/* end of buzzer.h */

