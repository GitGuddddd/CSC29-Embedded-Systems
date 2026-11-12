#ifndef BUZZER_H
#define BUZZER_H

#include <stdint.h>
#include "pico/types.h"

void buzzer_init(uint pin);
void buzzer_tone(uint32_t freq_hz);   // 0 = silent, always loudest (50% duty)

/* BRING-UP EDIT (Putra, 8 Nov): loudness-as-distance needs duty control.
 * duty_pct 1-50 (50 = loudest, see buzzer_tone); 0 or freq_hz 0 = silent. */
void buzzer_tone_vol(uint32_t freq_hz, uint8_t duty_pct);

#endif